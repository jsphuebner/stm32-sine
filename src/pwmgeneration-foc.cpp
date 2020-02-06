/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2015 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "pwmgeneration.h"
#include "hwdefs.h"
#include "params.h"
#include "inc_encoder.h"
#include "sine_core.h"
#include "fu.h"
#include "errormessage.h"
#include "digio.h"
#include "anain.h"
#include "my_math.h"
#include "foc.h"
#include "picontroller.h"

#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)

static int initwait = 0;
static s32fp idref = 0;
static int curki = 0;
static PiController qController;
static PiController dController;
static PiController fwController;

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN)
   {
      int dir = Param::GetInt(Param::dir);
      int kifrqgain = Param::GetInt(Param::curkifrqgain);
      uint16_t dc[3];
      s32fp id, iq;

      Encoder::UpdateRotorAngle(dir);

      if (Encoder::IsSyncMode())
         CalcNextAngleSync(dir);
      else
         CalcNextAngleAsync(dir);

      int moddedKi = curki + kifrqgain * FP_TOINT(frq);

      qController.SetIntegralGain(moddedKi);
      dController.SetIntegralGain(moddedKi);

      ProcessCurrents(id, iq);

      if (opmode == MOD_RUN)
      {
         s32fp fwIdRef = fwController.Run(iq);
         dController.SetRef(idref + fwIdRef);
      }
      else if (opmode == MOD_MANUAL)
      {
         idref = Param::Get(Param::manualid);
         dController.SetRef(idref);
         qController.SetRef(Param::Get(Param::manualiq));
      }

      int32_t ud = dController.Run(id);
      int32_t qlimit = FOC::GetQLimit(ud);
      qController.SetMinMaxY(-qlimit, qlimit);
      int32_t uq = qController.Run(iq);
      FOC::InvParkClarke(ud, uq, angle);

      s32fp idc = (iq * uq) / FOC::GetMaximumModulationIndex();

      Param::SetFlt(Param::fstat, frq);
      Param::SetFlt(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetInt(Param::ud, ud);
      Param::SetInt(Param::uq, uq);
      Param::SetFlt(Param::idc, idc);
      Param::SetFlt(Param::dspnt, dController.GetRef());
      Param::SetFlt(Param::qspnt, qController.GetRef());

      /* Shut down PWM on stopped motor, neutral gear or init phase */
      if ((0 == frq && 0 == idref) || 0 == dir || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         dController.ResetIntegrator();
         qController.ResetIntegrator();
         fwController.ResetIntegrator();
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      for (int i = 0; i < 3; i++)
      {
         dc[i] = FOC::DutyCycles[i] >> shiftForTimer;
      }

      if ((Param::GetInt(Param::pinswap) & SWAP_PWM) > 0)
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC1, dc[2]);
         timer_set_oc_value(PWM_TIMER, TIM_OC2, dc[1]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, dc[0]);
      }
      else
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC1, dc[0]);
         timer_set_oc_value(PWM_TIMER, TIM_OC2, dc[1]);
         timer_set_oc_value(PWM_TIMER, TIM_OC3, dc[2]);
      }
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      s32fp id, iq;
      initwait = 0;
      ProcessCurrents(id, iq);
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      initwait = 0;
      AcHeat();
   }
}

void PwmGeneration::SetTorquePercent(s32fp torquePercent)
{
   s32fp brkrampstr = Param::Get(Param::brkrampstr);
   int direction = Param::GetInt(Param::dir);

   if (frq < brkrampstr && torquePercent < 0)
   {
      torquePercent = FP_MUL(FP_DIV(frq, brkrampstr), torquePercent);
   }

   if (torquePercent < 0)
   {
      direction = Encoder::GetRotorDirection();
   }

   int32_t is = FP_TOINT(FP_MUL(Param::Get(Param::throtcur), direction * torquePercent));
   int32_t id, iq;

   FOC::Mtpa(is, id, iq);

   qController.SetRef(FP_FROMINT(iq));
   fwController.SetRef(FP_FROMINT(iq));
   idref = FP_FROMINT(id);
}

void PwmGeneration::SetControllerGains(int kp, int ki, int fwkp)
{
   qController.SetGains(kp, ki);
   dController.SetGains(kp, ki);
   fwController.SetGains(fwkp, 0);
   curki = ki;
}

void PwmGeneration::PwmInit()
{
   int32_t maxVd = FOC::GetMaximumModulationIndex() + Param::GetInt(Param::dmargin);
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = pwmfrq / 2; //0.5s
   qController.ResetIntegrator();
   qController.SetCallingFrequency(pwmfrq);
   qController.SetMinMaxY(-maxVd, maxVd);
   dController.ResetIntegrator();
   dController.SetCallingFrequency(pwmfrq);
   dController.SetMinMaxY(-maxVd, maxVd);
   fwController.ResetIntegrator();
   fwController.SetCallingFrequency(pwmfrq);
   fwController.SetMinMaxY(-50 * Param::Get(Param::throtcur), 0); //allow 50% of max current for extra field weakening

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::ProcessCurrents(s32fp& id, s32fp& iq)
{
   if (initwait > 0)
   {
      initwait--;
      SetCurrentOffset(AnaIn::Get(AnaIn::il1), AnaIn::Get(AnaIn::il2));
   }
   else
   {
      s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
      s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

      if ((Param::GetInt(Param::pinswap) & SWAP_CURRENTS) > 0)
         FOC::ParkClarke(il2, il1, angle);
      else
         FOC::ParkClarke(il1, il2, angle);
      id = FOC::id;
      iq = FOC::iq;

      Param::SetFlt(Param::id, FOC::id);
      Param::SetFlt(Param::iq, FOC::iq);
      Param::SetFlt(Param::il1, il1);
      Param::SetFlt(Param::il2, il2);
   }

   return 0;
}
