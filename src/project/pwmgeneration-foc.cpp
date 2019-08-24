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

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)

static s32fp idref = 0;
static int initwait = 0;
static int32_t maxVd = 0;
static PiController qController;
static PiController dController;

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN || opmode == MOD_SINE)
   {
      int dir = Param::GetInt(Param::dir);
      uint16_t dc[3];
      s32fp id, iq;
      s32fp fweak = Param::Get(Param::fweak);

      Encoder::UpdateRotorAngle(dir);

      if (opmode == MOD_SINE)
         CalcNextAngleConstant(dir);
      else if (Encoder::IsSyncMode())
         CalcNextAngleSync(dir);
      else
         CalcNextAngleAsync(dir);

      if (frq > fweak)
      {
         s32fp idweak = FP_MUL(Param::Get(Param::idweak), frq - fweak);
         dController.Setpoint(idweak + idref);
      }
      else
      {
         dController.Setpoint(idref);
      }

      ProcessCurrents(id, iq);

      int32_t ud = dController.Run(id);
      int32_t uq = qController.Run(iq);
      int32_t qlimit = FOC::GetQLimit(maxVd);
      qController.SetMinMaxY(-qlimit, qlimit);
      FOC::InvParkClarke(ud, uq, angle);

      s32fp idc = (iq * uq) / FOC::GetMaximumModulationIndex();

      Param::SetFlt(Param::fstat, frq);
      Param::SetFlt(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetInt(Param::ud, ud);
      Param::SetInt(Param::uq, uq);
      Param::SetFlt(Param::idc, idc);

      /* Shut down PWM on stopped motor, neutral gear or init phase */
      if ((0 == frq && 0 == idref) || 0 == dir || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         dController.ResetIntegrator();
         qController.ResetIntegrator();
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      for (int i = 0; i < 3; i++)
      {
         dc[i] = FOC::DutyCycles[i] >> shiftForTimer;
         Param::SetInt((Param::PARAM_NUM)(Param::dc1+i), dc[i]);
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
   s32fp id = FP_MUL(Param::Get(Param::throtid), ABS(torquePercent));
   s32fp iq = FP_MUL(Param::Get(Param::throtiq), Param::GetInt(Param::dir) * torquePercent);

   idref = id; //d-regulator set point is programmed in Run()
   qController.Setpoint(iq);
}

void PwmGeneration::SetControllerGains(int dkp, int dki, int qkp, int qki)
{
   qController.SetGains(qkp, qki);
   dController.SetGains(dkp, dki);
}

void PwmGeneration::PwmInit()
{
   maxVd = FOC::GetMaximumModulationIndex() - 4000;
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = 5000;
   qController.ResetIntegrator();
   qController.SetCallingFrequency(pwmfrq);
   qController.SetMinMaxY(-maxVd, maxVd);
   dController.ResetIntegrator();
   dController.SetCallingFrequency(pwmfrq);
   dController.SetMinMaxY(-maxVd, maxVd);

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
