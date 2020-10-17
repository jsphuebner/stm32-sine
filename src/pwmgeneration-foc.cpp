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
static int fwBaseGain = 0;
static s32fp idref = 0;
static int curki = 0;
static int idleCounter = 0;
static tim_oc_id ocChannels[3];
static PiController qController;
static PiController dController;
static PiController fwController;

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN)
   {
      static s32fp frqFiltered;
      int dir = Param::GetInt(Param::dir);
      int kifrqgain = Param::GetInt(Param::curkifrqgain);
      s32fp id, iq;

      Encoder::UpdateRotorAngle(dir);

      CalcNextAngleSync(dir);

      frqFiltered = IIRFILTER(frqFiltered, frq, 8);
      int moddedKi = curki + kifrqgain * FP_TOINT(frqFiltered);

      qController.SetIntegralGain(moddedKi);
      dController.SetIntegralGain(moddedKi);
      fwController.SetProportionalGain(fwBaseGain * dir);

      ProcessCurrents(id, iq);

      if (opmode == MOD_RUN && initwait == 0)
      {
         s32fp fwIdRef = idref <= 0 ? fwController.Run(iq) : 0;
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

      //This is probably not correct for IPM motors
      s32fp idc = (iq * uq) / FOC::GetMaximumModulationIndex();

      Param::SetFlt(Param::fstat, frq);
      Param::SetFlt(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetFlt(Param::idc, idc);
      Param::SetInt(Param::uq, uq);
      Param::SetInt(Param::ud, ud);

      /* Shut down PWM on stopped motor, neutral gear or init phase */
      if ((0 == frq && 0 == idref && 0 == qController.GetRef()) || 0 == dir || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         dController.ResetIntegrator();
         qController.ResetIntegrator();
         fwController.ResetIntegrator();
         idleCounter++;
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
         idleCounter = 0;
      }

      for (int i = 0; i < 3; i++)
      {
         timer_set_oc_value(PWM_TIMER, ocChannels[i], FOC::DutyCycles[i] >> shiftForTimer);
      }
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      initwait = 0;
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
   static int32_t heatCurRamped = 0;
   s32fp brkrampstr = Param::Get(Param::brkrampstr);
   int direction = Param::GetInt(Param::dir);
   int heatCur = Param::GetInt(Param::heatcur);

   heatCur = MIN(400, heatCur);

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

   if (heatCur > 0 && torquePercent < FP_FROMINT(30))
   {
      int speed = Param::GetInt(Param::speed);

      if (speed == 0 && torquePercent <= 0)
      {
         iq = 0;
         heatCurRamped = RAMPUP(heatCurRamped, heatCur, 10);
         id = heatCurRamped;
      }
      /*else if (torquePercent > 0)
      {
         id = FP_TOINT((-heatCur * torquePercent) / 30);
      }*/
      else
      {
         FOC::Mtpa(is, id, iq);
         heatCurRamped = 0;
      }
   }
   else
   {
      FOC::Mtpa(is, id, iq);
      heatCurRamped = 0;
   }

   qController.SetRef(FP_FROMINT(iq));
   fwController.SetRef(FP_FROMINT(iq));
   idref = FP_FROMINT(id);
}

void PwmGeneration::SetControllerGains(int kp, int ki, int fwkp)
{
   qController.SetGains(kp, ki);
   dController.SetGains(kp, ki);
   fwBaseGain = fwkp;
   curki = ki;
}

void PwmGeneration::PwmInit()
{
   int32_t maxVd = FOC::GetMaximumModulationIndex() + Param::GetInt(Param::dmargin);
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = pwmfrq / 2; //0.5s
   idref = 0;
   qController.ResetIntegrator();
   qController.SetCallingFrequency(pwmfrq);
   qController.SetMinMaxY(-maxVd, maxVd);
   dController.ResetIntegrator();
   dController.SetCallingFrequency(pwmfrq);
   dController.SetMinMaxY(-maxVd, maxVd);
   fwController.ResetIntegrator();
   fwController.SetCallingFrequency(pwmfrq);
   fwController.SetMinMaxY(-50 * Param::Get(Param::throtcur), 0); //allow 50% of max current for extra field weakening

   if ((Param::GetInt(Param::pinswap) & SWAP_PWM13) > 0)
   {
      ocChannels[0] = TIM_OC3;
      ocChannels[1] = TIM_OC2;
      ocChannels[2] = TIM_OC1;
   }
   else if ((Param::GetInt(Param::pinswap) & SWAP_PWM23) > 0)
   {
      ocChannels[0] = TIM_OC1;
      ocChannels[1] = TIM_OC3;
      ocChannels[2] = TIM_OC2;
   }
   else
   {
      ocChannels[0] = TIM_OC1;
      ocChannels[1] = TIM_OC2;
      ocChannels[2] = TIM_OC3;
   }

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::ProcessCurrents(s32fp& id, s32fp& iq)
{
   static int il1Avg = 0, il2Avg = 0;
   const int offsetSamples = 16;

   if (initwait > 0)
   {
      initwait--;

      if (initwait <= offsetSamples)
      {
         il1Avg += AnaIn::il1.Get();
         il2Avg += AnaIn::il2.Get();
      }
      else
      {
         il1Avg = il2Avg = 0;
      }

      if (initwait == 1)
      {
         SetCurrentOffset(il1Avg / offsetSamples, il2Avg / offsetSamples);
      }
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

void PwmGeneration::CalcNextAngleSync(int dir)
{
   if (Encoder::SeenNorthSignal())
   {
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();

      //Compensate rotor movement that happened between sampling and processing
      syncOfs += FP_TOINT(dir * frq * 10);

      angle = polePairRatio * rotorAngle + syncOfs;
      frq = polePairRatio * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += dir * FRQ_TO_ANGLE(fslip);
   }
}
