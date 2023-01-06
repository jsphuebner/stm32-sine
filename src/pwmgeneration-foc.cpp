/*
 * This file is part of the stm32-sine project.
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

#ifndef QLIMIT_FREQUENCY
#define QLIMIT_FREQUENCY FP_FROMINT(30)
#endif // QLIMIT_FREQUENCY

static int initwait = 0;
static bool isIdle = false;
static const s32fp dcCurFac = FP_FROMFLT(0.81649658092772603273 * 1.05); //sqrt(2/3)*1.05 (inverter losses)
static const int32_t fwOutMax = 1024;
static const uint32_t shiftForFilter = 8;
static s32fp idMtpa = 0, iqMtpa = 0;
static tim_oc_id ocChannels[3];
static PiController qController;
static PiController dController;

void PwmGeneration::Run()
{
   if (opmode == MOD_RUN)
   {
      static s32fp idcFiltered = 0;
      static int amplitudeErrFiltered;
      int dir = Param::GetInt(Param::dir);
      s32fp id, iq;

      Encoder::UpdateRotorAngle(0);
      CalcNextAngleSync();
      FOC::SetAngle(angle);
      ProcessCurrents(id, iq);

      frqFiltered = IIRFILTER(frqFiltered, frq, 4);

      if (initwait == 0)
      {
         int amplitudeErr = (FOC::GetMaximumModulationIndex() - Param::GetInt(Param::vlimmargin)) - Param::GetInt(Param::amp);
         amplitudeErr = MIN(fwOutMax, amplitudeErr);
         amplitudeErr = MAX(fwOutMax / 20, amplitudeErr);
         amplitudeErrFiltered = IIRFILTER(amplitudeErrFiltered, amplitudeErr << shiftForFilter, Param::GetInt(Param::vlimflt));

         if (0 == frq) amplitudeErrFiltered = fwOutMax << shiftForFilter;

         int vlim = amplitudeErrFiltered >> shiftForFilter;
         s32fp ifw = ((fwOutMax - vlim) * Param::Get(Param::fwcurmax)) / fwOutMax;
         Param::SetFixed(Param::ifw, ifw);

         s32fp limitedIq = (vlim * iqMtpa) / fwOutMax;
         qController.SetRef(limitedIq + Param::Get(Param::manualiq));

         s32fp limitedId = -2 * ABS(limitedIq); //ratio between idMtpa and iqMtpa never > 2
         limitedId = MAX(idMtpa, limitedId);
         limitedId = MIN(ifw, limitedId);
         dController.SetRef(limitedId + Param::Get(Param::manualid));
      }

      int32_t ud = dController.Run(id);
      int32_t qlimit = FOC::GetQLimit(ud);

      if (frqFiltered < QLIMIT_FREQUENCY)
         qController.SetMinMaxY(dir <= 0 ? -qlimit : 0, dir >= 0 ? qlimit : 0);
      else
         qController.SetMinMaxY(-qlimit, qlimit);

      int32_t uq = qController.Run(iq);
      uint16_t advancedAngle = angle + dir * FP_TOINT(FP_MUL(Param::Get(Param::syncadv), frqFiltered));
      FOC::SetAngle(advancedAngle);
      FOC::InvParkClarke(ud, uq);

      s32fp idc = (iq * uq + id * ud) / FOC::GetMaximumModulationIndex();
      idc = FP_MUL(idc, dcCurFac);
      idcFiltered = IIRFILTER(idcFiltered, idc, Param::GetInt(Param::idcflt));

      uint32_t amp = FOC::GetTotalVoltage(ud, uq);

      Param::SetFixed(Param::fstat, frq);
      Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetFixed(Param::idc, idcFiltered);
      Param::SetInt(Param::uq, uq);
      Param::SetInt(Param::ud, ud);
      Param::SetInt(Param::amp, amp);

      /* Shut down PWM on stopped motor or init phase */
      if (isIdle || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         dController.ResetIntegrator();
         qController.ResetIntegrator();
         RunOffsetCalibration();
         amplitudeErrFiltered = fwOutMax << shiftForFilter;
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
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

void PwmGeneration::SetTorquePercent(float torquePercent)
{
   float is = Param::GetFloat(Param::throtcur) * torquePercent;
   float id, iq;

   FOC::Mtpa(is, id, iq);

   //This is used to disable PWM and do offset calibration at standstill
   isIdle = 0 == frq &&
            0 == torquePercent &&
            0 == Param::Get(Param::manualid) &&
            0 == Param::Get(Param::manualiq) &&
            hwRev != HW_PRIUS; //Don't do it for Prius Gen2 inverters

   iqMtpa = FP_FROMFLT(iq);
   idMtpa = FP_FROMFLT(id);
}

void PwmGeneration::SetControllerGains(int kp, int ki)
{
   qController.SetGains(kp, ki);
   dController.SetGains(kp, ki);
}

void PwmGeneration::PwmInit()
{
   int32_t maxVd = FOC::GetMaximumModulationIndex() - 1000;
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = pwmfrq / 2; //0.5s
   qController.ResetIntegrator();
   qController.SetCallingFrequency(pwmfrq);
   qController.SetMinMaxY(-maxVd, maxVd);
   dController.ResetIntegrator();
   dController.SetCallingFrequency(pwmfrq);
   dController.SetMinMaxY(-maxVd, maxVd);

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
   if (initwait > 0)
   {
      initwait--;
   }

   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

   if ((Param::GetInt(Param::pinswap) & SWAP_CURRENTS) > 0)
      FOC::ParkClarke(il2, il1);
   else
      FOC::ParkClarke(il1, il2);
   id = FOC::id;
   iq = FOC::iq;

   Param::SetFixed(Param::id, FOC::id);
   Param::SetFixed(Param::iq, FOC::iq);
   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);

   return 0;
}

void PwmGeneration::CalcNextAngleSync()
{
   if (Encoder::SeenNorthSignal())
   {
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();

      angle = polePairRatio * rotorAngle + syncOfs;
      frq = polePairRatio * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += FRQ_TO_ANGLE(fslip);
   }
}

void PwmGeneration::RunOffsetCalibration()
{
   static int il1Avg = 0, il2Avg = 0, samples = 0;
   const int offsetSamples = 512;

   if (samples < offsetSamples)
   {
      il1Avg += AnaIn::il1.Get();
      il2Avg += AnaIn::il2.Get();
      samples++;
   }
   else
   {
      SetCurrentOffset(il1Avg / offsetSamples, il2Avg / offsetSamples);
      il1Avg = il2Avg = 0;
      samples = 0;
   }
}
