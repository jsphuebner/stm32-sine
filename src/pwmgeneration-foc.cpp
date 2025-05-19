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
#define DEGREE_TO_DIGIT(a) (((a) * 65536) / 360)

#ifndef QLIMIT_FREQUENCY
#define QLIMIT_FREQUENCY FP_FROMINT(30)
#endif // QLIMIT_FREQUENCY

static s32fp MeasureCoggingCurrent(uint16_t angle, s32fp id);
static int32_t GenerateAntiCoggingSignal(uint16_t angle, s32fp coggingCurrent);
static int initwait = 0;
static bool isIdle = false;
static const s32fp dcCurFac = FP_FROMFLT(0.81649658092772603273 * 1.05); //sqrt(2/3)*1.05 (inverter losses)
static const int32_t fwOutMax = 1024;
static const uint32_t shiftForFilter = 8;
static s32fp idMtpa = 0, iqMtpa = 0;
static PiController qController;
static PiController dController;
static PiController excController;
static s32fp fwCurMax = 0;
static s32fp excCurMax = 0;

void PwmGeneration::Run()
{
   if (opmode == MOD_RUN)
   {
      static s32fp idcFiltered = 0;
      static int amplitudeErrFiltered;
      int dir = Param::GetInt(Param::seldir);
      s32fp id, iq;

      Encoder::UpdateRotorAngle(0);
      CalcNextAngleSync();
      angle += dir * FP_TOINT(FP_MUL(Param::Get(Param::syncadv), frqFiltered));
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

         if (hwRev == HW_ZOE)
         {
            s32fp exciterSpnt = (excCurMax * vlim) / fwOutMax;
            s32fp iexc = FP_DIV((AnaIn::udc.Get() - Param::GetInt(Param::udcofs)), Param::GetInt(Param::udcgain));
            Param::SetFixed(Param::ifw, iexc);
            dController.SetRef(idMtpa + Param::Get(Param::manualid));
            excController.SetRef(exciterSpnt);
            qController.SetRef(iqMtpa + Param::Get(Param::manualiq));
            uint16_t pwm = excController.Run(iexc);
            Param::SetInt(Param::uexc, pwm);
            timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, pwm);
         }
         else
         {
            s32fp ifw = ((fwOutMax - vlim) * fwCurMax) / fwOutMax;
            Param::SetFixed(Param::ifw, ifw);

            s32fp limitedIq = (vlim * iqMtpa) / fwOutMax;
            qController.SetRef(limitedIq + Param::Get(Param::manualiq));

            s32fp limitedId = -2 * ABS(limitedIq); //ratio between idMtpa and iqMtpa never > 2
            limitedId = MAX(idMtpa, limitedId);
            limitedId = MIN(ifw, limitedId);
            dController.SetRef(limitedId + Param::Get(Param::manualid));
         }
      }

      s32fp coggingCurrent = MeasureCoggingCurrent(angle, id);
      int32_t antiCogScaled = GenerateAntiCoggingSignal(angle, coggingCurrent);
      Param::SetInt(Param::anticog, antiCogScaled);
      int32_t ud = dController.Run(id, antiCogScaled);
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
         //timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, 0); //stop rotor excitation
         dController.ResetIntegrator();
         qController.ResetIntegrator();
         //excController.ResetIntegrator();
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

void PwmGeneration::SetFwExcCurMax(float fwcur, float excur)
{
   fwCurMax = FP_FROMFLT(fwcur);
   excCurMax = FP_FROMFLT(excur);
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

void PwmGeneration::SetControllerGains(int iqkp, int idkp, int exckp, int ki)
{
   qController.SetGains(iqkp, ki);
   dController.SetGains(idkp, ki);
   excController.SetGains(exckp, 1000);
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
   excController.SetCallingFrequency(pwmfrq);
   excController.SetMinMaxY(0, 2048);

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
   if (true)
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

static s32fp MeasureCoggingCurrent(uint16_t angle, s32fp id)
{
   static uint16_t previousAngle = 0;
   static s32fp minId = 0x7fffffff, maxId = -0x7fffffff;
   static s32fp coggingCurrent = 0;

   if (previousAngle < 32767 && angle > 32767)
   {
      coggingCurrent = ABS(minId - maxId);
      minId = 0x7fffffff;
      maxId = -minId;
   }
   else
   {
      minId = MIN(id, minId);
      maxId = MAX(id, maxId);
   }
   previousAngle = angle;
   return coggingCurrent;
}

/** \brief Generates a trapezoidal wave form to counter the cogging current of IPM motors
 *
 * \param angle rotor angle
 * \param coggingCurrent magnitude of cogging current in A
 * \return int32_t counter waveform as integer
 *
 */
static int32_t GenerateAntiCoggingSignal(uint16_t angle, s32fp coggingCurrent)
{
   angle += Param::GetInt(Param::cogph);

   if (angle < DEGREE_TO_DIGIT(90))
      angle = angle; //no change
   else if (angle < DEGREE_TO_DIGIT(180)) //90 to 180°
      angle = 32767 - angle;
   else if (angle < DEGREE_TO_DIGIT(270)) //180 to 270°
      angle = angle - 32767;
   else //270 to 360°
      angle = 65535 - angle;

   uint16_t antiCog = 4 * angle;
   int32_t antiCogScaled = ((int32_t)antiCog) - 32767;
   int32_t antiCogMax = Param::GetInt(Param::cogmax);
   antiCogScaled = (antiCogScaled * FP_TOINT(coggingCurrent) * Param::GetInt(Param::cogkp)) / 65536;
   antiCogScaled = MIN(antiCogScaled, antiCogMax);
   antiCogScaled = MAX(antiCogScaled, -antiCogMax);

   return antiCogScaled;
}
