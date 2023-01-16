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

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)
#define INV_SQRT_1_5 FP_FROMFLT(0.8164965809)

void PwmGeneration::Run()
{
   if (opmode == MOD_MANUAL || opmode == MOD_RUN || opmode == MOD_SINE)
   {
      int dir = Param::GetInt(Param::dir);

      Encoder::UpdateRotorAngle(dir);
      s32fp ampNomLimited = LimitCurrent();

      if (opmode == MOD_SINE)
         CalcNextAngleConstant(dir);
      else
         CalcNextAngleAsync(dir);

      uint32_t amp = MotorVoltage::GetAmpPerc(frq, ampNomLimited);

      SineCore::SetAmp(amp);
      Param::SetInt(Param::amp, amp);
      Param::SetFixed(Param::fstat, frq);
      Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));
      SineCore::Calc(angle);

      /* Shut down PWM on zero voltage request */
      if (0 == amp || 0 == dir)
      {
         timer_disable_break_main_output(PWM_TIMER);
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      /* Match to PWM resolution */
      timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1] >> shiftForTimer);
      timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2] >> shiftForTimer);
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      AcHeat();
   }
}

void PwmGeneration::SetTorquePercent(float torque)
{
   int filterConst = Param::GetInt(Param::throtfilter);
   float roundingError = FP_TOFLOAT((float)((1 << filterConst) - 1));
   float fslipmin = Param::GetFloat(Param::fslipmin);
   float ampmin = Param::GetFloat(Param::ampmin);
   float slipstart = Param::GetFloat(Param::slipstart);
   float ampnomLocal;
   float fslipspnt = 0;

   if (torque >= 0)
   {
      /* In async mode first X% throttle commands amplitude, X-100% raises slip */
      ampnomLocal = ampmin + (100.0f - ampmin) * torque / slipstart;

      if (torque >= slipstart)
      {
         float fstat = Param::GetFloat(Param::fstat);
         float fweak = Param::GetFloat(Param::fweakcalc);
         float fslipmax = Param::GetFloat(Param::fslipmax);

         if (fstat > fweak)
         {
            float fconst = Param::GetFloat(Param::fconst);
            float fslipconstmax = Param::GetFloat(Param::fslipconstmax);
            //Basically, for every Hz above fweak we get a fraction of
            //the difference between fslipconstmax and fslipmax
            //of additional slip
            fslipmax += (fstat - fweak) / (fconst - fweak) * (fslipconstmax - fslipmax);
            fslipmax = MIN(fslipmax, fslipconstmax); //never exceed fslipconstmax!
         }

         float fslipdiff = fslipmax - fslipmin;
         fslipspnt = roundingError + fslipmin + (fslipdiff * (torque - slipstart)) / (100.0f - slipstart);
      }
      else
      {
         fslipspnt = fslipmin + roundingError;
      }
   }
   else if (Encoder::GetRotorDirection() != Param::GetInt(Param::dir))
   {
      // Do not apply negative torque if we are already traveling backwards.
      fslipspnt = 0;
      ampnomLocal = 0;
   }
   else
   {
      ampnomLocal = -torque;
      fslipspnt = -fslipmin;
   }

   ampnomLocal = MIN(ampnomLocal, 100.0f);
   //anticipate sudden changes by filtering
   ampnom = IIRFILTER(ampnom, FP_FROMFLT(ampnomLocal), filterConst);
   fslip = IIRFILTER(fslip, FP_FROMFLT(fslipspnt), filterConst);
   Param::Set(Param::ampnom, ampnom);
   Param::Set(Param::fslipspnt, fslip);

   slipIncr = FRQ_TO_ANGLE(fslip);
}

void PwmGeneration::PwmInit()
{
   PwmGeneration::SetCurrentOffset(AnaIn::il1.Get(), AnaIn::il2.Get());
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   Encoder::SetPwmFrequency(pwmfrq);

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::LimitCurrent()
{
   static s32fp curLimSpntFiltered = 0, slipFiltered = 0;
   s32fp slipmin = Param::Get(Param::fslipmin);
   s32fp imax = Param::Get(Param::iacmax);
   s32fp ilMax = ProcessCurrents();

   //setting of 0 disables current limiting
   if (imax == 0) return ampnom;

   s32fp a = imax / 20; //Start acting at 80% of imax
   s32fp imargin = imax - ilMax;
   s32fp curLimSpnt = FP_DIV(100 * imargin, a);
   s32fp slipSpnt = FP_DIV(FP_MUL(fslip, imargin), a);
   slipSpnt = MAX(slipmin, slipSpnt);
   curLimSpnt = MAX(FP_FROMINT(40), curLimSpnt); //Never go below 40%
   int filter = Param::GetInt(curLimSpnt < curLimSpntFiltered ? Param::ifltfall : Param::ifltrise);
   curLimSpntFiltered = IIRFILTER(curLimSpntFiltered, curLimSpnt, filter);
   slipFiltered = IIRFILTER(slipFiltered, slipSpnt, 1);

   s32fp ampNomLimited = MIN(ampnom, curLimSpntFiltered);
   slipSpnt = MIN(fslip, slipFiltered);
   slipIncr = FRQ_TO_ANGLE(slipSpnt);

   if (curLimSpnt < ampnom)
      ErrorMessage::Post(ERR_CURRENTLIMIT);

   return ampNomLimited;
}

s32fp PwmGeneration::GetIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   s32fp ilMax = FP_MUL(il1, il1) + FP_MUL(il2, il2) + FP_MUL(il3, il3);
   ilMax = fp_sqrt(ilMax);
   ilMax = FP_MUL(ilMax, INV_SQRT_1_5);

   return ilMax;
}

PwmGeneration::EdgeType PwmGeneration::CalcRms(s32fp il, EdgeType& lastEdge, s32fp& max, s32fp& rms, int& samples, s32fp prevRms)
{
   const s32fp oneOverSqrt2 = FP_FROMFLT(0.707106781187);
   int minSamples = pwmfrq / (4 * FP_TOINT(frq));
   EdgeType edgeType = NoEdge;

   minSamples = MAX(10, minSamples);

   if (samples > minSamples)
   {
      if (lastEdge == NegEdge && il > 0)
         edgeType = PosEdge;
      else if (lastEdge == PosEdge && il < 0)
         edgeType = NegEdge;
   }

   if (edgeType != NoEdge)
   {
      rms = (FP_MUL(oneOverSqrt2, max) + prevRms) / 2; // average with previous rms reading

      max = 0;
      samples = 0;
      lastEdge = edgeType;
   }

   il = ABS(il);
   max = MAX(il, max);
   samples++;

   return edgeType;
}

s32fp PwmGeneration::ProcessCurrents()
{
   static s32fp currentMax[2];
   static int samples[2] = { 0 };
   static EdgeType lastEdge[2] = { PosEdge, PosEdge };

   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));
   s32fp rms;
   s32fp il1PrevRms = Param::Get(Param::il1rms);
   s32fp il2PrevRms = Param::Get(Param::il2rms);
   EdgeType edge = CalcRms(il1, lastEdge[0], currentMax[0], rms, samples[0], il1PrevRms);

   if (edge != NoEdge)
   {
      Param::SetFixed(Param::il1rms, rms);

      if (opmode != MOD_BOOST || opmode != MOD_BUCK)
      {
         //rough approximation as we do not take power factor into account
         s32fp idc = (SineCore::GetAmp() * rms) / SineCore::MAXAMP;
         idc = FP_MUL(idc, FP_FROMFLT(1.2247)); //multiply by sqrt(3)/sqrt(2)
         idc *= fslip < 0 ? -1 : 1;
         Param::SetFixed(Param::idc, idc);
      }
   }
   if (CalcRms(il2, lastEdge[1], currentMax[1], rms, samples[1], il2PrevRms))
   {
      Param::SetFixed(Param::il2rms, rms);
   }

   s32fp ilMax = GetIlMax(il1, il2);

   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);
   Param::SetFixed(Param::ilmax, ilMax);

   return ilMax;
}
