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

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)

static uint8_t  pwmdigits;
static uint16_t pwmfrq;
static volatile uint16_t angle;
static s32fp ampnom;
static uint16_t slipIncr;
static s32fp fslip;
static s32fp frq;
static uint8_t shiftForTimer;
static int opmode;
static bool tripped;
static s32fp ilofs[2];
static uint16_t execTicks = 0;

/*********/
static s32fp ProcessCurrents();
static s32fp LimitCurrent();
static void CalcNextAngleSync(int dir);
static void CalcNextAngleAsync(int dir);
static void CalcNextAngleConstant(int dir);
static void Charge();
static void AcHeat();

uint16_t PwmGeneration::GetAngle()
{
   return angle;
}

bool PwmGeneration::Tripped()
{
   return tripped;
}

void PwmGeneration::SetAmpnom(s32fp amp)
{
   ampnom = amp;
}

void PwmGeneration::SetFslip(s32fp _fslip)
{
   slipIncr = FRQ_TO_ANGLE(_fslip);
   fslip = _fslip;
}

void PwmGeneration::SetCurrentOffset(int offset1, int offset2)
{
   ilofs[0] = FP_FROMINT(offset1);
   ilofs[1] = FP_FROMINT(offset2);
   SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
}

int PwmGeneration::GetCpuLoad()
{
   //PWM period 2x counter because of center aligned mode
   return (1000 * execTicks) / (2 << pwmdigits);
}

void PwmGeneration::SetOpmode(int _opmode)
{
   opmode = _opmode;

   if (opmode != MOD_OFF)
   {
      PwmInit();
   }

   switch (opmode)
   {
      default:
      case MOD_OFF:
         DisableOutput();
         execTicks = 0;
         break;
      case MOD_ACHEAT:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
         break;
      case MOD_BOOST:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
         break;
      case MOD_BUCK:
         DisableOutput();
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
         break;
      case MOD_MANUAL:
      case MOD_RUN:
      case MOD_SINE:
         EnableOutput();
         break;
   }
}

extern "C" void tim1_brk_isr(void)
{
   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   Param::SetInt(Param::opmode, MOD_OFF);
   DigIo::Set(Pin::err_out);
   tripped = true;

   if (!DigIo::Get(Pin::emcystop_in) && hwRev != HW_REV3)
      ErrorMessage::Post(ERR_EMCYSTOP);
   else if (!DigIo::Get(Pin::mprot_in))
      ErrorMessage::Post(ERR_MPROT);
   else if (!DigIo::Get(Pin::desat_in) && hwRev != HW_REV1)
      ErrorMessage::Post(ERR_DESAT);
   else if (!DigIo::Get(Pin::ocur_in) || hwRev == HW_REV1)
      ErrorMessage::Post(ERR_OVERCURRENT);
}

extern "C" void pwm_timer_isr(void)
{
   int start = timer_get_counter(PWM_TIMER);
   /* Clear interrupt pending flag */
   timer_clear_flag(PWM_TIMER, TIM_SR_UIF);

   if (opmode == MOD_MANUAL || opmode == MOD_RUN || opmode == MOD_SINE)
   {
      int dir = Param::GetInt(Param::dir);

      Encoder::UpdateRotorAngle(dir);

      s32fp ampNomLimited = LimitCurrent();

      if (opmode == MOD_SINE)
         CalcNextAngleConstant(dir);
      else if (Encoder::IsSyncMode())
         CalcNextAngleSync(dir);
      else
         CalcNextAngleAsync(dir);

      uint32_t amp = MotorVoltage::GetAmpPerc(frq, ampNomLimited);

      SineCore::SetAmp(amp);
      Param::SetInt(Param::amp, amp);
      Param::SetFlt(Param::fstat, frq);
      Param::SetFlt(Param::angle, DIGIT_TO_DEGREE(angle));
      SineCore::Calc(angle);

      /* Match to PWM resolution */
      SineCore::DutyCycles[0] >>= shiftForTimer;
      SineCore::DutyCycles[1] >>= shiftForTimer;
      SineCore::DutyCycles[2] >>= shiftForTimer;

      /* Shut down PWM on zero voltage request */
      if (0 == amp || 0 == dir)
      {
         timer_disable_break_main_output(PWM_TIMER);
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0]);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1]);
      timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2]);
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      ProcessCurrents();
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      AcHeat();
   }

   int time = timer_get_counter(PWM_TIMER) - start;
   execTicks = ABS(time);
}

/**
* Enable timer PWM output
*/
void PwmGeneration::EnableOutput()
{
   timer_enable_oc_output(PWM_TIMER, TIM_OC1);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3);
   timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_enable_oc_output(PWM_TIMER, TIM_OC3N);
}

/**
* Disable timer PWM output
*/
void PwmGeneration::DisableOutput()
{
   timer_disable_oc_output(PWM_TIMER, TIM_OC1);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3);
   timer_disable_oc_output(PWM_TIMER, TIM_OC1N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC2N);
   timer_disable_oc_output(PWM_TIMER, TIM_OC3N);
}

void PwmGeneration::SetCurrentLimitThreshold(s32fp ocurlim)
{
   //We use the average offset and gain values because we only
   //have one reference channel per polarity
   s32fp iofs = (ilofs[0] + ilofs[1]) / 2;
   s32fp igain= (Param::Get(Param::il1gain) + Param::Get(Param::il2gain)) / 2;

   ocurlim = FP_MUL(igain, ocurlim);
   int limNeg = FP_TOINT(iofs-ocurlim);
   int limPos = FP_TOINT(iofs+ocurlim);
   limNeg = MAX(0, limNeg);
   limPos = MIN(OCURMAX, limPos);

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC2, limNeg);
   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC3, limPos);
}


/*----- Private methods ----------------------------------------- */
void PwmGeneration::PwmInit()
{
   pwmdigits = MIN_PWM_DIGITS + Param::GetInt(Param::pwmfrq);
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   slipIncr = FRQ_TO_ANGLE(fslip);
   shiftForTimer = SineCore::BITS - pwmdigits;
   tripped = false;
   Encoder::SetPwmFrequency(pwmfrq);

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

static s32fp LimitCurrent()
{
   static s32fp curLimSpntFiltered = 0, slipFiltered = 0;
   s32fp slipmin = Param::Get(Param::fslipmin);
   s32fp imax = Param::Get(Param::iacmax);
   s32fp ilMax = ProcessCurrents();

   s32fp a = imax / 20; //Start acting at 80% of imax
   s32fp imargin = imax - ilMax;
   s32fp curLimSpnt = FP_DIV(100 * imargin, a);
   s32fp slipSpnt = FP_DIV(fslip *  imargin, a);
   slipSpnt = MAX(slipmin, slipSpnt);
   curLimSpnt = MAX(FP_FROMINT(40), curLimSpnt); //Never go below 40%
   int filter = Param::GetInt(curLimSpnt < curLimSpntFiltered ? Param::ifltfall : Param::ifltrise);
   curLimSpntFiltered = IIRFILTER(curLimSpntFiltered, curLimSpnt, filter);
   slipFiltered = IIRFILTER(slipFiltered, slipSpnt, 5);

   s32fp ampNomLimited = MIN(ampnom, curLimSpntFiltered);
   slipSpnt = MIN(fslip, slipFiltered);
   slipIncr = FRQ_TO_ANGLE(slipSpnt);

   if (ampNomLimited < ampnom)
      ErrorMessage::Post(ERR_CURRENTLIMIT);

   return ampNomLimited;
}

static s32fp GetIlMax(s32fp il1, s32fp il2)
{
   s32fp il3 = -il1 - il2;
   il1 = ABS(il1);
   il2 = ABS(il2);
   il3 = ABS(il3);
   s32fp ilMax = MAX(il1, il2);
   ilMax = MAX(ilMax, il3);

   Param::SetFlt(Param::ilmax, ilMax);

   return ilMax;
}

static s32fp GetCurrent(AnaIn::AnaIns input, s32fp offset, s32fp gain)
{
   s32fp il = FP_FROMINT(AnaIn::Get(input));
   il -= offset;
   return FP_DIV(il, gain);
}

static bool CalcRms(s32fp il, s32fp illast, s32fp& max, s32fp& rms, int& samples)
{
   const s32fp oneOverSqrt2 = FP_FROMFLT(0.707106781187);
   bool signChanged = ((illast <= 0 && il > 0) || (illast > 0 && il <= 0)) && samples > 10;

   if (signChanged)
   {
      rms = FP_MUL(oneOverSqrt2, max);
      max = 0;
      samples = 0;
   }

   il = ABS(il);
   max = MAX(il, max);
   samples++;

   return signChanged;
}

static s32fp ProcessCurrents()
{
   static s32fp currentMax[2];
   static int samples[2] = { 0 };

   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));
   s32fp rms;

   if (CalcRms(il1, Param::Get(Param::il1), currentMax[0], rms, samples[0]))
   {
      Param::SetFlt(Param::il1rms, rms);

      if (opmode != MOD_BOOST || opmode != MOD_BUCK)
      {
         //rough approximation as we do not take power factor into account
         s32fp idc = (SineCore::GetAmp() * rms) / SineCore::MAXAMP;
         idc = FP_DIV(idc, FP_FROMFLT(1.2247)); //divide by sqrt(3)/sqrt(2)
         idc *= fslip < 0 ? -1 : 1;
         Param::SetFlt(Param::idc, idc);
      }
   }
   if (CalcRms(il2, Param::Get(Param::il2), currentMax[1], rms, samples[1]))
   {
      Param::SetFlt(Param::il2rms, rms);
   }

   Param::SetFlt(Param::il1, il1);
   Param::SetFlt(Param::il2, il2);

   return GetIlMax(il1, il2);
}

static void CalcNextAngleSync(int dir)
{
   if (Encoder::SeenNorthSignal())
   {
      uint32_t polePairs = Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs);
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();

      if (dir < 0)
      {
         syncOfs += SHIFT_180DEG;
      }

      angle = polePairs * rotorAngle + syncOfs;
      frq = polePairs * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += dir * FRQ_TO_ANGLE(fslip);
   }
}

static void CalcNextAngleAsync(int dir)
{
   static uint16_t slipAngle = 0;
   uint32_t polePairs = Param::GetInt(Param::polepairs);
   uint16_t rotorAngle = Encoder::GetRotorAngle();

   frq = polePairs * Encoder::GetRotorFrequency() + fslip;
   slipAngle += dir * slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairs * rotorAngle + slipAngle;
}

static void CalcNextAngleConstant(int dir)
{
   frq = fslip;
   angle += dir * slipIncr;

   if (frq < 0) frq = 0;
}

static void Charge()
{
   int dc = ampnom * (1 << pwmdigits);
   dc = FP_TOINT(dc) / 100;

   if (dc > ((1 << pwmdigits) - 100))
      dc = (1 << pwmdigits) - 100;
   if (dc < 0)
      dc = 0;

   Param::SetInt(Param::amp, dc);

   timer_set_oc_value(PWM_TIMER, TIM_OC2, dc);
}

static void AcHeat()
{
   //We need to make sure the negative output is NEVER permanently on.
   if (ampnom < FP_FROMFLT(20))
   {
      timer_disable_break_main_output(PWM_TIMER);
   }
   else
   {
      timer_enable_break_main_output(PWM_TIMER);
      int dc = FP_TOINT((ampnom * 30000) / 100);
      Param::SetInt(Param::amp, dc);
      timer_set_period(PWM_TIMER, dc);
      timer_set_oc_value(PWM_TIMER, TIM_OC2, dc / 2);
   }
}

/**
* Setup main PWM timer
*
* @param[in] deadtime Deadtime between bottom and top (coded value, consult STM32 manual)
* @param[in] pwmpol Output Polarity. 0=Active High, 1=Active Low
* @return PWM frequency
*/
uint16_t PwmGeneration::TimerSetup(uint16_t deadtime, int pwmpol)
{
   const uint16_t pwmmax = 1U << pwmdigits;
   uint8_t outputMode;

   rcc_periph_reset_pulse(PWM_TIMRST);
   /* disable timer */
   timer_disable_counter(PWM_TIMER);
   /* Center aligned PWM */
   timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
   timer_enable_preload(PWM_TIMER);
   /* PWM mode 1 and preload enable */
   TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE |
                          TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
   TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;

   if (pwmpol)
   {
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_low(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN;
   }
   else
   {
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC1N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2N);
      timer_set_oc_polarity_high(PWM_TIMER, TIM_OC3N);
      outputMode = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;
   }

   timer_disable_break_automatic_output(PWM_TIMER);
   timer_enable_break_main_output(PWM_TIMER);
   timer_set_break_polarity_high(PWM_TIMER);
   timer_enable_break(PWM_TIMER);
   timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
   timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);

   timer_set_deadtime(PWM_TIMER, deadtime);

   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC1N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC2N);
   timer_set_oc_idle_state_unset(PWM_TIMER, TIM_OC3N);

   timer_clear_flag(PWM_TIMER, TIM_SR_BIF);
   timer_enable_irq(PWM_TIMER, TIM_DIER_UIE);
   timer_enable_irq(PWM_TIMER, TIM_DIER_BIE);

   timer_set_prescaler(PWM_TIMER, 0);
   /* PWM frequency */
   timer_set_period(PWM_TIMER, pwmmax);
   timer_set_repetition_counter(PWM_TIMER, 1);

   timer_set_oc_value(PWM_TIMER, TIM_OC1, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
   timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_enable_counter(PWM_TIMER);

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO13 | GPIO14 | GPIO15);

   return PERIPH_CLK / (uint32_t)pwmmax;
}

void PwmGeneration::AcHeatTimerSetup()
{
   timer_disable_counter(PWM_TIMER);
   timer_set_clock_division(PWM_TIMER, TIM_CR1_CKD_CK_INT_MUL_4);
   timer_set_deadtime(PWM_TIMER, 255);
   timer_set_period(PWM_TIMER, 8000);
   timer_set_oc_value(PWM_TIMER, TIM_OC2, 0);
   timer_generate_event(PWM_TIMER, TIM_EGR_UG);
   timer_enable_counter(PWM_TIMER);
}
