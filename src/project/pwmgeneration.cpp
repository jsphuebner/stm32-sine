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

uint16_t PwmGeneration::pwmfrq;
uint16_t PwmGeneration::angle;
s32fp    PwmGeneration::ampnom;
uint16_t PwmGeneration::slipIncr;
s32fp    PwmGeneration::fslip;
s32fp    PwmGeneration::frq;
uint8_t  PwmGeneration::shiftForTimer;
int      PwmGeneration::opmode;
s32fp    PwmGeneration::ilofs[2];

static int      execTicks;
static bool     tripped;
static uint8_t  pwmdigits;

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

   if (CHK_BIPOLAR_OFS(offset1))
   {
      ErrorMessage::Post(ERR_HICUROFS1);
   }
   if (CHK_BIPOLAR_OFS(offset2))
   {
      ErrorMessage::Post(ERR_HICUROFS2);
   }

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
      tripped = false;
      pwmdigits = MIN_PWM_DIGITS + Param::GetInt(Param::pwmfrq);
      shiftForTimer = SineCore::BITS - pwmdigits;
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
   if (!DigIo::Get(Pin::desat_in) && hwRev != HW_REV1)
      ErrorMessage::Post(ERR_DESAT);
   else if (!DigIo::Get(Pin::emcystop_in) && hwRev != HW_REV3)
      ErrorMessage::Post(ERR_EMCYSTOP);
   else if (!DigIo::Get(Pin::mprot_in))
      ErrorMessage::Post(ERR_MPROT);
   else //if (ocur || hwRev == HW_REV1)
      ErrorMessage::Post(ERR_OVERCURRENT);

   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   Param::SetInt(Param::opmode, MOD_OFF);
   DigIo::Set(Pin::err_out);
   tripped = true;
}

extern "C" void pwm_timer_isr(void)
{
   int start = timer_get_counter(PWM_TIMER);
   /* Clear interrupt pending flag */
   timer_clear_flag(PWM_TIMER, TIM_SR_UIF);

   PwmGeneration::Run();

   int time = timer_get_counter(PWM_TIMER) - start;

   if (TIM_CR1(PWM_TIMER) & TIM_CR1_DIR_DOWN)
      time = (2 << pwmdigits) - timer_get_counter(PWM_TIMER) - start;

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

   timer_set_oc_value(OVER_CUR_TIMER, OVER_CUR_NEG, limNeg);
   timer_set_oc_value(OVER_CUR_TIMER, OVER_CUR_POS, limPos);
}


/*----- Private methods ----------------------------------------- */
void PwmGeneration::CalcNextAngleSync(int dir)
{
   if (Encoder::SeenNorthSignal())
   {
      uint32_t polePairs = Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs);
      uint16_t syncOfs = /*potnom < 0 ? Param::GetInt(Param::syncofsregen) :*/ Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();

      angle = polePairs * rotorAngle + syncOfs;
      frq = polePairs * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += dir * FRQ_TO_ANGLE(fslip);
   }
}

void PwmGeneration::CalcNextAngleAsync(int dir)
{
   static uint16_t slipAngle = 0;
   uint32_t polePairs = Param::GetInt(Param::polepairs);
   uint16_t rotorAngle = Encoder::GetRotorAngle();

   frq = polePairs * Encoder::GetRotorFrequency() + fslip;
   slipAngle += dir * slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairs * rotorAngle + slipAngle;
}

void PwmGeneration::CalcNextAngleConstant(int dir)
{
   frq = fslip;
   angle += dir * slipIncr;

   if (frq < 0) frq = 0;
}

void PwmGeneration::Charge()
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

void PwmGeneration::AcHeat()
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

s32fp PwmGeneration::GetCurrent(AnaIn::AnaIns input, s32fp offset, s32fp gain)
{
   s32fp il = FP_FROMINT(AnaIn::Get(input));
   il -= offset;
   return FP_DIV(il, gain);
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

   if (hwRev == HW_BLUEPILL)
      timer_set_break_polarity_low(PWM_TIMER);
   else
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
