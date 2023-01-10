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
#include "picontroller.h"

#define SHIFT_180DEG (uint16_t)32768
#define SHIFT_90DEG  (uint16_t)16384
#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(angle) / (65536 / 360)
#define FRQ_DIVIDER 8192 //PWM ISR callback frequency divider

uint16_t PwmGeneration::pwmfrq = 1;
uint16_t PwmGeneration::angle;
s32fp    PwmGeneration::ampnom;
s32fp    PwmGeneration::fslip;
s32fp    PwmGeneration::frq;
s32fp    PwmGeneration::frqFiltered;
uint8_t  PwmGeneration::shiftForTimer;
int      PwmGeneration::opmode;
s32fp    PwmGeneration::ilofs[2];
int      PwmGeneration::polePairRatio;
int16_t  PwmGeneration::slipIncr;

static int      execTicks;
static bool     tripped;
static uint8_t  pwmdigits;
static PiController chargeController;

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
   return (1000 * execTicks) / FRQ_DIVIDER;
}

static void ConfigureChargeController()
{
   int pwmin = FP_TOINT((Param::Get(Param::chargepwmin) * (1 << pwmdigits)) / 100);
   int pwmax = FP_TOINT((Param::Get(Param::chargepwmax) * (1 << pwmdigits)) / 100);

   chargeController.SetCallingFrequency(rcc_apb2_frequency / FRQ_DIVIDER);
   chargeController.SetMinMaxY(pwmin, pwmax);
   chargeController.SetGains(Param::GetInt(Param::chargekp), Param::GetInt(Param::chargeki));
   chargeController.SetRef(0);
   chargeController.PreloadIntegrator(pwmin);
}

void PwmGeneration::SetOpmode(int _opmode)
{
   if (opmode == _opmode) return;
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
         timer_enable_break_main_output(PWM_TIMER);
         break;
      case MOD_BOOST:
         DisableOutput();
         EnableChargeOutput();
         timer_enable_break_main_output(PWM_TIMER);
         ConfigureChargeController();
         break;
      case MOD_BUCK:
         DisableOutput();
         EnableChargeOutput();
         timer_enable_break_main_output(PWM_TIMER);
         ConfigureChargeController();
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
   if (!DigIo::desat_in.Get() && hwRev != HW_REV1 && hwRev != HW_BLUEPILL)
      ErrorMessage::Post(ERR_DESAT);
   else if (!DigIo::emcystop_in.Get() && hwRev != HW_REV3)
      ErrorMessage::Post(ERR_EMCYSTOP);
   else if (!DigIo::mprot_in.Get() && hwRev != HW_BLUEPILL)
      ErrorMessage::Post(ERR_MPROT);
   else //if (ocur || hwRev == HW_REV1)
      ErrorMessage::Post(ERR_OVERCURRENT);

   timer_disable_irq(PWM_TIMER, TIM_DIER_BIE);
   Param::SetInt(Param::opmode, MOD_OFF);
   DigIo::err_out.Set();
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

   if (hwRev != HW_PRIUS)
   {
      timer_enable_oc_output(PWM_TIMER, TIM_OC1N);
      timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
      timer_enable_oc_output(PWM_TIMER, TIM_OC3N);
   }
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

void PwmGeneration::EnableChargeOutput()
{
   /* Prius GEN2 inverter only has one control signal for the buck/boost converter
    * When we output a "high" the upper IGBT is switched on. So for bucking
    * output current and duty cycle are proportional
    * For boosting we need to output a "low" to enable the low side IGBT and
    * thus the output current and duty cycle are inverse proportional.
    */
   if (hwRev == HW_PRIUS)
   {
      //Disable other PWM source.
      timer_disable_oc_output(OVER_CUR_TIMER, TIM_OC4);

      if (opmode == MOD_BOOST)
         timer_set_oc_polarity_low(PWM_TIMER, TIM_OC2N);
      else if (opmode == MOD_BUCK)
         timer_set_oc_polarity_high(PWM_TIMER, TIM_OC2N);

      timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
   }
   else
   {
      if (opmode == MOD_BOOST)
         timer_enable_oc_output(PWM_TIMER, TIM_OC2N);
      else if (opmode == MOD_BUCK)
         timer_enable_oc_output(PWM_TIMER, TIM_OC2);
   }
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

void PwmGeneration::SetChargeCurrent(float cur)
{
   chargeController.SetRef(FP_FROMFLT(cur));
}


/*----- Private methods ----------------------------------------- */

void PwmGeneration::CalcNextAngleAsync(int dir)
{
   static uint16_t slipAngle = 0;
   uint16_t rotorAngle = Encoder::GetRotorAngle();

   frq = polePairRatio * Encoder::GetRotorFrequency() + fslip;
   slipAngle += dir * slipIncr;

   if (frq < 0) frq = 0;

   angle = polePairRatio * rotorAngle + slipAngle;
}

void PwmGeneration::CalcNextAngleConstant(int dir)
{
   frq = fslip;
   angle += dir * slipIncr;

   if (frq < 0) frq = 0;
}

void PwmGeneration::Charge()
{
   static s32fp iFlt;
   int pwmin = FP_TOINT((Param::Get(Param::chargepwmin) * (1 << pwmdigits)) / 100);
   int pwmax = FP_TOINT((Param::Get(Param::chargepwmax) * (1 << pwmdigits)) / 100);

   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

   il1 = ABS(il1);
   il2 = ABS(il2);

   s32fp ilSum = il1 + il2;

   iFlt = IIRFILTER(iFlt, ilSum, Param::GetInt(Param::chargeflt));

   chargeController.SetMinMaxY(pwmin, pwmax);
   chargeController.SetGains(Param::GetInt(Param::chargekp), Param::GetInt(Param::chargeki));

   int dc = chargeController.Run(iFlt);

   if (opmode == MOD_BOOST)
      Param::SetFixed(Param::idc, (((1 << pwmdigits) - dc) * iFlt) / (1 << pwmdigits));
   else
      Param::SetFixed(Param::idc, iFlt);

   Param::SetInt(Param::amp, dc);
   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);

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

s32fp PwmGeneration::GetCurrent(AnaIn& input, s32fp offset, s32fp gain)
{
   s32fp il = FP_FROMINT(input.Get());
   il -= offset;
   return FP_DIV(il, gain);
}

/**
* Setup main PWM timer
*
* @param[in] deadtime Deadtime between bottom and top (coded value, consult STM32 manual)
* @param[in] activeLow Set Output Polarity true=Active Low
* @return PWM ISR callback frequency
*/
uint16_t PwmGeneration::TimerSetup(uint16_t deadtime, bool activeLow)
{
   ///There are two update events per PWM period
   ///One when counter reaches top, one when it reaches bottom
   ///We set the repetition counter in a way, that the ISR
   ///Callback frequency is constant i.e. independent from PWM frequency
   ///- for 17.6 kHz: call ISR every four update events (that is every other period)
   ///- for 8.8kHz: call ISR every other update event (that is once per PWM period)
   ///- for 4.4kHz: call ISR on every update event (that is twice per period)
   const uint8_t repCounters[] = { 3, 1, 0 };
   const uint16_t pwmmax = 1U << pwmdigits;
   const uint8_t outputMode = activeLow ? GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN : GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;

   //Disable output in active low mode before resetting timer, otherwise shoot through will occur!
   if (activeLow)
   {
      gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8 | GPIO9 | GPIO10);
      gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO13 | GPIO14 | GPIO15);
   }

   rcc_periph_reset_pulse(PWM_TIMRST);

   /* Center aligned PWM */
   timer_set_alignment(PWM_TIMER, TIM_CR1_CMS_CENTER_1);
   timer_enable_preload(PWM_TIMER);

   for (int channel = TIM_OC1; channel <= TIM_OC3N; channel++)
   {
      timer_enable_oc_preload(PWM_TIMER, (tim_oc_id)channel);
      timer_set_oc_mode(PWM_TIMER, (tim_oc_id)channel, TIM_OCM_PWM1);
      timer_set_oc_idle_state_unset(PWM_TIMER, (tim_oc_id)channel);
      timer_set_oc_value(PWM_TIMER, (tim_oc_id)channel, 0);

      if (activeLow)
         timer_set_oc_polarity_low(PWM_TIMER, (tim_oc_id)channel);
      else
         timer_set_oc_polarity_high(PWM_TIMER, (tim_oc_id)channel);
   }

   timer_disable_break_automatic_output(PWM_TIMER);

   if (hwRev == HW_BLUEPILL || hwRev == HW_PRIUS)
      timer_set_break_polarity_low(PWM_TIMER);
   else
      timer_set_break_polarity_high(PWM_TIMER);

   timer_enable_break(PWM_TIMER);
   timer_set_enabled_off_state_in_run_mode(PWM_TIMER);
   timer_set_enabled_off_state_in_idle_mode(PWM_TIMER);
   timer_set_deadtime(PWM_TIMER, deadtime);
   timer_clear_flag(PWM_TIMER, TIM_SR_UIF | TIM_SR_BIF);
   timer_enable_irq(PWM_TIMER, TIM_DIER_UIE | TIM_DIER_BIE);

   timer_set_prescaler(PWM_TIMER, 0);
   /* PWM frequency */
   timer_set_period(PWM_TIMER, pwmmax);
   timer_set_repetition_counter(PWM_TIMER, repCounters[pwmdigits - MIN_PWM_DIGITS]);

   timer_generate_event(PWM_TIMER, TIM_EGR_UG);

   timer_enable_counter(PWM_TIMER);

   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, outputMode, GPIO13 | GPIO14 | GPIO15);
   //Callback frequency is constant because we use the repetition counter
   return rcc_apb2_frequency / FRQ_DIVIDER;
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
