/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#include "hwdefs.h"
#include "inc_encoder.h"
#include "my_math.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include "errormessage.h"
#include "params.h"
#include "sine_core.h"

#define TWO_PI 65536
#define MAX_CNT 65535
#define MAX_REVCNT_VALUES 5

static void InitTimerSingleChannelMode();
static void InitTimerABZMode();
static void InitSPIMode();
static void InitResolverMode();
static void DMASetup();
static uint16_t GetAngleSPI();
static uint16_t GetAngleResolver();
static int GetPulseTimeFiltered();
static void GetMinMaxTime(uint32_t& min, uint32_t& max);

static volatile uint16_t timdata[MAX_REVCNT_VALUES];
static volatile uint16_t angle = 0;
static uint32_t lastPulseTimespan = 0;
static uint16_t filter = 0;
static uint32_t anglePerPulse = 0;
static uint16_t pulsesPerTurn = 0;
static uint32_t turnsSinceLastSample = 0;
static u32fp lastFrequency = 0;
static bool ignore = true;
static enum Encoder::mode encMode = Encoder::INVALID;
static bool seenNorthSignal = false;

void Encoder::Init(void)
{
   //encMode = INVALID; //Make sure SetMode does something
   //SetMode(false, false);
   Reset();
}

void Encoder::Reset()
{
   ignore = true;
   lastPulseTimespan = MAX_CNT;
   for (uint32_t i = 0; i < MAX_REVCNT_VALUES; i++)
      timdata[i] = MAX_CNT;
}

/** Since power up, have we seen the north marker? */
bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}

/** Use pulse timing (single channel mode) or ABZ encoder mode
 * @param useAbzMode use ABZ mode, single channel otherwise
 * @param useSyncMode reset counter on rising edge of ETR */
void Encoder::SetMode(enum Encoder::mode mode)
{
   if (encMode == mode) return;

   encMode = mode;
   if (mode == AB || mode == ABZ)
      InitTimerABZMode();
   else if (mode == SINGLE)
      InitTimerSingleChannelMode();
   else if (mode == SPI)
      InitSPIMode();
   else if (mode == RESOLVER)
      InitResolverMode();
}

/** set number of impulses per shaft rotation
  */
void Encoder::SetImpulsesPerTurn(uint16_t imp)
{
   if (imp == pulsesPerTurn) return;

   pulsesPerTurn = imp;
   anglePerPulse = TWO_PI / imp;

   if (encMode == AB || encMode == ABZ)
      InitTimerABZMode();
}

/** set filter constant of filter after pulse counter */
void Encoder::SetFilterConst(uint8_t flt)
{
   filter = flt;
}

void Encoder::UpdateRotorAngle(int dir)
{
   if (encMode == SINGLE)
   {
      int16_t numPulses = GetPulseTimeFiltered();

      angle += (int16_t)(dir * numPulses * anglePerPulse);
   }
   else if (encMode == RESOLVER)
   {
      GetAngleResolver();
   }
   else
   {
      static uint16_t lastAngle = 0;
      uint16_t angleDiff;

      if (encMode == SPI)
      {
         angle = GetAngleSPI(); //Gets 12-bit
         angle <<= 4;
         uint16_t diffPos = angle - lastAngle;
         uint16_t diffNeg = lastAngle - angle;
         angleDiff = MIN(diffNeg, diffPos);
      }
      else
      {
         uint32_t cntVal = timer_get_counter(REV_CNT_TIMER);
         cntVal *= TWO_PI;
         cntVal /= pulsesPerTurn * 4;
         angle = (uint16_t)cntVal;
         angleDiff = (angle - lastAngle) & 0xFFFF;
         if ((TIM_CR1(REV_CNT_TIMER) & TIM_CR1_DIR_DOWN))
            angleDiff = (lastAngle - angle) & 0xFFFF;
      }

      lastAngle = angle;
      turnsSinceLastSample += angleDiff;
      //Param::SetDig(Param::tm_meas, angle);
   }
}

/** Update rotor frequency.
 *
 * @param timeBase calling frequency in Hz
 */
void Encoder::UpdateRotorFrequency(int frq)
{
   //Found here: https://www.embeddedrelated.com/showarticle/530.php
   if ((encMode != SINGLE) && (encMode != RESOLVER) && frq > 0)
   {
      static int velest = 0, velint = 0;
      static uint16_t posest = 0;
      const int ki = 800, kp = 20;

      posest += velest / frq;
      int16_t poserr = angle - posest;
      velint += (poserr * ki) / frq;
      velest = poserr * kp + velint;
      lastFrequency = ABS(velint) / 1820;
   }
}

/** Returns current angle of motor shaft to some arbitrary 0-axis
 * @return angle in digit (2Pi=65536)
*/
uint16_t Encoder::GetRotorAngle(int dir)
{
   if (encMode == SINGLE)
   {
      uint32_t timeSinceLastPulse = timer_get_counter(REV_CNT_TIMER);
      uint16_t interpolatedAngle = ignore ? (anglePerPulse * timeSinceLastPulse) / lastPulseTimespan : 0;

      return angle + dir * interpolatedAngle;
   }
   else
   {
      return angle;
   }
}


/** Return rotor frequency in Hz
 * @pre in AB/ABZ encoder mode UpdateRotorFrequency must be called at a regular interval */
u32fp Encoder::GetRotorFrequency()
{
   if (encMode == SINGLE)
   {
      if (ignore) return 0;
      return FP_FROMINT(1000000) / (lastPulseTimespan * pulsesPerTurn);
   }
   else
   {
      return lastFrequency;
   }
}

/** Get current speed in rpm */
uint32_t Encoder::GetSpeed()
{
   if (encMode == SINGLE)
   {
      if (ignore) return 0;
      return 60000000 / (lastPulseTimespan * pulsesPerTurn);
   }
   else
   {
      return FP_TOINT(60 * lastFrequency);
   }
}

bool Encoder::IsSyncMode()
{
   return encMode == Encoder::ABZ || encMode == Encoder::SPI || encMode == Encoder::RESOLVER;
}

static void DMASetup()
{
   dma_disable_channel(DMA1, REV_CNT_DMACHAN);
   dma_set_peripheral_address(DMA1, REV_CNT_DMACHAN, (uint32_t)&REV_CNT_CCR);
   dma_set_memory_address(DMA1, REV_CNT_DMACHAN, (uint32_t)timdata);
   dma_set_peripheral_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_PSIZE_16BIT);
   dma_set_memory_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_MSIZE_16BIT);
   dma_set_number_of_data(DMA1, REV_CNT_DMACHAN, MAX_REVCNT_VALUES);
   dma_enable_memory_increment_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_circular_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_channel(DMA1, REV_CNT_DMACHAN);
}

static void InitTimerSingleChannelMode()
{
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
   //Some explanation: HCLK=72MHz
   //APB1-Prescaler is 2 -> 36MHz
   //Timer clock source is ABP1*2 because APB1 prescaler > 1
   //So clock source is 72MHz (again)
   //We want the timer to run at 1MHz = 72MHz/72
   //Prescaler is div-1 => 71
   timer_set_prescaler(REV_CNT_TIMER, 71);
   timer_set_period(REV_CNT_TIMER, MAX_CNT);
   timer_direction_up(REV_CNT_TIMER);

   /* Reset counter on input pulse. Filter constant must be larger than that of the capture input
      So that the counter value is first saved, then reset */
   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_RM); // reset mode
   timer_slave_set_polarity(REV_CNT_TIMER, TIM_ET_FALLING);
   timer_slave_set_trigger(REV_CNT_TIMER, TIM_SMCR_TS_ETRF);
   timer_slave_set_filter(REV_CNT_TIMER, TIM_IC_DTF_DIV_32_N_8);

   /* Save timer value on input pulse with smaller filter constant */
   timer_ic_set_filter(REV_CNT_TIMER, REV_CNT_IC, TIM_IC_DTF_DIV_32_N_6);
   timer_ic_set_input(REV_CNT_TIMER, REV_CNT_IC, TIM_IC_IN_TI1);
   TIM_CCER(REV_CNT_TIMER) |= REV_CNT_CCER; //No API function yet available
   timer_ic_enable(REV_CNT_TIMER, REV_CNT_IC);

   timer_enable_irq(REV_CNT_TIMER, REV_CNT_DMAEN);
   timer_set_dma_on_compare_event(REV_CNT_TIMER);

   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
   timer_enable_counter(REV_CNT_TIMER);
   gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);
   DMASetup();
   exti_disable_request(EXTI2);
}

static void InitSPIMode()
{
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
   exti_disable_request(EXTI2);
   gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
   seenNorthSignal = true;
}

static void InitTimerABZMode()
{
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
	timer_set_period(REV_CNT_TIMER, 4 * pulsesPerTurn); //2 channels and 2 edges -> 4 times the number of base pulses

	//In sync mode start out in reset mode and switch to encoder
	//mode once the north marker has been detected
	if (encMode == Encoder::ABZ)
   {
      exti_select_source(EXTI2, GPIOD);
      exti_set_trigger(EXTI2, EXTI_TRIGGER_RISING);
      exti_enable_request(EXTI2);
   }
   else
   {
      exti_disable_request(EXTI2);
   }

   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_EM3); // encoder mode
	timer_ic_set_input(REV_CNT_TIMER, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(REV_CNT_TIMER, TIM_IC2, TIM_IC_IN_TI2);
	timer_ic_set_filter(REV_CNT_TIMER, TIM_IC1, TIM_IC_DTF_DIV_32_N_8);
	timer_ic_set_filter(REV_CNT_TIMER, TIM_IC2, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC1);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC2);
	timer_enable_counter(REV_CNT_TIMER);
   gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);
   seenNorthSignal = false;
}

static void InitResolverMode()
{
   uint8_t channels[] = { 6, 6, 7, 7 };
   adc_enable_discontinuous_mode_injected(ADC1);
   adc_set_injected_sequence(ADC1, 4, channels);
   adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_JSWSTART);
   adc_set_sample_time(ADC1, 6, ADC_SMPR_SMP_1DOT5CYC);
   adc_set_sample_time(ADC1, 7, ADC_SMPR_SMP_1DOT5CYC);
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
   timer_set_prescaler(REV_CNT_TIMER, 71); //run at 1MHz
   timer_set_period(REV_CNT_TIMER, 65535);
   timer_one_shot_mode(REV_CNT_TIMER);
   timer_set_oc_mode(REV_CNT_TIMER, TIM_OC4, TIM_OCM_PWM2);
   timer_enable_oc_output(REV_CNT_TIMER, TIM_OC4); //OC4 is connected to ADC trigger
   timer_enable_preload(REV_CNT_TIMER);
   timer_direction_up(REV_CNT_TIMER);
   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);

   gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO6);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO7);
   exti_disable_request(EXTI2);

   adc_start_conversion_injected(ADC1);

   for (volatile int i = 0; i < 80000; i++);
   adc_start_conversion_injected(ADC1);

   for (volatile int i = 0; i < 80000; i++);
   //while (!adc_eoc_injected(ADC1));
   //while (!(ADC_SR(ADC1) & ADC_SR_JEOC));

   int cos = adc_read_injected(ADC1, 1);
   int sin = adc_read_injected(ADC1, 3);
   adc_set_injected_offset(ADC1, 1, cos);
   adc_set_injected_offset(ADC1, 2, cos);
   adc_set_injected_offset(ADC1, 3, sin);
   adc_set_injected_offset(ADC1, 4, sin);
   adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_TIM3_CC4);
   seenNorthSignal = true;
}

static uint16_t GetAngleSPI()
{
   uint32_t d = 0;
   //Skip the abstraction, we need speed here
   GPIO_BSRR(GPIOA) |= GPIO7; //Clock high
   GPIO_BSRR(GPIOD) |= GPIO2 << 16; //Read LOW = active

   for (int i = 15; i >= 0; i--)
   {
      GPIO_BSRR(GPIOA) |= GPIO7 << 16; //Clock low
      d |= ((int)GPIO_IDR(GPIOA) & GPIO6) << i;
      GPIO_BSRR(GPIOA) |= GPIO7;
   }
   GPIO_BSRR(GPIOD) |= GPIO2; //Read high
   d >>= 10; //6 because of GPIO6, 4 because of encoder format
   //Encoder format is ANGLE[11:0], RDVEL, Parity, DOS, LOT
   return d;
}

/** Calculates current angle and velocity from resolver feedback
 * @pre Must be called at 8.8kHz */
static uint16_t GetAngleResolver()
{
   static bool state = false;
   static uint16_t lastAngle;

   if (state)
   {
      gpio_clear(GPIOD, GPIO2);
      /* The phase delay of the 3-pole filter, amplifier and resolver is 96µs.
         That is 40µs after the falling edge of the exciting square wave */
      timer_set_oc_value(REV_CNT_TIMER, TIM_OC4, 40);
      timer_set_counter(REV_CNT_TIMER, 0);
      timer_enable_counter(REV_CNT_TIMER);
      state = false;
      angle = lastAngle;
   }
   else
   {
      gpio_set(GPIOD, GPIO2);
      int cos = adc_read_injected(ADC1, 1) + adc_read_injected(ADC1, 2);
      int sin = adc_read_injected(ADC1, 3) + adc_read_injected(ADC1, 4);
      angle = SineCore::Atan2(cos, sin);
      //Param::SetInt(Param::sin, sin);
      //Param::SetInt(Param::cos, cos);
      state = true;
      uint16_t diffPos = angle - lastAngle;
      uint16_t diffNeg = lastAngle - angle;
      uint16_t angleDiff = MIN(diffNeg, diffPos);
      angleDiff &= 0xFFF0; //Filter out some jitter
      u32fp frq = FP_MUL(angleDiff, FP_FROMFLT(2.148)); //32/(65536/4400)
      lastFrequency = IIRFILTER(lastFrequency, frq, filter);
      lastAngle = angle;
   }

   return angle;
}

extern "C" void exti2_isr(void)
{
	exti_reset_request(EXTI2);
   timer_set_counter(REV_CNT_TIMER, 0);
	seenNorthSignal = true;
}

static int GetPulseTimeFiltered()
{
   static int lastN = 0;
   static int noMovement = 0;
   uint16_t n = REV_CNT_DMA_CNDTR;
   uint16_t measTm = REV_CNT_CCR;
   int pulses = n <= lastN ? lastN - n : lastN + MAX_REVCNT_VALUES - n;
   uint32_t max = 0;
   uint32_t min = 0xFFFF;
   lastN = n;

   GetMinMaxTime(min, max);

   if (pulses > 0)
   {
      noMovement = 0;
      ignore = false;
   }
   else
   {
      noMovement++;
   }

   //If we haven't seen movement for 1000 cycles, we assume the motor is stopped
   //The time that 1000 cycles corresponds to is dependent from the calling
   //frequency, i.e. the PWM frequency. The results proved ok for 8.8kHz
   if (noMovement > 1000)
   {
      ignore = true;
   }

   //spike detection, a factor of 8 between adjacent pulses is most likely caused by interference
   if (max > (8 * min) && min > 0)
   {
      ErrorMessage::Post(ERR_ENCODER);
   }
   //a factor of 3 is still not stable, use the maximum
   else if (max > (3 * min))
   {
      lastPulseTimespan = max;
   }
   else
   {
      lastPulseTimespan = IIRFILTER(lastPulseTimespan, measTm, filter);
   }

   return pulses;
}

static void GetMinMaxTime(uint32_t& min, uint32_t& max)
{
   for (uint32_t i = 0; i < MAX_REVCNT_VALUES; i++)
   {
      min = MIN(min, timdata[i]);
      max = MAX(max, timdata[i]);
   }
}
