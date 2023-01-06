/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 * Copyright (C) 2019 Nail Guzel
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
#include "printf.h"

#define TWO_PI            65536
//Angle difference at which we assume jitter to become irrelevant
#define STABLE_ANGLE      ((10 * TWO_PI) / 360)
#define MAX_CNT           TWO_PI - 1
#define MAX_REVCNT_VALUES 5
#define MIN_RES_AMP       1000

#define FRQ_TO_PSC(frq) ((72000000 / frq) - 1)
#define NUM_ENCODER_CONFIGS (sizeof(encoderConfigurations) / sizeof(encoderConfigurations[0]))

typedef struct EncoderConfiguration
{
   uint32_t pulseMeasFrequency;
   enum tim_ic_filter resetFilter;
   enum tim_ic_filter captureFilter;
   uint16_t maxPpr;
} ENCODER_CONFIG;

static const ENCODER_CONFIG encoderConfigurations[] =
{
   { 1000000, TIM_IC_DTF_DIV_32_N_8, TIM_IC_DTF_DIV_32_N_6, 60   }, // CFG1 - this will be selected for numimp <= 60
   { 2400000, TIM_IC_DTF_DIV_2_N_8,  TIM_IC_DTF_DIV_2_N_6,  125  }, // CFG2 - this will be selected for numimp <= 125
   { 7200000, TIM_IC_DTF_DIV_2_N_8,  TIM_IC_DTF_DIV_2_N_6,  8192 }, // CFG3 - this will be selected for numimp <= 8192
};

static const ENCODER_CONFIG * selectedConfig = &encoderConfigurations[0];
static uint32_t pulseMeasFrq = selectedConfig->pulseMeasFrequency;

//Delay in us between generating an edge on the exciter output and measuring the
//Return values via ADC - Found this by scoping
static const uint16_t resolverSampleDelay = 40;
static volatile uint16_t timdata[MAX_REVCNT_VALUES];
static volatile uint16_t angle = 0;
static uint16_t pulsesPerTurn = 0;
static uint32_t lastPulseTimespan = 0;
static uint32_t anglePerPulse = 0;
static uint32_t fullTurns = 0;
static uint32_t pwmFrq = 1;
static u32fp lastFrequency = 0;
static bool ignore = true;
static enum Encoder::mode encMode = Encoder::INVALID;
static bool seenNorthSignal = false;
static int32_t turnsSinceLastSample = 0;
static int32_t distance = 0;
static int32_t resolverMin = 0, resolverMax = 0, startupDelay;
static int32_t sinChan = 3, cosChan = 2;
static int32_t detectedDirection = 0;
static uint16_t sincosoffs = 2048;

void Encoder::Reset()
{
   ignore = true;
   lastPulseTimespan = MAX_CNT;
   resolverMin = 0;
   resolverMax = 0;
   lastFrequency = 0;
   detectedDirection = 0;
   startupDelay = 4000;
   for (uint32_t i = 0; i < MAX_REVCNT_VALUES; i++)
      timdata[i] = MAX_CNT;
}

/** Since power up, have we seen the north marker? */
bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}

/** Set type of motor position feedback
 * @param mode type of motor position feedback */
void Encoder::SetMode(Encoder::mode mode)
{
   if (encMode == mode) return;

   encMode = mode;

   switch (mode)
   {
      case AB:
      case ABZ:
         InitTimerABZMode();
         break;
      case SINGLE:
         InitTimerSingleChannelMode();
         break;
      case SPI:
         InitSPIMode();
         break;
      case RESOLVER:
      case SINCOS:
         InitResolverMode();
         break;
      default:
         break;
   }
}

void Encoder::SetPwmFrequency(uint32_t frq)
{
   pwmFrq = frq;
}

void Encoder::SetSinCosOffset(uint16_t offset)
{
   sincosoffs = offset;
}

/** set number of impulses per shaft rotation
  */
void Encoder::SetImpulsesPerTurn(uint16_t imp)
{
   if (imp == pulsesPerTurn) return;

   pulsesPerTurn = imp;
   anglePerPulse = TWO_PI / imp;

   if (encMode == SINGLE)
      InitTimerSingleChannelMode();

   if (encMode == AB || encMode == ABZ)
      InitTimerABZMode();
}

void Encoder::SwapSinCos(bool swap)
{
   if (swap)
   {
      sinChan = 2;
      cosChan = 3;
   }
   else
   {
      sinChan = 3;
      cosChan = 2;
   }
}

void Encoder::UpdateRotorAngle(int dir)
{
   static uint16_t lastAngle = 0;
   static uint16_t accumulatedAngle = 0;
   static int poleCounter = 0;
   uint32_t cntVal;
   int16_t numPulses;
   uint32_t timeSinceLastPulse;
   uint16_t interpolatedAngle;
   uint16_t angleDiff;

   switch (encMode)
   {
      case AB:
      case ABZ:
         cntVal = timer_get_counter(REV_CNT_TIMER);
         cntVal *= TWO_PI;
         cntVal /= pulsesPerTurn * 4;
         angle = (uint16_t)cntVal;
         detectedDirection = (TIM_CR1(REV_CNT_TIMER) & TIM_CR1_DIR_DOWN) ? -1 : 1;
         angleDiff = (detectedDirection < 0 ?
                     (lastAngle - angle) : (angle - lastAngle)) & 0xFFFF;
         turnsSinceLastSample += angleDiff;
         break;
      case SINGLE:
         numPulses = GetPulseTimeFiltered();
         timeSinceLastPulse = timer_get_counter(REV_CNT_TIMER);
         interpolatedAngle = ignore ? 0 : (anglePerPulse * timeSinceLastPulse) / lastPulseTimespan;
         accumulatedAngle += (int16_t)(dir * numPulses * anglePerPulse);
         angle = accumulatedAngle + dir * interpolatedAngle;
         lastFrequency = ignore ? lastFrequency : FP_FROMINT(pulseMeasFrq) / (lastPulseTimespan * pulsesPerTurn);
         detectedDirection = dir;
         break;
      case SPI:
         angle = GetAngleSPI();
         UpdateTurns(angle, lastAngle);
         break;
      case RESOLVER:
         angle = GetAngleResolver();
         UpdateTurns(angle, lastAngle);
         break;
      case SINCOS:
         angle = GetAngleSinCos();
         UpdateTurns(angle, lastAngle);
         break;
      default:
         break;
   }

   if (lastAngle <= (TWO_PI / 2) && angle > (TWO_PI / 2))
   {
      if (poleCounter == 0)
      {
         fullTurns++;
         poleCounter = Param::GetInt(Param::respolepairs);
      }
      else
      {
         poleCounter--;
      }
   }

   startupDelay = startupDelay > 0 ? startupDelay - 1 : 0;
   lastAngle = angle;
}

/** Update rotor frequency.
 * @param callingFrequency Frequency at which this function is called in Hz
 */
void Encoder::UpdateRotorFrequency(int callingFrequency)
{
   distance += turnsSinceLastSample;

   if ((encMode == AB) || (encMode == ABZ))
   {
      //65536 is one turn
      lastFrequency = (callingFrequency * turnsSinceLastSample) / FP_TOINT(TWO_PI);
      turnsSinceLastSample = 0;
   }
   else if ((encMode == RESOLVER) || (encMode == SPI) || (encMode == SINCOS))
   {
      int absTurns = ABS(turnsSinceLastSample);
      if (startupDelay == 0 && absTurns > STABLE_ANGLE)
      {
         lastFrequency = (callingFrequency * absTurns) / FP_TOINT(TWO_PI);
         detectedDirection = turnsSinceLastSample > 0 ? 1 : -1;
      }
      else
      {
         lastFrequency = 0;
      }
      turnsSinceLastSample = 0;
   }
}

/** Returns current angle of motor shaft to some arbitrary 0-axis
 * @return angle in digit (2Pi=65536)
*/
uint16_t Encoder::GetRotorAngle()
{
   return angle;
}

/** Return rotor frequency in Hz
 * @pre in AB/ABZ encoder mode UpdateRotorFrequency must be called at a regular interval */
u32fp Encoder::GetRotorFrequency()
{
   return lastFrequency;
}

void Encoder::ResetDistance()
{
   distance = 0;
}

int32_t Encoder::GetDistance()
{
   return distance;
}

int Encoder::GetRotorDirection()
{
   return detectedDirection;
}

/** Get current speed in rpm */
uint32_t Encoder::GetSpeed()
{
   if (encMode == RESOLVER || encMode == SINCOS)
   {
      return FP_TOINT(60 * lastFrequency) / Param::GetInt(Param::respolepairs);
   }
   else
   {
      return FP_TOINT(60 * lastFrequency);
   }
}

/** Get number of rotor turns since power up */
uint32_t Encoder::GetFullTurns()
{
   return fullTurns;
}

void Encoder::UpdateTurns(uint16_t angle, uint16_t lastAngle)
{
   int signedDiff = (int)angle - (int)lastAngle;
   int absDiff = ABS(signedDiff);
   int sign = signedDiff < 0 ? -1 : 1;

   if (absDiff > (TWO_PI / 2)) //wrap detection
   {
      sign = -sign;
      signedDiff += sign * TWO_PI;
      absDiff = ABS(signedDiff);
   }

   turnsSinceLastSample += signedDiff;
}

void Encoder::InitTimerSingleChannelMode()
{
   const ENCODER_CONFIG* currentConfig = &encoderConfigurations[0];

   for (uint8_t i = 0; i < NUM_ENCODER_CONFIGS; i++, currentConfig++)
   {
      if (pulsesPerTurn <= currentConfig->maxPpr)
      {
         if (selectedConfig != currentConfig)
         {
            debugf("Reconfiguring encoder timer for CFG%d", (i+1));
         }
         selectedConfig = currentConfig;
         pulseMeasFrq = selectedConfig->pulseMeasFrequency;
         break;
      }
   }

   rcc_periph_reset_pulse(REV_CNT_TIMRST);

   //Some explanation: HCLK=72MHz
   //APB1-Prescaler is 2 -> 36MHz
   //Timer clock source is ABP1*2 because APB1 prescaler > 1
   //So clock source is 72MHz (again)
   //We want the timer to run at 1MHz = 72MHz/72
   //Prescaler is div-1 => 71

   timer_set_prescaler(REV_CNT_TIMER, FRQ_TO_PSC(pulseMeasFrq));
   timer_set_period(REV_CNT_TIMER, MAX_CNT);
   timer_direction_up(REV_CNT_TIMER);

   /* Reset counter on input pulse. Filter constant must be larger than that of the capture input
      So that the counter value is first saved, then reset */
   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_RM); // reset mode
   timer_slave_set_polarity(REV_CNT_TIMER, TIM_ET_FALLING);

   if (hwRev == HW_REV1)
   {
      //Since channel 3 is not connected to slave mmode controller, we have to use the ETR pin
      timer_slave_set_trigger(REV_CNT_TIMER, TIM_SMCR_TS_ETRF);
      timer_slave_set_filter(REV_CNT_TIMER, selectedConfig->resetFilter);
      gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
   }
   else
   {
      //ETR pin not needed, can use channel 1 (after filter)
      timer_slave_set_trigger(REV_CNT_TIMER, TIM_SMCR_TS_TI1FP1);
      gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6 | GPIO7);
   }

   /* Save timer value on input pulse with smaller filter constant */
   timer_ic_set_filter(REV_CNT_TIMER, REV_CNT_IC, selectedConfig->captureFilter);
   timer_ic_set_input(REV_CNT_TIMER, REV_CNT_IC, TIM_IC_IN_TI1);
   timer_set_oc_polarity_low(REV_CNT_TIMER, REV_CNT_OC);
   timer_ic_enable(REV_CNT_TIMER, REV_CNT_IC);

   timer_enable_irq(REV_CNT_TIMER, REV_CNT_DMAEN);
   timer_set_dma_on_compare_event(REV_CNT_TIMER);

   timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
   timer_enable_counter(REV_CNT_TIMER);
   gpio_set_mode(NORTH_EXC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, NORTH_EXC_PIN);
   exti_disable_request(NORTH_EXC_EXTI);

   dma_disable_channel(DMA1, REV_CNT_DMACHAN);
   dma_set_peripheral_address(DMA1, REV_CNT_DMACHAN, REV_CNT_CCR_PTR);
   dma_set_memory_address(DMA1, REV_CNT_DMACHAN, (uint32_t)timdata);
   dma_set_peripheral_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_PSIZE_16BIT);
   dma_set_memory_size(DMA1, REV_CNT_DMACHAN, DMA_CCR_MSIZE_16BIT);
   dma_set_number_of_data(DMA1, REV_CNT_DMACHAN, MAX_REVCNT_VALUES);
   dma_enable_memory_increment_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_circular_mode(DMA1, REV_CNT_DMACHAN);
   dma_enable_channel(DMA1, REV_CNT_DMACHAN);
}

void Encoder::InitSPIMode()
{
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
   exti_disable_request(NORTH_EXC_EXTI);
   gpio_set_mode(NORTH_EXC_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NORTH_EXC_PIN);
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);
   seenNorthSignal = true;
}

void Encoder::InitTimerABZMode()
{
   rcc_periph_reset_pulse(REV_CNT_TIMRST);
   timer_set_period(REV_CNT_TIMER, 4 * pulsesPerTurn - 1); //2 channels and 2 edges -> 4 times the number of base pulses

   //In sync mode start out in reset mode and switch to encoder
   //mode once the north marker has been detected
   if (encMode == Encoder::ABZ)
   {
      exti_select_source(NORTH_EXC_EXTI, NORTH_EXC_PORT);
      exti_set_trigger(NORTH_EXC_EXTI, EXTI_TRIGGER_RISING);
      exti_enable_request(NORTH_EXC_EXTI);
   }
   else
   {
      exti_disable_request(NORTH_EXC_EXTI);
   }

   timer_slave_set_mode(REV_CNT_TIMER, TIM_SMCR_SMS_EM3); // encoder mode
   timer_ic_set_input(REV_CNT_TIMER, TIM_IC1, TIM_IC_IN_TI1);
   timer_ic_set_input(REV_CNT_TIMER, TIM_IC2, TIM_IC_IN_TI2);
   timer_ic_set_filter(REV_CNT_TIMER, TIM_IC1, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_set_filter(REV_CNT_TIMER, TIM_IC2, TIM_IC_DTF_DIV_32_N_8);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC1);
   timer_ic_enable(REV_CNT_TIMER, TIM_IC2);
   timer_enable_counter(REV_CNT_TIMER);
   gpio_set_mode(NORTH_EXC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, NORTH_EXC_PIN);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6 | GPIO7);
   seenNorthSignal = false;
}

void Encoder::InitResolverMode()
{
   //The first injected channel is always noisy, so we insert one dummy channel
   uint8_t channels[3] = { 6, 6, 7 };

   adc_set_injected_sequence(ADC1, sizeof(channels), channels);
   adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_JSWSTART);
   adc_set_sample_time(ADC1, 6, ADC_SMPR_SMP_1DOT5CYC);
   adc_set_sample_time(ADC1, 7, ADC_SMPR_SMP_1DOT5CYC);

   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO6 | GPIO7);
   exti_disable_request(NORTH_EXC_EXTI);

   if (encMode == RESOLVER)
   {
      rcc_periph_reset_pulse(REV_CNT_TIMRST);
      timer_set_prescaler(REV_CNT_TIMER, 71); //run at 1MHz
      timer_set_period(REV_CNT_TIMER, 65535);
      timer_one_shot_mode(REV_CNT_TIMER);
      timer_set_oc_mode(REV_CNT_TIMER, TIM_OC4, TIM_OCM_PWM2);
      timer_enable_oc_output(REV_CNT_TIMER, TIM_OC4); //OC4 is connected to ADC trigger
      timer_enable_preload(REV_CNT_TIMER);
      timer_direction_up(REV_CNT_TIMER);
      timer_generate_event(REV_CNT_TIMER, TIM_EGR_UG);
      gpio_set_mode(NORTH_EXC_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NORTH_EXC_PIN);
      adc_set_injected_offset(ADC1, 2, 0);
      adc_set_injected_offset(ADC1, 3, 0);

      adc_start_conversion_injected(ADC1); //Determine offset

      while (!adc_eoc_injected(ADC1));

      int ch1 = adc_read_injected(ADC1, 2);
      int ch2 = adc_read_injected(ADC1, 3);
      adc_set_injected_offset(ADC1, 2, ch1);
      adc_set_injected_offset(ADC1, 3, ch2);
      adc_enable_external_trigger_injected(ADC1, ADC_CR2_JEXTSEL_TIM3_CC4);

      if (CHK_BIPOLAR_OFS(ch1) || CHK_BIPOLAR_OFS(ch2))
      {
         ErrorMessage::Post(ERR_HIRESOFS);
      }
   }
   else //SINCOS
   {
      //Offset assumed 3.3V/2 - 2048
      //on my hardware, min is 0.465V, max is 2.510v, so offset is 1.4875v, or 1846
      //this should be a parameter?
      adc_set_injected_offset(ADC1, 2, sincosoffs);
      adc_set_injected_offset(ADC1, 3, sincosoffs);
   }

   seenNorthSignal = true;
}

/** Gets angle from an AD2S chip */
uint16_t Encoder::GetAngleSPI()
{
   uint32_t d = 0;
   //Skip the abstraction, we need speed here
   GPIO_BSRR(GPIOA) = GPIO7; //Clock high
   GPIO_BRR(NORTH_EXC_PORT) = NORTH_EXC_PIN;

   for (int i = 15; i >= 0; i--)
   {
      GPIO_BRR(GPIOA) = GPIO7;
      uint32_t bit = ((uint32_t)GPIO_IDR(GPIOA) & GPIO6);
      GPIO_BSRR(GPIOA) = GPIO7;
      d |= bit << i;
   }
   GPIO_BSRR(NORTH_EXC_PORT) = NORTH_EXC_PIN; //Read high
   d >>= 10; //6 because of GPIO6, 4 because of encoder format
   //Encoder format is ANGLE[11:0], RDVEL, Parity, DOS, LOT
   return d << 4; //we want 16-bit representation
}

/** Calculates current angle from resolver feedback
 *  and generates square wave that is filtered
 *  into a sine wave for resolver excitation
 */
uint16_t Encoder::GetAngleResolver()
{
   if (gpio_get(NORTH_EXC_PORT, NORTH_EXC_PIN))
   {
      gpio_clear(NORTH_EXC_PORT, NORTH_EXC_PIN);
      /* The phase delay of the 3-pole filter, amplifier and resolver is 305 degrees
         That is 125 degrees after the falling edge of the exciting square wave */
      timer_set_oc_value(REV_CNT_TIMER, TIM_OC4, resolverSampleDelay);
      timer_set_counter(REV_CNT_TIMER, 0);
      timer_enable_counter(REV_CNT_TIMER);
      angle = DecodeAngle(true);
   }
   else
   {
      gpio_set(NORTH_EXC_PORT, NORTH_EXC_PIN);
      timer_set_counter(REV_CNT_TIMER, 0);
      timer_enable_counter(REV_CNT_TIMER);
      angle = DecodeAngle(false);
   }

   return angle;
}

/** Calculates angle from a Hall sin/cos encoder like MLX90380 */
uint16_t Encoder::GetAngleSinCos()
{
   uint16_t calcAngle = DecodeAngle(false);

   adc_start_conversion_injected(ADC1);

   return calcAngle;
}

/** Calculates angle from sin and cos value
 * @param invert flip values to positive side in order to read a resolver modulated signal on negative edge
*/
uint16_t Encoder::DecodeAngle(bool invert)
{
   int sin = adc_read_injected(ADC1, sinChan);
   int cos = adc_read_injected(ADC1, cosChan);

   //Wait for signal to reach usable amplitude
   if ((resolverMax - resolverMin) > MIN_RES_AMP)
   {
      if (invert)
         return SineCore::Atan2(-sin, -cos);
      return SineCore::Atan2(sin, cos);
   }
   else
   {
      int temp = MIN(sin, cos);
      resolverMin = MIN(temp, resolverMin);
      temp = MAX(sin, cos);
      resolverMax = MAX(temp, resolverMax);

      if (0 == startupDelay)
      {
         ErrorMessage::Post(ERR_LORESAMP);
      }
      return 0;
   }
}

int Encoder::GetPulseTimeFiltered()
{
   static int lastN = 0;
   static int noMovement = 0;
   int n = dma_get_number_of_data(DMA1, REV_CNT_DMACHAN);
   int measTm = timer_get_ic_value(REV_CNT_TIMER, REV_CNT_IC);
   int pulses = n <= lastN ? lastN - n : lastN + MAX_REVCNT_VALUES - n;
   int max = 0;
   int min = 0xFFFF;
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
      lastFrequency = 0;
      for (uint32_t i = 0; i < MAX_REVCNT_VALUES; i++)
         timdata[i] = MAX_CNT;
   }

   //spike detection, a factor of 8 between adjacent pulses is most likely caused by interference
   if (max > (8 * min) && min > 0)
   {
      ErrorMessage::Post(ERR_ENCODER);
   }
   //a factor of 2 is still not stable, use the maximum
   else if (max > (2 * min))
   {
      //lastPulseTimespan = max;
   }
   else
   {
      lastPulseTimespan = IIRFILTER(lastPulseTimespan, measTm + 1, 1);
   }

   return pulses;
}

void Encoder::GetMinMaxTime(int& min, int& max)
{
   for (int i = 0; i < MAX_REVCNT_VALUES; i++)
   {
      min = MIN(min, timdata[i]);
      max = MAX(max, timdata[i]);
   }
}

extern "C" void exti2_isr(void)
{
   exti_reset_request(EXTI2);
   timer_set_counter(REV_CNT_TIMER, 0);
   seenNorthSignal = true;
}

extern "C" void exti15_10_isr(void)
{
   exti_reset_request(EXTI14);
   timer_set_counter(REV_CNT_TIMER, 0);
   seenNorthSignal = true;
}
