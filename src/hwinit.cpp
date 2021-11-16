/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/desig.h>
#include "hwdefs.h"
#include "hwinit.h"
#include "stm32_loader.h"
#include "my_string.h"
#include "digio.h"
#include "anain.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

   //Run ADC at 12 MHz
   rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_TIM1); //Main PWM
   rcc_periph_clock_enable(RCC_TIM2); //Scheduler, over current on blue pill
   rcc_periph_clock_enable(RCC_TIM3); //Rotor Encoder
   rcc_periph_clock_enable(RCC_TIM4); //Overcurrent / AUX PWM, scheduler on blue pill
   rcc_periph_clock_enable(RCC_DMA1);  //ADC, Encoder and UART3
   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO); //CAN
   rcc_periph_clock_enable(RCC_CAN1); //CAN
   rcc_periph_clock_enable(RCC_SPI1); //Only needed for i3 inverter but we always enable it
}

void spi_setup()
{
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);

   spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);

   spi_enable_software_slave_management(SPI1);
   spi_set_nss_high(SPI1);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO3);
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4);
   spi_enable(SPI1);
}

static bool is_floating(uint32_t port, uint16_t pin)
{
   //A pin is considered floating if its state can be controlled with the internal 30k pull-up/down resistor
   bool isFloating = false;
   gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, pin);
   gpio_set(port, pin); //pull up with internal ~30k
   for (volatile int i = 0; i < 80000; i++);
   isFloating = gpio_get(port, pin);
   gpio_clear(port, pin);
   for (volatile int i = 0; i < 80000; i++);
   isFloating &= gpio_get(port, pin) == 0;
   return isFloating;
}

//Careful when using this function, it configures the given pin as output!
static bool is_existent(uint32_t port, uint16_t pin)
{
   gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pin);
   gpio_set(port, pin);
   for (volatile int i = 0; i < 80000; i++);
   bool isExistent = gpio_get(port, pin);
   gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin);

   return isExistent;
}

HWREV detect_hw()
{
   //Pull low via 30k the precharge output to test whether it is tied to Vcc
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO1);

   if (!is_existent(GPIOC, GPIO12)) //Olimex LED pin does not exist
      return HW_BLUEPILL;
   /*else if (is_floating(GPIOC, GPIO1))
      return HW_PRIUSMG1;*/
   //else if (gpio_get(GPIOB, GPIO1)) //On Tesla M3 board precharge output is tied to Vcc
     // return HW_TESLAM3;
   else if (is_floating(GPIOC, GPIO9)) //Desat pin is floating
      return HW_REV1;
   else if (is_floating(GPIOB, GPIO5)) //Cruise pin is floating
      return HW_PRIUS;
   else if (is_floating(GPIOA, GPIO0)) //uvlo/button pin is floating
      return HW_REV3;
   else if (is_floating(GPIOC, GPIO8)) //bms input is output for mux and floating
      return HW_TESLA;
   else
      return HW_REV2;
}

HWREV io_setup()
{
   hwRev = detect_hw();

   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   switch (hwRev)
   {
      case HW_REV1:
         AnaIn::il2.Configure(GPIOA, 6);
         break;
      case HW_REV2:
      case HW_REV3:
         break;
      case HW_PRIUS:
         DigIo::emcystop_in.Configure(GPIOC, GPIO7, PinMode::INPUT_PU);
         break;
      case HW_TESLA:
         DigIo::temp1_out.Configure(GPIOC, GPIO8, PinMode::OUTPUT);
         //Essentially disable error output by mapping it to an unused pin
         DigIo::err_out.Configure(GPIOB, GPIO9, PinMode::INPUT_FLT);
         break;
      case HW_BLUEPILL:
         ANA_IN_CONFIGURE(ANA_IN_LIST_BLUEPILL);
         DIG_IO_CONFIGURE(DIG_IO_BLUEPILL);
         break;
   }

   AnaIn::Start();

   return hwRev;
}

uint16_t pwmio_setup(bool activeLow)
{
   uint8_t outputMode = activeLow ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_50_MHZ;
   uint8_t outputConf = activeLow ? GPIO_CNF_INPUT_FLOAT : GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;
   uint16_t actualPattern = gpio_get(GPIOA, GPIO8 | GPIO9 | GPIO10) | gpio_get(GPIOB, GPIO13 | GPIO14 | GPIO15);

   gpio_set_mode(GPIOA, outputMode, outputConf, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, outputMode, outputConf, GPIO13 | GPIO14 | GPIO15);

   return actualPattern;
}

void write_bootloader_pininit(bool bootprec)
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* flashCommands = (struct pincommands*)pindefAddr;

   struct pincommands commands;

   memset32((int*)&commands, 0, PINDEF_NUMWORDS);

   commands.pindef[0].port = GPIOC;
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 0;
   commands.pindef[1].port = GPIOB;
   commands.pindef[1].inout = PIN_OUT;
   commands.pindef[1].level = bootprec;

   if (hwRev == HW_BLUEPILL)
   {
      commands.pindef[0].pin = GPIO15;
      commands.pindef[1].pin = GPIO4;
   }
   else
   {
      commands.pindef[0].pin = GPIO13;
      commands.pindef[1].pin = GPIO1;
   }

   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)&commands), PINDEF_NUMWORDS);
   commands.crc = crc;

   if (commands.crc != flashCommands->crc)
   {
      flash_unlock();
      flash_erase_page(pindefAddr);

      //Write flash including crc, therefor <=
      for (uint32_t idx = 0; idx <= PINDEF_NUMWORDS; idx++)
      {
         uint32_t* pData = ((uint32_t*)&commands) + idx;
         flash_program_word(pindefAddr + idx * sizeof(uint32_t), *pData);
      }
      flash_lock();
   }
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(PWM_TIMER_IRQ); //Main PWM
   nvic_set_priority(PWM_TIMER_IRQ, 1 << 4); //Set second-highest priority

   nvic_enable_irq(NVIC_TIM1_BRK_IRQ); //Emergency shut down
   nvic_set_priority(NVIC_TIM1_BRK_IRQ, 0); //Highest priority

   nvic_enable_irq(NVIC_EXTI2_IRQ); //Encoder Index pulse
   nvic_set_priority(NVIC_EXTI2_IRQ, 0); //Set highest priority

   if (hwRev == HW_BLUEPILL)
   {
      nvic_enable_irq(NVIC_TIM4_IRQ); //Scheduler
      nvic_set_priority(NVIC_TIM4_IRQ, 0xe << 4); //second lowest priority
   }
   else
   {
      nvic_enable_irq(NVIC_TIM2_IRQ); //Scheduler
      nvic_set_priority(NVIC_TIM2_IRQ, 0xe << 4); //second lowest priority
   }
}

void rtc_setup()
{
   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
   //62.5kHz / (624 + 1) = 100Hz
   rtc_auto_awake(RCC_HSE, 624); //10ms tick
   rtc_set_counter_val(0);
}

/**
* Setup main PWM timer and timer for generating over current
* reference values and external PWM
*/
void tim_setup()
{
   /*** Setup over/undercurrent and PWM output timer */
   timer_disable_counter(OVER_CUR_TIMER);
   //edge aligned PWM
   timer_set_alignment(OVER_CUR_TIMER, TIM_CR1_CMS_EDGE);
   timer_enable_preload(OVER_CUR_TIMER);
   /* PWM mode 1 and preload enable */
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC1, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_mode(OVER_CUR_TIMER, TIM_OC4, TIM_OCM_PWM1);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC1);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC3);
   timer_enable_oc_preload(OVER_CUR_TIMER, TIM_OC4);

   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC1);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC3);
   timer_set_oc_polarity_high(OVER_CUR_TIMER, TIM_OC4);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC1);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC2);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC3);
   timer_enable_oc_output(OVER_CUR_TIMER, TIM_OC4);
   timer_generate_event(OVER_CUR_TIMER, TIM_EGR_UG);
   timer_set_prescaler(OVER_CUR_TIMER, 0);

   /* PWM frequency */
   if (hwRev == HW_PRIUS)
   {
      //It is used for driving the booster converter at 8.8 kHz.
      timer_set_period(OVER_CUR_TIMER, OCURMAX * 2);
   }
   else
   {
      timer_set_period(OVER_CUR_TIMER, OCURMAX);
   }

   timer_enable_counter(OVER_CUR_TIMER);

   /** setup gpio */
   if (hwRev == HW_BLUEPILL)
   {
      gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO15);
      gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO3);
      gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP1);
   }
   else
   {
      gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8 | GPIO9);
   }
}

