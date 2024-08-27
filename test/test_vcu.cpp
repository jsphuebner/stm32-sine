/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
#include "canhardware.h"
#include "params.h"
#include "errormessage.h"
#include "inc_encoder.h"
#include "digio.h"
#include "hwdefs.h"
#include "pwmgeneration.h"
#include "vehiclecontrol.h"
#include "stub_canhardware.h"
#include "test.h"

static uint32_t crc32_word(uint32_t Crc, uint32_t Data);

static uint32_t crc = 0;
static uint32_t rtc = 0;
static uint32_t speed = 0;
static ERROR_MESSAGE_NUM errorMessage;

class VCUTest: public UnitTest
{
   public:
      VCUTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
      virtual void TestCaseSetup();
};

static void FillInCanData(uint32_t* data, uint32_t pot, uint32_t pot2, uint32_t canio, uint32_t cruisespeed, uint32_t regenPreset, uint32_t seq)
{
   uint32_t crc = 0xFFFFFFFF;
   data[0] = pot| (pot2 << 12) | (canio << 24) | (seq << 30);
   data[1] = cruisespeed | (seq << 14) | (regenPreset << 16);
   crc = crc32_word(crc, data[0]);
   crc = crc32_word(crc, data[1]);
   data[1] |= crc << 24;
}

static void CanTest1()
{
   ASSERT(vcuCan != 0);
   ASSERT(vcuCanId == Param::GetInt(Param::controlid));
}

static void CanTest2()
{
   uint32_t data[2];

   FillInCanData(data, 100, 200, CAN_IO_FWD, 1000, 50, 1);

   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();

   ASSERT(Param::GetBool(Param::din_forward));
   ASSERT(!Param::GetBool(Param::din_reverse));
   ASSERT(Param::GetInt(Param::pot) == 0); //pot=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::pot2) == 0); //pot2=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::regenpreset) == 100); //Overwritten by "ADC" value
}

static void CanTest3()
{
   uint32_t data[2];

   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 50, 1);
   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);

   vcuCan->HandleRx(vcuCanId, data, 8);
   VehicleControl::GetDigInputs();
   VehicleControl::ProcessThrottle();

   ASSERT(Param::GetBool(Param::din_start));
   ASSERT(!Param::GetBool(Param::din_forward));
   ASSERT(Param::GetInt(Param::pot) == 100); //pot=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::pot2) == 200); //pot2=0 because throtmode is not CAN
   ASSERT(Param::GetInt(Param::regenpreset) == 50);
}

static void TestCanSeqError1()
{
   uint32_t data[2];

   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);
   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);
   //call again with same sequence counter -> triggers an error message
   FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
   vcuCan->HandleRx(vcuCanId, data, 8);

   ASSERT(errorMessage == ERR_CANCOUNTER);
   ASSERT(Param::GetInt(Param::regenpreset) == 60); //Only after 5 errors will this be reset to 0
}

static void TestCanSeqError2()
{
   uint32_t data[2];

   Param::SetInt(Param::potmode, POTMODE_DUALCHANNEL | POTMODE_CAN);

   for (int i = 0; i < 6; i++)
   {
      //Call 6 times with same sequence counter -> will trigger unrecoverable error
      FillInCanData(data, 100, 200, CAN_IO_START, 1000, 60, 1);
      vcuCan->HandleRx(vcuCanId, data, 8);
   }

   //Now simulate 500ms or 50 rtc ticks passed
   rtc = 51;
   VehicleControl::ProcessThrottle();
   VehicleControl::GetDigInputs();

   ASSERT(errorMessage == ERR_CANTIMEOUT);
   ASSERT(Param::GetInt(Param::regenpreset) == 0);
   ASSERT(!Param::GetBool(Param::din_start));
   ASSERT(Param::GetInt(Param::potnom) == 0);
}

void VCUTest::TestCaseSetup()
{
   VehicleControl::SetCan(new CanStub());
   Param::LoadDefaults();
}

REGISTER_TEST(VCUTest, CanTest1, CanTest2, CanTest3, TestCanSeqError1, TestCanSeqError2);

/* Stub functions */
extern "C" void crc_reset()
{
   crc = 0xFFFFFFFF;
}

extern "C" uint32_t crc_calculate_block(uint32_t* data, uint32_t len)
{
   while (len--)
      crc = crc32_word(crc, *(data++));
   return crc;
}

extern "C" uint32_t rtc_get_counter_val()
{
   return rtc;
}

extern "C" void timer_set_oc_value(uint32_t, enum tim_oc_id, uint32_t)
{

}

extern "C" void spi_setup()
{

}

extern "C" uint16_t gpio_get(uint32_t port, uint16_t pin)
{
   return 0;
}

extern "C" void gpio_set(uint32_t port, uint16_t pin)
{
}

extern "C" void gpio_clear(uint32_t port, uint16_t pin)
{
}

extern "C" uint16_t spi_xfer(uint32_t, uint16_t)
{
   return 0;
}

bool PwmGeneration::Tripped()
{
   return true;
}

DigIo DigIo::err_out;
DigIo DigIo::brk_out;
DigIo DigIo::dcsw_out;
DigIo DigIo::prec_out;
DigIo DigIo::vtg_out;
DigIo DigIo::fan_out;
DigIo DigIo::temp0_out;
DigIo DigIo::cruise_in;
DigIo DigIo::start_in;
DigIo DigIo::brake_in;
DigIo DigIo::mprot_in;
DigIo DigIo::fwd_in;
DigIo DigIo::rev_in;
DigIo DigIo::emcystop_in;
DigIo DigIo::bms_in;
DigIo DigIo::ocur_in;
DigIo DigIo::desat_in;
AnaIn AnaIn::uaux(0);
AnaIn AnaIn::udc(1);
AnaIn AnaIn::tmphs(2);
AnaIn AnaIn::tmpm(3);
AnaIn AnaIn::throttle1(4);
AnaIn AnaIn::throttle2(5);

void DigIo::Configure(uint32_t, uint16_t, PinMode::PinMode)
{

}

uint16_t AnaIn::Get()
{
   return 0;
}

void ErrorMessage::Post(ERROR_MESSAGE_NUM e)
{
   errorMessage = e;
}

uint32_t Encoder::GetSpeed()
{
   return speed;
}

u32fp Encoder::GetRotorFrequency()
{
   return speed;
}

void Encoder::ResetDistance()
{

}

int32_t Encoder::GetDistance()
{
   return 0;
}

int Encoder::GetRotorDirection()
{
   return 1;
}

void Param::Change(Param::PARAM_NUM p)
{

}

const char* errorListString = "";
HWREV hwRev = HW_REV3;
uint16_t AnaIn::values[NUM_SAMPLES*ANA_IN_COUNT];

static uint32_t crc32_word(uint32_t Crc, uint32_t Data)
{
  int i;

  Crc = Crc ^ Data;

  for(i=0; i<32; i++)
    if (Crc & 0x80000000)
      Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      Crc = (Crc << 1);

  return(Crc);
}
