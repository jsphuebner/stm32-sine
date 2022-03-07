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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include "vehiclecontrol.h"
#include "temp_meas.h"
#include "fu.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "anain.h"
#include "inc_encoder.h"
#include "throttle.h"
#include "my_math.h"
#include "pwmgeneration.h"
#include "hwinit.h"

#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms
#define ADC_CHAN_UDC      3
#define MAP(x, in_min, in_max, out_min,out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


Can* VehicleControl::can;
bool VehicleControl::lastCruiseSwitchState = false;
bool VehicleControl::canIoActive = false;
bool VehicleControl::spiEnabled = false;
int VehicleControl::temphsFiltered = 0;
int VehicleControl::tempmFiltered = 0;
int VehicleControl::udcFiltered = 0;
uint16_t VehicleControl::bmwAdcNextChan = 0;
uint16_t VehicleControl::bmwAdcValues[4];

void VehicleControl::PostErrorIfRunning(ERROR_MESSAGE_NUM err)
{
   if (Param::GetInt(Param::opmode) == MOD_RUN)
   {
      ErrorMessage::Post(err);
   }
}

void VehicleControl::CruiseControl()
{
   //Always disable cruise control when brake pedal is pressed
   if (Param::GetBool(Param::din_brake))
   {
      Throttle::cruiseSpeed = -1;
   }
   else
   {
      if (Param::GetInt(Param::cruisemode) == CRUISE_BUTTON)
      {
         //Enable/update cruise control when button is pressed
         if (Param::GetBool(Param::din_cruise))
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }
      }
      else if (Param::GetInt(Param::cruisemode) == CRUISE_SWITCH)
      {
         //Enable/update cruise control when switch is toggled on
         if (Param::GetBool(Param::din_cruise) && !lastCruiseSwitchState)
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }

         //Disable cruise control when switch is off
         if (!Param::GetBool(Param::din_cruise))
         {
            Throttle::cruiseSpeed = -1;
         }
      }
      else if (Param::GetInt(Param::cruisemode) == CRUISE_CAN)
      {
         Throttle::cruiseSpeed = Param::GetInt(Param::cruisespeed);
      }
   }

   if (Param::GetInt(Param::cruisemode) != CRUISE_CAN)
   {
      Param::SetInt(Param::cruisespeed, Throttle::cruiseSpeed);
   }

   lastCruiseSwitchState = Param::GetBool(Param::din_cruise);
}

void VehicleControl::SelectDirection()
{
   int selectedDir = Param::GetInt(Param::dir);
   int userDirSelection = 0;
   int dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;

   //When in bidirection throttle mode, direction is determined by that
   if (Param::GetInt(Param::potmode) & POTMODE_BIDIR) return;

   if (Param::GetInt(Param::dirmode) == DIR_DEFAULTFORWARD)
   {
      if (Param::GetBool(Param::din_forward) && Param::GetBool(Param::din_reverse))
         selectedDir = 0;
      else if (Param::GetBool(Param::din_reverse))
         userDirSelection = -1;
      else
         userDirSelection = 1;
   }
   else if ((Param::GetInt(Param::dirmode) & 1) == DIR_BUTTON)
   {
      /* if forward AND reverse selected, force neutral, because it's charge mode */
      if (Param::GetBool(Param::din_forward) && Param::GetBool(Param::din_reverse))
         selectedDir = 0;
      else if (Param::GetBool(Param::din_forward))
         userDirSelection = 1 * dirSign;
      else if (Param::GetBool(Param::din_reverse))
         userDirSelection = -1 * dirSign;
      else
         userDirSelection = selectedDir;
   }
   else
   {
      /* neither forward nor reverse or both forward and reverse -> neutral */
      if (!(Param::GetBool(Param::din_forward) ^ Param::GetBool(Param::din_reverse)))
         selectedDir = 0;
      else if (Param::GetBool(Param::din_forward))
         userDirSelection = 1 * dirSign;
      else if (Param::GetBool(Param::din_reverse))
         userDirSelection = -1 * dirSign;
   }

   /* Only change direction when below certain motor speed */
   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::dirchrpm))
      selectedDir = userDirSelection;

   /* Current direction doesn't match selected direction -> neutral */
   if (selectedDir != userDirSelection)
      selectedDir = 0;

   Param::SetInt(Param::dir, selectedDir);
}

float VehicleControl::ProcessThrottle()
{
   float throtSpnt, finalSpnt;

   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::throtramprpm))
      Throttle::throttleRamp = Param::GetFloat(Param::throtramp);
   else
      Throttle::throttleRamp = FP_TOFLOAT(Param::GetAttrib(Param::throtramp)->max);

   throtSpnt = GetUserThrottleCommand();
   bool determineDirection = GetCruiseCreepCommand(finalSpnt, throtSpnt);
   finalSpnt = Throttle::RampThrottle(finalSpnt);

   if (hwRev != HW_TESLA)
      Throttle::BmsLimitCommand(finalSpnt, Param::GetBool(Param::din_bms));

   Throttle::UdcLimitCommand(finalSpnt, Param::GetFloat(Param::udc));
   Throttle::IdcLimitCommand(finalSpnt, Param::GetFloat(Param::idc));
   Throttle::FrequencyLimitCommand(finalSpnt, Param::GetFloat(Param::fstat));

   if (Throttle::TemperatureDerate(Param::GetFloat(Param::tmphs), Param::GetFloat(Param::tmphsmax), finalSpnt))
   {
      DigIo::err_out.Set();
      ErrorMessage::Post(ERR_TMPHSMAX);
   }

   if (Throttle::TemperatureDerate(Param::GetFloat(Param::tmpm), Param::GetFloat(Param::tmpmmax), finalSpnt))
   {
      DigIo::err_out.Set();
      ErrorMessage::Post(ERR_TMPMMAX);
   }

   Param::SetFloat(Param::potnom, finalSpnt);

   if (finalSpnt < Param::GetFloat(Param::brklightout))
      DigIo::brk_out.Set();
   else
      DigIo::brk_out.Clear();

   if (determineDirection)
   {
      float rotorfreq = FP_TOFLOAT(Encoder::GetRotorFrequency());
      float brkrampstr = Param::GetFloat(Param::regenrampstr);

      if (rotorfreq < brkrampstr && finalSpnt < 0)
      {
         finalSpnt = (rotorfreq / brkrampstr) * finalSpnt;
      }

      if (finalSpnt < 0)
         finalSpnt *= Encoder::GetRotorDirection();
#if CONTROL == CTRL_FOC
      else //inconsistency here: in slip control negative always means regen
         finalSpnt *= Param::GetInt(Param::dir);
#endif // CONTROL
   }

   return finalSpnt;
}

void VehicleControl::SetContactorsOffState()
{
   if (PwmGeneration::Tripped())
   {
      switch (Param::GetInt(Param::tripmode))
      {
         default:
         case TRIP_ALLOFF:
            DigIo::dcsw_out.Clear();
            break;
         case TRIP_AUTORESUME:
         case TRIP_DCSWON:
            //do nothing
            break;
         case TRIP_PRECHARGEON:
            DigIo::dcsw_out.Clear();
            DigIo::prec_out.Set();
            break;
      }
   }
   else
   {
      DigIo::dcsw_out.Clear();
   }
}

void VehicleControl::GetDigInputs()
{
   int canio = Param::GetInt(Param::canio);

   canIoActive |= canio != 0;

   if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) >= CAN_TIMEOUT && canIoActive)
   {
      canio = 0;
      Param::SetInt(Param::canio, 0);
      ErrorMessage::Post(ERR_CANTIMEOUT);
   }

   Param::SetInt(Param::din_cruise, DigIo::cruise_in.Get() | ((canio & CAN_IO_CRUISE) != 0));
   Param::SetInt(Param::din_start, DigIo::start_in.Get() | ((canio & CAN_IO_START) != 0));
   Param::SetInt(Param::din_brake, DigIo::brake_in.Get() | ((canio & CAN_IO_BRAKE) != 0));
   Param::SetInt(Param::din_mprot, DigIo::mprot_in.Get());
   Param::SetInt(Param::din_forward, DigIo::fwd_in.Get() | ((canio & CAN_IO_FWD) != 0));
   Param::SetInt(Param::din_reverse, DigIo::rev_in.Get() | ((canio & CAN_IO_REV) != 0));
   Param::SetInt(Param::din_emcystop, DigIo::emcystop_in.Get());
   Param::SetInt(Param::din_bms, (canio & CAN_IO_BMS) != 0 || (hwRev == HW_TESLA ? false : DigIo::bms_in.Get()) );

   if (hwRev != HW_REV1 && hwRev != HW_BLUEPILL)
   {
      Param::SetInt(Param::din_ocur, DigIo::ocur_in.Get());
      Param::SetInt(Param::din_desat, DigIo::desat_in.Get());
   }
}

void VehicleControl::CalcAndOutputTemp()
{
   float pwmgain = Param::GetFloat(Param::pwmgain);
   int pwmofs = Param::GetInt(Param::pwmofs);
   int pwmfunc = Param::GetInt(Param::pwmfunc);
   int tmpout = 0;
   float tmphs = 0, tmpm = 0;

   GetTemps(tmphs, tmpm);

   temphsFiltered = IIRFILTERF(tmphs, temphsFiltered, 15);
   tempmFiltered = IIRFILTERF(tmpm, tempmFiltered, 18);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = tmpm * pwmgain + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = tmphs * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = Param::GetInt(Param::speed) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEEDFRQ:
         //Handled in 1ms task
         break;
   }

   tmpout = MIN(0xFFFF, MAX(0, tmpout));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, tmpout);

   Param::SetFloat(Param::tmphs, tmphs);
   Param::SetFloat(Param::tmpm, tmpm);
}

float VehicleControl::ProcessUdc()
{
   float udcfp;
   float udcmin = Param::GetFloat(Param::udcmin);
   float udcmax = Param::GetFloat(Param::udcmax);
   float udclim = Param::GetFloat(Param::udclim);
   float udcgain = Param::GetFloat(Param::udcgain);
   float udcsw = Param::GetFloat(Param::udcsw);
   int snshs = Param::GetInt(Param::snshs);
   int udcofs = Param::GetInt(Param::udcofs);
   int udcRaw;

   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
   //HW_REV1 had 3.9k resistors
   int uauxGain = hwRev == HW_REV1 ? 289 : 249;
   Param::SetFloat(Param::uaux, (float)AnaIn::uaux.Get() / uauxGain);

   //Yes heatsink temperature also selects external ADC as udc source
   if (snshs == TempMeas::TEMP_BMWI3HS)
   {
      BmwAdcAcquire();
      udcRaw = bmwAdcValues[ADC_CHAN_UDC];
   }
   else
   {
      udcRaw = AnaIn::udc.Get();
   }

   udcFiltered = IIRFILTER(udcFiltered, udcRaw, 2);
   udcfp = (udcFiltered - udcofs) / udcgain;

   //On i3 pin is used as SPI_MOSI
   if (snshs != TempMeas::TEMP_BMWI3HS)
   {
      if (udcfp < udcmin || udcfp > udcmax)
         DigIo::vtg_out.Set();
      else
         DigIo::vtg_out.Clear();
   }

   if (udcfp > udclim)
   {
      if (Encoder::GetSpeed() < 50) //If motor is stationary, over voltage comes from outside
      {
         DigIo::dcsw_out.Clear();  //In this case, open DC switch
         DigIo::prec_out.Clear();  //and precharge
      }

      Param::SetInt(Param::opmode, MOD_OFF);
      DigIo::err_out.Set();
      ErrorMessage::Post(ERR_OVERVOLTAGE);
   }

   if (udcfp < (udcsw / 2) && rtc_get_counter_val() > PRECHARGE_TIMEOUT && DigIo::prec_out.Get())
   {
      DigIo::err_out.Set();
      DigIo::prec_out.Clear();
      ErrorMessage::Post(ERR_PRECHARGE);
   }

   #if CONTROL == CTRL_SINE
   {
      float udcnom = Param::GetFloat(Param::udcnom);
      float boost = Param::GetFloat(Param::boost);
      float fweak;

      if (Param::GetInt(Param::potnom) > 35)
      {
         fweak = MAP(Param::GetFloat(Param::potnom), 36, 100, (Param::GetFloat(Param::fweakstrt)), (Param::GetFloat(Param::fweak)));
      }
      else
      {
         fweak = Param::GetFloat(Param::fweakstrt);
      }

      if (udcnom > 0)
      {
         float udcdiff = udcfp - udcnom;
         float factor = 1.0 + udcdiff / udcnom;
         //increase fweak on voltage above nominal
         fweak = fweak * factor;
         //decrease boost on voltage below nominal
         boost = boost / factor;
      }

      Param::SetFloat(Param::fweakcalc, fweak);
      Param::SetFloat(Param::boostcalc, boost);
      MotorVoltage::SetWeakeningFrq(fweak);
      MotorVoltage::SetBoost(boost);
   }
   #endif // CONTROL

   Param::SetFloat(Param::udc, udcfp);

   return udcfp;
}

void VehicleControl::GetTemps(float& tmphs, float &tmpm)
{
   if (hwRev == HW_TESLA)
   {
      static float hsTemps[3];
      static float mTemps[2];
      static bool isLdu = false;

      int input = DigIo::temp0_out.Get() + 2 * DigIo::temp1_out.Get();
      int tmphsi = AnaIn::tmphs.Get();
      int tmpmi = AnaIn::tmpm.Get();

      tmphs = Param::GetFloat(Param::tmphs); //default to last value;
      tmpm = Param::GetFloat(Param::tmpm);

      switch (input)
      {
         case 0:
            //Mux1.0 reads fluid temperature on both drive units which we ignore
            //Mux2.0 reads inverter temperature A on both drive units
            hsTemps[0] = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            DigIo::temp0_out.Set();
            DigIo::temp1_out.Clear(); //Switch mux for next round
            break;
         case 1:
            //Mux1.1 reads stator temperature and SDU and fluid temperature on LDU which we ignore
            //Mux2.1 reads inverter temperature B on both drive units
            hsTemps[1] = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            if (!isLdu)
               mTemps[0] = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
            DigIo::temp0_out.Clear();
            DigIo::temp1_out.Set(); //Switch mux for next round
            break;
         case 2:
            //Mux1.2 reads case temperature on SDU and stator temperature 1 on LDU
            //Mux2.2 reads inverter temperature C on both drive units
            hsTemps[2] = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            if (isLdu)
               mTemps[0] = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
            else
               mTemps[1] = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_10K);
            //Done reading inverter temps, update to maximum
            tmphs = MAX(hsTemps[0], MAX(hsTemps[1], hsTemps[2]));
            DigIo::temp0_out.Set();
            DigIo::temp1_out.Set(); //Switch mux for next round
            break;
         case 3:
            //Mux 1.3 is grounded on SDU and reads stator temperature 2 on LDU
            //Mux 2.3 is grounded on both drive units
            isLdu = tmpmi > 50;  //Tied to GND on SDU
            if (isLdu)
               mTemps[1] = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
            //Now update to maximum temperaure
            tmpm = MAX(mTemps[0], mTemps[1]);
            DigIo::temp0_out.Clear();
            DigIo::temp1_out.Clear(); //Switch mux for next round
            break;
      }
   }
   else
   {
      TempMeas::Sensors snshs = (TempMeas::Sensors)Param::GetInt(Param::snshs);
      TempMeas::Sensors snsm = (TempMeas::Sensors)Param::GetInt(Param::snsm);

      int tmphsi = AnaIn::tmphs.Get();
      int tmpmi = AnaIn::tmpm.Get();

      tmpm = TempMeas::Lookup(tmpmi, snsm);

      if (hwRev == HW_PRIUS)
      {
         static float priusTempCoeff = 18.62;

         //We need to install a 10k pull-down resistor to be able
         //to measure temps lower than 56°C. If this resistor
         //is not installed, we will see readings above 2.5V
         //and we double the coefficient to at least get a valid
         //reading.
         if (priusTempCoeff < 20 && tmphsi > 3200)
         {
            priusTempCoeff *= 2;
         }

         tmphs = 166.66f - tmphsi / priusTempCoeff;
      }
      else if (snshs == TempMeas::TEMP_BMWI3HS)
      {
         //For the next line to work, BmwAdcGet() must be called regularly
         //Which currently happens in ProcessUdc()
         tmphsi = MIN(bmwAdcValues[0], MIN(bmwAdcValues[1], bmwAdcValues[2]));
         tmphs = TempMeas::Lookup(tmphsi, snshs);
      }
      else
      {
         tmphs = TempMeas::Lookup(tmphsi, snshs);
      }
   }
}

float VehicleControl::GetUserThrottleCommand()
{
   float potnom1, potnom2;
   int potval, pot2val;
   bool brake = Param::GetBool(Param::din_brake);
   int potmode = Param::GetInt(Param::potmode);

   if ((potmode & POTMODE_CAN) > 0)
   {
      //500ms timeout
      if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) < CAN_TIMEOUT)
      {
         potval = Param::GetInt(Param::pot);
         pot2val = Param::GetInt(Param::pot2);
      }
      else
      {
         DigIo::err_out.Set();
         PostErrorIfRunning(ERR_CANTIMEOUT);
         return 0;
      }
   }
   else
   {
      potval = AnaIn::throttle1.Get();
      pot2val = AnaIn::throttle2.Get();
      Param::SetInt(Param::pot, potval);
      Param::SetInt(Param::pot2, pot2val);
   }

   bool inRange1 = Throttle::CheckAndLimitRange(&potval, 0);
   bool inRange2 = Throttle::CheckAndLimitRange(&pot2val, 1);

   if (!inRange1)
   {
      DigIo::err_out.Set();
      PostErrorIfRunning(ERR_THROTTLE1);
   }

   potnom1 = Throttle::DigitsToPercent(potval, 0);
   potnom2 = Throttle::DigitsToPercent(pot2val, 1);

   if ((potmode & POTMODE_BIDIR) > 0)
   {
      if (!inRange1) return 0;
      float bidirThrot = Throttle::CalcThrottleBiDir(potnom1, brake);

      if (bidirThrot == 0)
      {
         bidirThrot = Throttle::brkmax;
         Param::SetInt(Param::dir, Encoder::GetRotorDirection());
      }
      else if (bidirThrot < 0)
      {
         bidirThrot = -bidirThrot;
         Param::SetInt(Param::dir, -1);
      }
      else //bidirThrot > 0
      {
         Param::SetInt(Param::dir, 1);
      }
      return bidirThrot;
   }
   else if ((potmode & POTMODE_DUALCHANNEL) > 0)
   {
      if (inRange1 && inRange2)
      {
         //Both channels good, take lower one
         potnom1 = MIN(potnom1, potnom2);
      }
      else if (inRange1)
      {
         //channel 1 good, channel 2 not good, show warning, use channel 1 value as is
         DigIo::err_out.Set();
         PostErrorIfRunning(ERR_THROTTLE2);
      }
      else if (inRange2)
      {
         //channel 1 not good, channel 2 good, use channel 2 value
         potnom1 = potnom2;
      }
      else
      {
         //Both pots out of range, inhibit movement
         PostErrorIfRunning(ERR_THROTTLE2);
         return 0;
      }

      potnom2 = 100.0f; //No regen attenuation
   }
   else if (!inRange1)
   {
      //Only one channel and it is out of range
      return 0;
   }

   if (Param::GetInt(Param::dir) == 0)
      return 0;

   return Throttle::CalcThrottle(potnom1, potnom2, brake);
}

bool VehicleControl::GetCruiseCreepCommand(float& finalSpnt, float throtSpnt)
{
   static bool runHillHold = false;
   bool autoDertermineDirection = true;
   bool brake = Param::GetBool(Param::din_brake);
   int idlemode = Param::GetInt(Param::idlemode);
   uint32_t speed = Encoder::GetSpeed();
   float idleSpnt = Throttle::CalcIdleSpeed(speed);
   float cruiseSpnt = Throttle::CalcCruiseSpeed(speed);

   finalSpnt = throtSpnt; //assume no regulation

   if (idlemode == IDLE_MODE_ALWAYS ||
      (idlemode == IDLE_MODE_NOBRAKE && !brake) ||
      (idlemode == IDLE_MODE_CRUISE && !brake && Param::GetBool(Param::din_cruise)))
   {
      finalSpnt = MAX(throtSpnt, idleSpnt);
   }
   else if (idlemode == IDLE_MODE_HILLHOLD)
   {
      if (brake && speed == 0)
      {
         Encoder::ResetDistance();
         runHillHold = true;
      }
      else if (runHillHold)
      {
         runHillHold = Throttle::HoldPosition(Encoder::GetDistance(), finalSpnt);
         autoDertermineDirection = !runHillHold;
      }

      if (throtSpnt > 0 && throtSpnt > finalSpnt)
      {
         runHillHold = false;
      }
   }

   if (Throttle::cruiseSpeed > 0 && Throttle::cruiseSpeed > Throttle::idleSpeed)
   {
      if (throtSpnt <= 0)
         finalSpnt = cruiseSpnt;
      else if (throtSpnt > 0)
         finalSpnt = MAX(cruiseSpnt, throtSpnt);
   }

   return autoDertermineDirection;
}

void VehicleControl::BmwAdcAcquire()
{
   const uint16_t adcGetManual = 1 << 12, adcSetDio = 1 << 11, adcVrefDual = 1 << 6;
   const uint32_t maxChan = sizeof(bmwAdcValues) / sizeof(bmwAdcValues[0]);

   if (!spiEnabled)
   {
      spi_setup();
      //Brake pin is used as SPI_MISO
      DigIo::brk_out.Configure(GPIOC, GPIO5, PinMode::INPUT_FLT);
      DigIo::spi_cs_out.Set();
      spiEnabled = true;
   }

   DigIo::spi_cs_out.Clear();
   uint16_t data = spi_xfer(SPI1, adcGetManual | adcSetDio | adcVrefDual | (bmwAdcNextChan << 7));
   DigIo::spi_cs_out.Set();
   uint16_t readChan = data >> 12;
   bmwAdcNextChan = (bmwAdcNextChan == maxChan) ? 0 : bmwAdcNextChan + 1;

   if (readChan < maxChan)
      bmwAdcValues[readChan] = data & 0xFFF;
}
