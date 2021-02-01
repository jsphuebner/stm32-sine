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

#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

Can* VehicleControl::can;
bool VehicleControl::lastCruiseSwitchState = false;
bool VehicleControl::canIoActive = false;
int VehicleControl::temphsFiltered = 0;
int VehicleControl::tempmFiltered = 0;
int VehicleControl::udcFiltered = 0;

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

s32fp VehicleControl::ProcessThrottle()
{
   s32fp throtSpnt, finalSpnt;

   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::throtramprpm))
      Throttle::throttleRamp = Param::Get(Param::throtramp);
   else
      Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;

   throtSpnt = GetUserThrottleCommand();
   GetCruiseCreepCommand(finalSpnt, throtSpnt);
   finalSpnt = Throttle::RampThrottle(finalSpnt);

   if (hwRev != HW_TESLA)
      Throttle::BmsLimitCommand(finalSpnt, Param::GetBool(Param::din_bms));

   Throttle::UdcLimitCommand(finalSpnt, Param::Get(Param::udc));
   Throttle::IdcLimitCommand(finalSpnt, Param::Get(Param::idc));
   Throttle::FrequencyLimitCommand(finalSpnt, Param::Get(Param::fstat));

   if (Throttle::TemperatureDerate(Param::Get(Param::tmphs), Param::Get(Param::tmphsmax), finalSpnt))
   {
      DigIo::err_out.Set();
      ErrorMessage::Post(ERR_TMPHSMAX);
   }

   if (Throttle::TemperatureDerate(Param::Get(Param::tmpm), Param::Get(Param::tmpmmax), finalSpnt))
   {
      DigIo::err_out.Set();
      ErrorMessage::Post(ERR_TMPMMAX);
   }

   Param::SetFlt(Param::potnom, finalSpnt);

   if (finalSpnt < Param::Get(Param::brkout))
      DigIo::brk_out.Set();
   else
      DigIo::brk_out.Clear();

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
   s32fp pwmgain = Param::Get(Param::pwmgain);
   int pwmofs = Param::GetInt(Param::pwmofs);
   int pwmfunc = Param::GetInt(Param::pwmfunc);
   int tmpout = 0;
   s32fp tmphs = 0, tmpm = 0;

   GetTemps(tmphs, tmpm);

   temphsFiltered = IIRFILTER(tmphs, temphsFiltered, 15);
   tempmFiltered = IIRFILTER(tmpm, tempmFiltered, 18);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = FP_TOINT(FP_MUL(tmpm, pwmgain)) + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = FP_TOINT(FP_MUL(tmphs, pwmgain)) + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = FP_TOINT(FP_MUL(Param::Get(Param::speed), pwmgain)) + pwmofs;
         break;
      case PWM_FUNC_SPEEDFRQ:
         //Handled in 1ms task
         break;
   }

   tmpout = MIN(0xFFFF, MAX(0, tmpout));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, tmpout);

   Param::SetFlt(Param::tmphs, tmphs);
   Param::SetFlt(Param::tmpm, tmpm);
}

s32fp VehicleControl::ProcessUdc()
{
   s32fp udcfp;
   s32fp udcmin = Param::Get(Param::udcmin);
   s32fp udcmax = Param::Get(Param::udcmax);
   s32fp udclim = Param::Get(Param::udclim);
   s32fp udcgain = Param::Get(Param::udcgain);
   s32fp udcsw = Param::Get(Param::udcsw);
   int udcofs = Param::GetInt(Param::udcofs);

   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
   //HW_REV1 had 3.9k resistors
   int uauxGain = hwRev == HW_REV1 ? 289 : 249;
   Param::SetFlt(Param::uaux, FP_DIV(AnaIn::uaux.Get(), uauxGain));
   udcFiltered = IIRFILTER(udcFiltered, AnaIn::udc.Get(), 2);
   udcfp = FP_DIV(FP_FROMINT(udcFiltered - udcofs), udcgain);

   if (hwRev != HW_TESLAM3)
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
      s32fp udcnom = Param::Get(Param::udcnom);
      s32fp fweak = Param::Get(Param::fweak);
      s32fp boost = Param::Get(Param::boost);

      if (udcnom > 0)
      {
         s32fp udcdiff = udcfp - udcnom;
         s32fp factor = FP_FROMINT(1) + FP_DIV(udcdiff, udcnom);
         //increase fweak on voltage above nominal
         fweak = FP_MUL(fweak, factor);
         //decrease boost on voltage below nominal
         boost = FP_DIV(boost, factor);
      }

      Param::SetFlt(Param::fweakcalc, fweak);
      Param::SetFlt(Param::boostcalc, boost);
      MotorVoltage::SetWeakeningFrq(fweak);
      MotorVoltage::SetBoost(FP_TOINT(boost));
   }
   #endif // CONTROL

   Param::SetFlt(Param::udc, udcfp);

   return udcfp;
}

void VehicleControl::GetTemps(s32fp& tmphs, s32fp &tmpm)
{
   if (hwRev == HW_TESLA)
   {
      static s32fp hsTemps[3];
      static s32fp mTemps[2];
      static bool isLdu = false;

      int input = DigIo::temp0_out.Get() + 2 * DigIo::temp1_out.Get();
      int tmphsi = AnaIn::tmphs.Get();
      int tmpmi = AnaIn::tmpm.Get();

      tmphs = Param::Get(Param::tmphs); //default to last value;
      tmpm = Param::Get(Param::tmpm);

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
         static s32fp priusTempCoeff = FP_FROMFLT(18.62);

         //We need to install a 10k pull-down resistor to be able
         //to measure temps lower than 56°C. If this resistor
         //is not installed, we will see readings above 2.5V
         //and we double the coefficient to at least get a valid
         //reading.
         if (priusTempCoeff < FP_FROMFLT(20) && tmphsi > 3200)
         {
            priusTempCoeff *= 2;
         }

         tmphs = FP_FROMFLT(166.66) - FP_DIV(FP_FROMINT(tmphsi), priusTempCoeff);
      }
      else
      {
         tmphs = TempMeas::Lookup(tmphsi, snshs);
      }
   }
}

s32fp VehicleControl::GetUserThrottleCommand()
{
   s32fp potnom1, potnom2;
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

   if ((potmode & POTMODE_DUALCHANNEL) > 0)
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

      potnom2 = FP_FROMINT(100); //No regen attenuation
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

void VehicleControl::GetCruiseCreepCommand(s32fp& finalSpnt, s32fp throtSpnt)
{
   bool brake = Param::GetBool(Param::din_brake);
   s32fp idleSpnt = Throttle::CalcIdleSpeed(Encoder::GetSpeed());
   s32fp cruiseSpnt = Throttle::CalcCruiseSpeed(Encoder::GetSpeed());

   finalSpnt = throtSpnt; //assume no regulation

   if (Param::GetInt(Param::idlemode) == IDLE_MODE_ALWAYS ||
      (Param::GetInt(Param::idlemode) == IDLE_MODE_NOBRAKE && !brake) ||
      (Param::GetInt(Param::idlemode) == IDLE_MODE_CRUISE && !brake && Param::GetBool(Param::din_cruise)))
   {
      finalSpnt = MAX(throtSpnt, idleSpnt);
   }

   if (Throttle::cruiseSpeed > 0 && Throttle::cruiseSpeed > Throttle::idleSpeed)
   {
      if (throtSpnt <= 0)
         finalSpnt = cruiseSpnt;
      else if (throtSpnt > 0)
         finalSpnt = MAX(cruiseSpnt, throtSpnt);
   }
}
