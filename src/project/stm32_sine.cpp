/*
 * This file is part of the tumanako_vc project.
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "sine_core.h"
#include "fu.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"
#include "inc_encoder.h"
#include "foc.h"
#include "throttle.h"
#include "my_math.h"
#include "errormessage.h"
#include "pwmgeneration.h"
#include "printf.h"
#include "stm32scheduler.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

HWREV hwRev; //Hardware variant of board we are running on

//Precise control of executing the boost controller
static bool runChargeControl = false;
static Stm32Scheduler* scheduler;

static void CruiseControl()
{
   static bool lastState = false;

   //Always disable cruise control when brake pedal is pressed
   if (Param::GetBool(Param::din_brake))
   {
      Throttle::cruiseSpeed = -1;
   }
   else
   {
      if (Param::GetInt(Param::cruisemode) == BUTTON)
      {
         //Enable/update cruise control when button is pressed
         if (Param::GetBool(Param::din_cruise))
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }
      }
      else //if cruiseMode == SWITCH
      {
         //Enable/update cruise control when switch is toggled on
         if (Param::GetBool(Param::din_cruise) && !lastState)
         {
            Throttle::cruiseSpeed = Encoder::GetSpeed();
         }

         //Disable cruise control when switch is off
         if (!Param::GetBool(Param::din_cruise))
         {
            Throttle::cruiseSpeed = -1;
         }
      }
   }

   lastState = Param::GetBool(Param::din_cruise);
}

static void CalcFancyValues()
{
   //const s32fp twoPi = FP_FROMFLT(2*3.141593);
   s32fp amp = Param::Get(Param::amp);
   //s32fp speed = Param::Get(Param::speed);
   s32fp il1 = Param::Get(Param::il1);
   s32fp il2 = Param::Get(Param::il2);
   s32fp udc = Param::Get(Param::udc);
   s32fp fac = FP_DIV(amp, FP_FROMINT(SineCore::MAXAMP));
   s32fp uac = FP_MUL(fac, FP_MUL(udc, FP_FROMFLT(0.7071)));
   s32fp idc; //, is, p, q, s, pf, t;

   FOC::ParkClarke(il1, il2, PwmGeneration::GetAngle());
   //is = fp_sqrt(FP_MUL(FOC::id,FOC::id)+FP_MUL(FOC::iq,FOC::iq));
   /*s = FP_MUL(fac, FP_MUL(uac, is) / 1000);
   p = FP_MUL(fac, FP_MUL(uac, FOC::id));
   t = FP_DIV(p, FP_MUL(twoPi, speed / 60));
   p/= 1000;
   q = FP_MUL(fac, FP_MUL(uac, FOC::iq) / 1000);
   pf = MIN(FP_FROMINT(1), MAX(0, FP_DIV(FOC::id, is)));*/

   if (Param::GetInt(Param::opmode) < MOD_BOOST)
   {
      idc = FP_MUL(FP_MUL(fac, FOC::id), FP_FROMFLT(0.7071));
      Param::SetFlt(Param::idc, idc);
   }

   /*Param::SetFlt(Param::id, FOC::id);
   Param::SetFlt(Param::iq, FOC::iq);*/
   Param::SetFlt(Param::uac, uac);
   /*Param::SetFlt(Param::pf, pf);
   Param::SetFlt(Param::p, p);
   Param::SetFlt(Param::q, q);
   Param::SetFlt(Param::s, s);
   Param::SetFlt(Param::t, t);*/
}

static void Ms100Task(void)
{
   int dir = Param::GetInt(Param::dir);
   int newDir = 0;

   DigIo::Toggle(Pin::led_out);
   ErrorMessage::PrintNewErrors();
   iwdg_reset();
   s32fp cpuLoad = FP_FROMINT(PwmGeneration::GetCpuLoad() + scheduler->GetCpuLoad());
   Param::SetFlt(Param::cpuload, cpuLoad / 10);

   if (Param::GetBool(Param::din_forward))
      newDir = 1;
   else if (Param::GetBool(Param::din_reverse))
      newDir = -1;

   /* Only change direction when below certain motor speed */
   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::dirchrpm))
      dir = newDir;

   /* Current direction doesn't match selected direction -> neutral */
   if (dir != newDir)
      dir = 0;

   /* neither forward nor reverse or both forward and reverse -> neutral */
   if (!(Param::GetBool(Param::din_forward) ^ Param::GetBool(Param::din_reverse)))
      dir = 0;

   FOC::SetDirection(dir);

   if (hwRev == HW_REV1)
   {
      //If break pin is high and both mprot and emcystop are high than it must be over current
      if (DigIo::Get(Pin::emcystop_in) && DigIo::Get(Pin::mprot_in) && DigIo::Get(Pin::bk_in))
      {
         Param::SetInt(Param::din_ocur, 1);
      }
      else
      {
         Param::SetInt(Param::din_ocur, 0);
      }
   }

   CruiseControl();
   CalcFancyValues();

   Param::SetInt(Param::dir, dir);
   Param::SetInt(Param::turns, Encoder::GetFullTurns());

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      Can::SendAll();
}

static void GetDigInputs()
{
   static bool canIoActive = false;
   int canio = Param::GetInt(Param::canio);

   canIoActive |= canio != 0;

   if ((rtc_get_counter_val() - Can::GetLastRxTimestamp()) >= CAN_TIMEOUT && canIoActive)
   {
      canio = 0;
      Param::SetInt(Param::canio, 0);
      ErrorMessage::Post(ERR_CANTIMEOUT);
   }

   Param::SetInt(Param::din_cruise, DigIo::Get(Pin::cruise_in) | ((canio & CAN_IO_CRUISE) != 0));
   Param::SetInt(Param::din_start, DigIo::Get(Pin::start_in) | ((canio & CAN_IO_START) != 0));
   Param::SetInt(Param::din_brake, DigIo::Get(Pin::brake_in) | ((canio & CAN_IO_BRAKE) != 0));
   Param::SetInt(Param::din_mprot, DigIo::Get(Pin::mprot_in));
   Param::SetInt(Param::din_forward, DigIo::Get(Pin::fwd_in) | ((canio & CAN_IO_FWD) != 0));
   Param::SetInt(Param::din_reverse, DigIo::Get(Pin::rev_in) | ((canio & CAN_IO_REV) != 0));
   Param::SetInt(Param::din_emcystop, DigIo::Get(Pin::emcystop_in));
   Param::SetInt(Param::din_bms, DigIo::Get(Pin::bms_in) | ((canio & CAN_IO_BMS) != 0));

   if (hwRev != HW_REV1)
   {
      Param::SetInt(Param::din_ocur, DigIo::Get(Pin::ocur_in));
      Param::SetInt(Param::din_desat, DigIo::Get(Pin::desat_in));
   }
}

static void CalcAmpAndSlip()
{
   s32fp fslipmin = Param::Get(Param::fslipmin);
   s32fp ampmin = Param::Get(Param::ampmin);
   s32fp potnom = Param::Get(Param::potnom);
   s32fp slipstart = Param::Get(Param::slipstart);
   s32fp ampnom;
   s32fp fslipspnt;

   if (potnom >= 0)
   {
      /* In sync mode throttle only commands amplitude. Above back-EMF is acceleration, below is regen */
      if (Encoder::IsSyncMode())
         ampnom = ampmin + FP_DIV(FP_MUL((FP_FROMINT(100) - ampmin), potnom), FP_FROMINT(100));
      else /* In async mode first X% throttle commands amplitude, X-100% raises slip */
         ampnom = 100 * FP_DIV(potnom, slipstart);
         //ampmin + (potnom - ampmin) * FP_DIV(FP_FROMINT(100), slipstart);

      if (potnom >= slipstart)
      {
         s32fp fslipmax = Param::Get(Param::fslipmax);
         s32fp fpconst =  Param::Get(Param::fpconst);
         s32fp fstat = Param::Get(Param::fstat);
         s32fp fweak = Param::Get(Param::fweak);
         s32fp fslipdiff = fslipmax - fslipmin;
         //Derate the slip frequency above fpconst and uprate above fweak
         s32fp ffac = 2*(fstat > fweak?fpconst + fstat - 2 * fweak:fpconst - fstat);
         s32fp fslipdrt = FP_MUL(FP_DIV(ffac, fweak), fslipdiff) + fslipmax;

         fslipspnt = fslipmin + (FP_MUL(fslipdiff, (potnom - slipstart)) / (100 - FP_TOINT(slipstart)));
         fslipspnt = MIN(fslipspnt, fslipdrt);
      }
      else
      {
         fslipspnt = fslipmin;
      }
      DigIo::Clear(Pin::brk_out);
   }
   else
   {
      u32fp brkrampstr = (u32fp)Param::Get(Param::brkrampstr);

      if (Encoder::IsSyncMode())
         ampnom = ampmin - FP_DIV(FP_MUL(ampmin, potnom), FP_FROMINT(100));
      else
         ampnom = -potnom;

      fslipspnt = -fslipmin;
      if (Encoder::GetRotorFrequency() < brkrampstr)
      {
         ampnom = FP_TOINT(FP_DIV(Encoder::GetRotorFrequency(), brkrampstr) * ampnom);
      }
      //This works because ampnom = -potnom
      if (ampnom >= -Param::Get(Param::brkout))
         DigIo::Set(Pin::brk_out);
      else
         DigIo::Clear(Pin::brk_out);
   }

   ampnom = MIN(ampnom, FP_FROMINT(100));
   //anticipate sudden changes by filtering
   s32fp ampnomLast = Param::Get(Param::ampnom);
   s32fp fslipLast = Param::Get(Param::fslipspnt);
   ampnomLast = IIRFILTER(ampnomLast, ampnom, 4);
   fslipLast = IIRFILTER(fslipLast, fslipspnt, 4);
   Param::Set(Param::ampnom, ampnomLast);
   Param::Set(Param::fslipspnt, fslipLast);
}

static void GetTemps(s32fp& tmphs, s32fp &tmpm)
{
   if (hwRev == HW_TESLA)
   {
      static int tmphsMax = 0, tmpmMax = 0;
      static int input = 0;

      int tmphsi = AnaIn::Get(AnaIn::tmphs);
      int tmpmi = AnaIn::Get(AnaIn::tmpm);

      switch (input)
      {
         case 0:
            DigIo::Clear(Pin::temp0_out);
            DigIo::Clear(Pin::temp1_out);
            input = 1;
            //Handle mux inputs 11
            tmphs = 0; //not connected
            tmpm = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
            break;
         case 1:
            DigIo::Set(Pin::temp0_out);
            DigIo::Clear(Pin::temp1_out);
            tmphsMax = 0;
            tmpmMax = 0;
            input = 2;
            //Handle mux inputs 00
            tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            tmpm = 0; //don't know yet
            break;
         case 2:
            DigIo::Clear(Pin::temp0_out);
            DigIo::Set(Pin::temp1_out);
            input = 3;
            //Handle mux inputs 01
            tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            tmpm = 0; //don't know yet
            break;
         case 3:
            DigIo::Set(Pin::temp0_out);
            DigIo::Set(Pin::temp1_out);
            input = 0;
            //Handle mux inputs 10
            tmphs = TempMeas::Lookup(tmphsi, TempMeas::TEMP_TESLA_52K);
            tmpm = TempMeas::Lookup(tmpmi, TempMeas::TEMP_TESLA_100K);
            break;
      }

      tmphs = tmphsMax = MAX(tmphsMax, tmphs);
      tmpm = tmpmMax = MAX(tmpmMax, tmpm);
   }
   else
   {
      TempMeas::Sensors snshs = (TempMeas::Sensors)Param::GetInt(Param::snshs);
      TempMeas::Sensors snsm = (TempMeas::Sensors)Param::GetInt(Param::snsm);

      int tmphsi = AnaIn::Get(AnaIn::tmphs);
      int tmpmi = AnaIn::Get(AnaIn::tmpm);

      tmpm = TempMeas::Lookup(tmpmi, snsm);
      tmphs = TempMeas::Lookup(tmphsi, snshs);
   }
}

static void CalcAndOutputTemp()
{
   static int temphsFlt = 0;
   static int tempmFlt = 0;
   int pwmgain = Param::GetInt(Param::pwmgain);
   int pwmofs = Param::GetInt(Param::pwmofs);
   int pwmfunc = Param::GetInt(Param::pwmfunc);
   int tmpout = 0;
   s32fp tmphs = 0, tmpm = 0;

   GetTemps(tmphs, tmpm);

   temphsFlt = IIRFILTER(tmphs, temphsFlt, 15);
   tempmFlt = IIRFILTER(tmpm, tempmFlt, 18);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = FP_TOINT(tmpm) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = FP_TOINT(tmphs) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = Param::Get(Param::speed) * pwmgain + pwmofs;
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

static s32fp ProcessUdc()
{
   static int32_t udc = 0;
   s32fp udcfp;
   s32fp udcmin = Param::Get(Param::udcmin);
   s32fp udcmax = Param::Get(Param::udcmax);
   s32fp udclim = Param::Get(Param::udclim);
   s32fp udcgain = Param::Get(Param::udcgain);
   s32fp udcnom = Param::Get(Param::udcnom);
   s32fp fweak = Param::Get(Param::fweak);
   s32fp boost = Param::Get(Param::boost);
   s32fp udcsw = Param::Get(Param::udcsw);
   int udcofs = Param::GetInt(Param::udcofs);

   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
   Param::SetFlt(Param::uaux, FP_DIV(AnaIn::Get(AnaIn::uaux), 249));
   udc = IIRFILTER(udc, AnaIn::Get(AnaIn::udc), 2);
   udcfp = FP_DIV(FP_FROMINT(udc - udcofs), udcgain);

   if (udcfp < udcmin || udcfp > udcmax)
      DigIo::Set(Pin::vtg_out);
   else
      DigIo::Clear(Pin::vtg_out);

   if (udcfp > udclim)
   {
      if (Encoder::GetSpeed() < 50) //If motor is stationary, over voltage comes from outside
      {
         DigIo::Clear(Pin::dcsw_out);  //In this case, open DC switch
         DigIo::Clear(Pin::prec_out);  //and precharge
      }

      Param::SetInt(Param::opmode, MOD_OFF);
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_OVERVOLTAGE);
   }

   if (udcfp < (udcsw / 2) && rtc_get_counter_val() > PRECHARGE_TIMEOUT)
   {
      DigIo::Set(Pin::err_out);
      DigIo::Clear(Pin::prec_out);
      ErrorMessage::Post(ERR_PRECHARGE);
   }

   if (udcnom > 0)
   {
      s32fp udcdiff = udcfp - udcnom;
      s32fp factor = FP_FROMINT(1) + FP_DIV(udcdiff, udcnom);
      //increase fweak on voltage above nominal
      fweak = FP_MUL(fweak, factor);
      //decrease boost on voltage below nominal
      boost = FP_DIV(boost, factor);
   }

   Param::SetFlt(Param::udc, udcfp);
   Param::SetFlt(Param::fweakcalc, fweak);
   Param::SetFlt(Param::boostcalc, boost);
   MotorVoltage::SetWeakeningFrq(fweak);
   MotorVoltage::SetBoost(FP_TOINT(boost));

   return udcfp;
}

static int GetUserThrottleCommand()
{
   int potval, pot2val;
   bool brake = Param::GetBool(Param::din_brake);
   int potmode = Param::GetInt(Param::potmode);

   if (potmode == POTMODE_CAN)
   {
      //500ms timeout
      if ((rtc_get_counter_val() - Can::GetLastRxTimestamp()) < CAN_TIMEOUT)
      {
         potval = Param::GetInt(Param::pot);
         pot2val = Param::GetInt(Param::pot2);
      }
      else
      {
         DigIo::Set(Pin::err_out);
         ErrorMessage::Post(ERR_CANTIMEOUT);
         return 0;
      }
   }
   else
   {
      potval = AnaIn::Get(AnaIn::throttle1);
      pot2val = AnaIn::Get(AnaIn::throttle2);
      Param::SetInt(Param::pot, potval);
      Param::SetInt(Param::pot2, pot2val);
   }

   /* Error light on implausible value */
   if (!Throttle::CheckAndLimitRange(&potval, 0))
   {
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_THROTTLE1);
      return 0;
   }

   bool throt2Res = Throttle::CheckAndLimitRange(&pot2val, 1);

   if (potmode == POTMODE_DUALCHANNEL)
   {
      if (!Throttle::CheckDualThrottle(&potval, pot2val) || !throt2Res)
      {
         DigIo::Set(Pin::err_out);
         ErrorMessage::Post(ERR_THROTTLE1);
         Param::SetInt(Param::potnom, 0);
         return 0;
      }
      pot2val = Throttle::potmax[1]; //make sure we don't attenuate regen
   }

   if (Param::GetInt(Param::dir) == 0)
      return 0;

   return Throttle::CalcThrottle(potval, pot2val, brake);
}

static void GetCruiseCreepCommand(int& finalSpnt, int throtSpnt)
{
   bool brake = Param::GetBool(Param::din_brake);
   int cruiseSpnt = Throttle::CalcCruiseSpeed(Encoder::GetSpeed());
   int idleSpnt = Throttle::CalcIdleSpeed(Encoder::GetSpeed());

   if (Param::GetInt(Param::idlemode) == IDLE_MODE_ALWAYS ||
      (Param::GetInt(Param::idlemode) == IDLE_MODE_NOBRAKE && !brake) ||
      (Param::GetInt(Param::idlemode) == IDLE_MODE_CRUISE && !brake && Param::GetBool(Param::din_cruise)))
   {
      finalSpnt = MAX(throtSpnt, idleSpnt);
   }
   else
   {
      finalSpnt = throtSpnt;
   }

   if (Throttle::cruiseSpeed > 0 && Throttle::cruiseSpeed > Throttle::idleSpeed)
   {
      if (throtSpnt < 0)
         finalSpnt = cruiseSpnt;
      else if (throtSpnt > 0)
         finalSpnt = MAX(cruiseSpnt, throtSpnt);
   }
}

static void BmsLimitCommand(int& finalSpnt)
{
   if (hwRev != HW_TESLA && Param::GetInt(Param::din_bms))
   {
      if (finalSpnt >= 0)
         finalSpnt = (finalSpnt * Param::GetInt(Param::bmslimhigh)) / 100;
      else
         finalSpnt = -(finalSpnt * Param::GetInt(Param::bmslimlow)) / 100;
   }
}

static void UdcLimitCommand(int& finalSpnt)
{
   s32fp udc = Param::Get(Param::udc);

   if (udc < Param::Get(Param::udcmin) && finalSpnt >= 0)
   {
      s32fp udcErr = Param::Get(Param::udcmin) - udc;
      s32fp res = finalSpnt - FP_TOINT(udcErr * 10);
      finalSpnt = MAX(0, res);
   }
   else if (udc > Param::Get(Param::udcmax) && finalSpnt < 0)
   {
      s32fp udcErr = udc - Param::Get(Param::udcmax);
      s32fp res = finalSpnt + FP_TOINT(udcErr * 10);
      finalSpnt = MIN(0, res);
   }
}

static void ProcessThrottle()
{
   int throtSpnt, finalSpnt;

   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::throtramprpm))
      Throttle::throttleRamp = Param::GetInt(Param::throtramp);
   else
      Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;

   throtSpnt = GetUserThrottleCommand();
   GetCruiseCreepCommand(finalSpnt, throtSpnt);
   BmsLimitCommand(finalSpnt);
   UdcLimitCommand(finalSpnt);

   if (Throttle::TemperatureDerate(Param::Get(Param::tmphs), finalSpnt))
   {
      DigIo::Set(Pin::err_out);
      ErrorMessage::Post(ERR_TMPHSMAX);
   }

   Param::SetInt(Param::potnom, finalSpnt);
}

static void SetContactorsOffState()
{
   if (PwmGeneration::Tripped())
   {
      switch (Param::GetInt(Param::tripmode))
      {
         default:
         case TRIP_ALLOFF:
            DigIo::Clear(Pin::dcsw_out);
            break;
         case TRIP_DCSWON:
            //do nothing
            break;
         case TRIP_PRECHARGEON:
            DigIo::Clear(Pin::dcsw_out);
            DigIo::Set(Pin::prec_out);
            break;
      }
   }
   else
   {
      DigIo::Clear(Pin::dcsw_out);
   }
}

//Normal run takes 70µs -> 0.7% cpu load (last measured version 3.5)
static void Ms10Task(void)
{
   static int initWait = 0;
   int opmode = Param::GetInt(Param::opmode);
   int chargemode = Param::GetInt(Param::chargemode);
   int newMode = MOD_OFF;
   s32fp udc = ProcessUdc();

   GetDigInputs();
   ProcessThrottle();
   CalcAndOutputTemp();
   Param::SetInt(Param::speed, Encoder::GetSpeed());

   if (MOD_RUN == opmode)
   {
      CalcAmpAndSlip();
   }
   else if (MOD_OFF == opmode)
   {
      PwmGeneration::SetCurrentOffset(AnaIn::Get(AnaIn::il1), AnaIn::Get(AnaIn::il2));
   }

   /* switch on DC switch above threshold but only if
    * - throttle is not pressed
    * - start pin is high
    * - motor protection switch and emcystop is high (=inactive)
    */
   if (DigIo::Get(Pin::emcystop_in) &&
       DigIo::Get(Pin::mprot_in) &&
       Param::GetInt(Param::potnom) <= 0)
   {
      if (udc >= Param::Get(Param::udcsw))
      {
         /* Switch to charge mode if
          * - Charge mode is enabled
          * - Fwd AND Rev are high
          */
         if (DigIo::Get(Pin::fwd_in) && DigIo::Get(Pin::rev_in) && !DigIo::Get(Pin::bms_in) && chargemode >= MOD_BOOST)
         {
            //In buck mode we precharge to a different voltage
            if ((chargemode == MOD_BUCK && udc >= Param::Get(Param::udcswbuck)) || chargemode == MOD_BOOST)
               newMode = chargemode;
         }
         else if (Param::GetBool(Param::din_start))
         {
            newMode = MOD_RUN;
         }
      }
   }

   if (newMode != MOD_OFF)
   {
      DigIo::Set(Pin::dcsw_out);
      DigIo::Clear(Pin::err_out);
      DigIo::Clear(Pin::prec_out);
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }

   if (hwRev != HW_TESLA && opmode >= MOD_BOOST && DigIo::Get(Pin::bms_in))
   {
      opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   }

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      SetContactorsOffState();
      Param::SetInt(Param::amp, 0);
      PwmGeneration::SetOpmode(MOD_OFF);
      Throttle::cruiseSpeed = -1;
      runChargeControl = false;
   }
   else if (0 == initWait)
   {
      Encoder::Reset();
      PwmGeneration::PwmInit(); //this applies new deadtime and pwmfrq
      PwmGeneration::SetOpmode(opmode); //this enables the outputs for the given mode
      runChargeControl = (opmode == MOD_BOOST || opmode == MOD_BUCK);
      DigIo::Clear(Pin::err_out);
      initWait = -1;
   }
   else if (initWait > 0)
   {
      initWait--;
   }

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      Can::SendAll();
}

static void GenerateSpeedFrequencyOutput()
{
   static int speedCnt = 0;

   if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ)
   {
      int speed = Param::GetInt(Param::speed);
      if (speedCnt == 0 && speed != 0)
      {
         DigIo::Toggle(Pin::speed_out);
         speedCnt = Param::GetInt(Param::pwmgain) / (2 * speed);
      }
      else if (speedCnt > 0)
      {
         speedCnt--;
      }
   }
}

//Normal run takes 16µs -> 1.6% CPU load (last measured version 3.5)
static void Ms1Task(void)
{
   static s32fp ilFlt = 0;
   static s32fp iSpntFlt = 0;
   s32fp il1 = Param::Get(Param::il1);
   s32fp il2 = Param::Get(Param::il2);
   s32fp ilMax;
   int opmode = Param::GetInt(Param::opmode);

   il1 = ABS(il1);
   il2 = ABS(il2);
   ilMax = MAX(il1, il2);

   ErrorMessage::SetTime(rtc_get_counter_val());

   GenerateSpeedFrequencyOutput();

   if (runChargeControl)
   {
      ilFlt = IIRFILTER(ilFlt, ilMax << 8, Param::GetInt(Param::chargeflt));
      iSpntFlt = IIRFILTER(iSpntFlt, Param::Get(Param::chargecur) << 8, 11);

      s32fp ampnom = FP_MUL(Param::GetInt(Param::chargekp), (iSpntFlt - ilFlt));
      if (opmode == MOD_BOOST)
         Param::SetFlt(Param::idc, (FP_MUL((FP_FROMINT(100) - ampnom), ilFlt) / 100) >> 8);
      else
         Param::SetFlt(Param::idc, ilFlt >> 8);
      //Throttle::TemperatureDerate(Param::Get(Param::tmphs), ampnom);
      ampnom = MAX(0, ampnom);
      ampnom = MIN(Param::Get(Param::chargemax), ampnom);
      PwmGeneration::SetAmpnom(ampnom);
      Param::SetFlt(Param::ampnom, ampnom);
   }
   else
   {
      iSpntFlt = 0;
   }
}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
   if (Param::fslipspnt == paramNum)
      PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
   else if (Param::ampnom == paramNum)
      PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
   else if (Param::canspeed == paramNum)
      Can::SetBaudrate((enum Can::baudrates)Param::GetInt(Param::canspeed));
   else
   {
      PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));

      Encoder::SetFilterConst(Param::GetInt(Param::enckp));
      Encoder::SetMode((enum Encoder::mode)Param::GetInt(Param::encmode));
      Encoder::SetImpulsesPerTurn(Param::GetInt(Param::numimp));

      MotorVoltage::SetMinFrq(Param::Get(Param::fmin));
      MotorVoltage::SetMaxFrq(Param::Get(Param::fmax));
      SineCore::SetMinPulseWidth(Param::GetInt(Param::minpulse));

      Throttle::potmin[0] = Param::GetInt(Param::potmin);
      Throttle::potmax[0] = Param::GetInt(Param::potmax);
      Throttle::potmin[1] = Param::GetInt(Param::pot2min);
      Throttle::potmax[1] = Param::GetInt(Param::pot2max);
      Throttle::brknom = Param::GetInt(Param::brknom);
      Throttle::brknompedal = Param::GetInt(Param::brknompedal);
      Throttle::brkPedalRamp = Param::GetInt(Param::brkpedalramp);
      Throttle::brkmax = Param::GetInt(Param::brkmax);
      Throttle::idleSpeed = Param::GetInt(Param::idlespeed);
      Throttle::speedkp = Param::Get(Param::speedkp);
      Throttle::speedflt = Param::GetInt(Param::speedflt);
      Throttle::idleThrotLim = Param::Get(Param::idlethrotlim);

      if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ)
         gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
      else
         gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
   }
}

static void InitPWMIO()
{
   uint8_t outputMode = Param::GetInt(Param::pwmpol) == 0 ? GPIO_MODE_OUTPUT_50_MHZ : GPIO_MODE_INPUT;
   uint8_t outputConf = Param::GetInt(Param::pwmpol) == 0 ? GPIO_CNF_OUTPUT_ALTFN_PUSHPULL : GPIO_CNF_INPUT_FLOAT;

   gpio_set_mode(GPIOA, outputMode, outputConf, GPIO8 | GPIO9 | GPIO10);
   gpio_set_mode(GPIOB, outputMode, outputConf, GPIO13 | GPIO14 | GPIO15);
}

static void ConfigureVariantIO()
{
   AnaIn::AnaInfo analogInputs[] = ANA_IN_ARRAY;

   hwRev = detect_hw();
   Param::SetInt(Param::hwver, hwRev);

   switch (hwRev)
   {
      case HW_REV1:
         analogInputs[AnaIn::il2].port = GPIOA;
         analogInputs[AnaIn::il2].pin = 6;
         break;
      case HW_REV2:
         break;
      case HW_REV3:
         break;
      case HW_TESLA:
         DigIo::Configure(Pin::temp1_out, GPIOC, GPIO8, PinMode::OUTPUT);
         break;
   }

   AnaIn::Init(analogInputs);
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   clock_setup();
   rtc_setup();
   ConfigureVariantIO();

   //Additional test pins on JTAG header
   //AFIO_MAPR |= AFIO_MAPR_SPI1_REMAP | AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF;
   char* termBuf = usart_setup();
   tim_setup();
   DigIo::Init();
   nvic_setup();
   Encoder::Reset();
   term_Init(termBuf);
   parm_load();
   parm_Change(Param::PARAM_LAST);
   Can::Init((Can::baudrates)Param::GetInt(Param::canspeed));
   InitPWMIO();

   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms1Task, 100);
   s.AddTask(Ms10Task, 1000);
   s.AddTask(Ms100Task, 10000);

   DigIo::Set(Pin::prec_out);

   if (Param::GetInt(Param::snsm) < 12)
      Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter

   term_Run();

   return 0;
}

