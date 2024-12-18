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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/iwdg.h>
#include "terminal.h"
#include "terminalcommands.h"
#include "sine_core.h"
#include "fu.h"
#include "foc.h"
#include "hwdefs.h"
#include "hwinit.h"
#include "params.h"
#include "param_save.h"
#include "digio.h"
#include "anain.h"
#include "inc_encoder.h"
#include "throttle.h"
#include "my_math.h"
#include "stm32scheduler.h"
#include "pwmgeneration.h"
#include "temp_meas.h"
#include "vehiclecontrol.h"
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "GD31xxOI.h"
#include "delay.h"

#define PRINT_JSON 0

HWREV hwRev; //Hardware variant of board we are running on

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static Terminal* terminal;
static bool seenBrakePedal = false;

static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = PwmGeneration::GetCpuLoad() + scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::turns, Encoder::GetFullTurns());
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

   if (hwRev == HW_REV1 || hwRev == HW_BLUEPILL)
   {
      //If break pin is high and both mprot and emcystop are high than it must be over current
      if (DigIo::emcystop_in.Get() && DigIo::mprot_in.Get() && DigIo::bk_in.Get())
      {
         Param::SetInt(Param::din_ocur, 0);
      }
      else
      {
         Param::SetInt(Param::din_ocur, 1);
      }
      Param::SetInt(Param::din_desat, 2);
   }

   if (rtc_get_counter_val() > 50) //500ms after start check for brake pedal
      seenBrakePedal |= (Param::GetInt(Param::cruisemode) == CRUISE_OFF) || Param::GetBool(Param::din_brake);

   VehicleControl::SelectDirection();
   VehicleControl::CruiseControl();

   if (HW_MG == hwRev) MGSPI::CyclicFunction();

   #if CONTROL == CTRL_SINE
   //uac = udc * amp/maxamp / sqrt(2)
   float uac = Param::GetFloat(Param::udc) * SineCore::GetAmp();
   uac /= SineCore::MAXAMP;
   uac /= 1.4142;

   Param::SetFloat(Param::uac, uac);
   #endif // CONTROL

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      canMap->SendAll();
}

static void RunCharger(float udc)
{
   static float chargeCurRamped = 0;

   float chargeCur = Param::GetFloat(Param::chargecur);
   float tempDerate = 100;
   float udcDerate = -100; //we use the regen udc limiter, therefor negative starting value

   Throttle::TemperatureDerate(Param::GetFloat(Param::tmphs), Param::GetFloat(Param::tmphsmax), tempDerate);
   Throttle::UdcLimitCommand(udcDerate, udc);
   udcDerate = MIN(-udcDerate, tempDerate); //and back to positive
   chargeCur = udcDerate * chargeCur / 100;

   if (chargeCur < chargeCurRamped)
      chargeCurRamped = chargeCur;
   else
      chargeCurRamped = RAMPUP(chargeCurRamped, chargeCur, 1);
   PwmGeneration::SetChargeCurrent(chargeCurRamped);
}

//Normal run takes 70µs -> 0.7% cpu load (last measured version 3.5)
static void Ms10Task(void)
{
   static int initWait = 0;
   int opmode = Param::GetInt(Param::opmode);
   int chargemode = Param::GetInt(Param::chargemode);
   int newMode = MOD_OFF;
   int stt = STAT_NONE;
   float udc = VehicleControl::ProcessUdc();

   ErrorMessage::SetTime(rtc_get_counter_val());
   Encoder::UpdateRotorFrequency(100);
   VehicleControl::CalcAndOutputTemp();
   VehicleControl::GetDigInputs();
   float torquePercent = VehicleControl::ProcessThrottle();

   Param::SetInt(Param::speed, Encoder::GetSpeed());
   Param::SetInt(Param::rotordir, Encoder::GetRotorDirection());

   if (MOD_RUN == opmode && initWait == -1)
   {
      PwmGeneration::SetTorquePercent(torquePercent);
   }
   else if ((MOD_BOOST == opmode || MOD_BUCK == opmode) && initWait == -1)
   {
      RunCharger(udc);
   }

   stt |= DigIo::emcystop_in.Get() || hwRev == HW_REV3 ? STAT_NONE : STAT_EMCYSTOP;
   stt |= DigIo::mprot_in.Get() ? STAT_NONE : STAT_MPROT;
   stt |= Throttle::IsThrottlePressed(Param::GetInt(Param::pot)) ? STAT_POTPRESSED : STAT_NONE;
   stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;
   stt |= seenBrakePedal ? STAT_NONE : STAT_BRAKECHECK;

   /* switch on DC switch if
    * - throttle is not pressed
    * - start pin is high
    * - motor protection switch and emcystop is high (=inactive)
    * - udc >= udcsw
    * - udc < udclim
    */
   if ((stt & (STAT_EMCYSTOP | STAT_MPROT | STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM | STAT_BRAKECHECK)) == STAT_NONE)
   {
      /* Switch to charge mode if
       * - Charge mode is enabled
       * - Fwd AND Rev are high
       */
      if (Param::GetBool(Param::din_forward) &&
          Param::GetBool(Param::din_reverse) &&
         !Param::GetBool(Param::din_bms) &&
          chargemode >= MOD_BOOST)
      {
         //In buck mode we precharge to a different voltage
         if ((chargemode == MOD_BUCK && udc >= Param::GetFloat(Param::udcswbuck)) || chargemode == MOD_BOOST)
         {
            newMode = chargemode;

            //Prius needs to run PWM before closing the contactor
            if (hwRev == HW_PRIUS && opmode != MOD_BOOST && opmode != MOD_BUCK)
            {
               PwmGeneration::SetChargeCurrent(0);
               PwmGeneration::SetOpmode(newMode);
            }
         }
      }
      else if (Param::GetBool(Param::din_start) || Param::GetBool(Param::manualstart) ||
              (Param::GetInt(Param::tripmode) == TRIP_AUTORESUME && PwmGeneration::Tripped()))
      {
         newMode = MOD_RUN;
         Param::SetInt(Param::manualstart, 0);
      }
      stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
   }

   Param::SetInt(Param::status, stt);

   if (newMode != MOD_OFF)
   {
      opmode = newMode;
      DigIo::dcsw_out.Set();
      DigIo::err_out.Clear();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }

   if (hwRev != HW_TESLA && opmode >= MOD_BOOST && Param::GetBool(Param::din_bms))
   {
      opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   }

   if (MOD_OFF == opmode)
   {
      initWait = 50;

      VehicleControl::SetContactorsOffState();
      PwmGeneration::SetOpmode(MOD_OFF);
      Throttle::cruiseSpeed = -1;
      TerminalCommands::EnableSaving();
      canSdo->EnableSaving();
   }
   else if (0 == initWait)
   {
      //Disable saving in Run mode
      TerminalCommands::DisableSaving();
      canSdo->DisableSaving();
      PwmGeneration::SetTorquePercent(0);
      Throttle::RampThrottle(0); //Restart ramp
      Encoder::Reset();
      //this applies new deadtime and pwmfrq and enables the outputs for the given mode
      PwmGeneration::SetOpmode(opmode);
      DigIo::err_out.Clear();
      DigIo::prec_out.Clear();
      initWait = -1;
   }
   else if (initWait == 10)
   {
      PwmGeneration::SetCurrentOffset(AnaIn::il1.Get(), AnaIn::il2.Get());
      initWait--;
   }
   else if (initWait > 0)
   {
      initWait--;
   }

   Param::SetInt(Param::uptime, rtc_get_counter_val());

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      canMap->SendAll();
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   #if CONTROL == CTRL_SINE
      case Param::fslipspnt:
         PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
         break;
      case Param::ampnom:
         PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
         break;
   #endif
      case Param::canspeed:
         can->SetBaudrate((CanHardware::baudrates)Param::GetInt(Param::canspeed));
         break;
      case Param::outmode:
         if (hwRev == HW_BLUEPILL) return; //disable for blue pill
         switch (Param::GetInt(Param::outmode))
         {
            default:
            case PRECHARGE_DCSW:
               /* Map dcsw to actual output pins, fan to dummy pin */
               DigIo::dcsw_out.Configure(GPIOC, GPIO13, PinMode::OUTPUT);
               DigIo::fan_out.Configure(GPIOA, GPIO0, PinMode::OUTPUT);
               break;
            case ERR_TMPM_THRESH:
            case ERR_TMPHS_THRESH:
               /* Map dcsw to dummy pin, fan to actual output pins */
               DigIo::fan_out.Configure(GPIOC, GPIO13, PinMode::OUTPUT);
               DigIo::dcsw_out.Configure(GPIOA, GPIO0, PinMode::OUTPUT);
               break;
         }
         break;
      case Param::throtmax:
      case Param::throtmin:
      case Param::idcmin:
      case Param::idcmax:
      case Param::offthrotregen:
         //These are candidates to be frequently set by CAN, so we handle them separately
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         break;
      case Param::nodeid:
         canSdo->SetNodeId(Param::GetInt(Param::nodeid));
         //terminal->SetNodeId(Param::GetInt(Param::nodeid));
         break;
      default:
         PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
         PwmGeneration::SetPolePairRatio(Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs));

         #if CONTROL == CTRL_FOC
         PwmGeneration::SetControllerGains(Param::GetInt(Param::iqkp), Param::GetInt(Param::idkp), Param::GetInt(Param::curki));
         Encoder::SwapSinCos((Param::GetInt(Param::pinswap) & SWAP_RESOLVER) > 0);
         FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld) / 1000.0f, Param::GetFloat(Param::fluxlinkage) / 1000.0f);
         FOC::SetMaximumModulationIndex(Param::GetInt(Param::modmax));
         #endif // CONTROL

         Encoder::SetMode((enum Encoder::mode)Param::GetInt(Param::encmode));
         Encoder::SetImpulsesPerTurn(Param::GetInt(Param::numimp));
         Encoder::SetSinCosOffset(Param::GetInt(Param::sincosofs));

         Throttle::potmin[0] = Param::GetInt(Param::potmin);
         Throttle::potmax[0] = Param::GetInt(Param::potmax);
         Throttle::potmin[1] = Param::GetInt(Param::pot2min);
         Throttle::potmax[1] = Param::GetInt(Param::pot2max);
         Throttle::brknom = Param::GetFloat(Param::regentravel);
         Throttle::brknompedal = Param::GetFloat(Param::brakeregen);
         Throttle::regenRamp = Param::GetFloat(Param::regenramp);
         Throttle::maxregentravelhz = Param::GetFloat(Param::maxregentravelhz);
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         Throttle::brkcruise = Param::GetFloat(Param::cruiseregen);
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idleSpeed = Param::GetInt(Param::idlespeed);
         Throttle::holdkp = Param::GetFloat(Param::holdkp);
         Throttle::speedkp = Param::GetFloat(Param::speedkp);
         Throttle::speedflt = Param::GetInt(Param::speedflt);
         Throttle::idleThrotLim = Param::GetFloat(Param::idlethrotlim);
         Throttle::cruiseThrotLim = Param::GetFloat(Param::cruisethrotlim);
         Throttle::bmslimlow = Param::GetInt(Param::bmslimlow);
         Throttle::bmslimhigh = Param::GetInt(Param::bmslimhigh);
         Throttle::udcmin = Param::GetFloat(Param::udcmin) * 0.99f; //Leave some room for the notification light
         Throttle::udcmax = Param::GetFloat(Param::udcmax) * 1.01f;
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         Throttle::idckp = Param::GetFloat(Param::idckp);
         Throttle::accelflt = Param::GetInt(Param::accelflt);
         Throttle::accelmax = Param::GetInt(Param::accelmax);

         if (hwRev != HW_BLUEPILL)
         {
            if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ)
               gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
            else
               gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
         }
         break;
   }
}

static void UpgradeParameters()
{
   Param::SetInt(Param::version, 4); //backward compatibility
   Param::SetInt(Param::hwver, hwRev);
   Param::SetInt(Param::regenpreset, 100); //default to 100% regen if not CAN mapped

   if (Param::GetInt(Param::snsm) < 12)
      Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter
   if (Param::Get(Param::offthrotregen) > 0)
      Param::Set(Param::offthrotregen, -Param::Get(Param::offthrotregen));

   s32fp maxPotMax = Param::GetAttrib(Param::potmax)->max;
   s32fp potMax = Param::Get(Param::potmax);

   if (potMax > maxPotMax)
      Param::SetFixed(Param::potmax, maxPotMax);

   //Remove CAN mapping for safety critical values
   canMap->Remove(Param::pot);
   canMap->Remove(Param::pot2);
   canMap->Remove(Param::canio);
   canMap->Remove(Param::cruisespeed);
   canMap->Remove(Param::regenpreset);
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" void tim4_isr(void)
{
   scheduler->Run();
}

//C++ run time requires that when using interfaces and not optimizing for size
extern "C" void __cxa_pure_virtual() { while (1); }

extern "C" int main(void)
{
   extern const TERM_CMD TermCmds[];

   clock_setup();
   rtc_setup();
   hwRev = io_setup();
   tim_setup();
   nvic_setup();
   parm_load();
   ErrorMessage::SetTime(1);
   if (HW_MG == hwRev) MGSPI::Initialize();
   Param::SetInt(Param::pwmio, pwmio_setup(Param::GetBool(Param::pwmpol)));

   MotorVoltage::SetMaxAmp(SineCore::MAXAMP);
   PwmGeneration::SetCurrentOffset(2048, 2048);

   Stm32Scheduler s(hwRev == HW_BLUEPILL ? TIM4 : TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;
   Stm32Can c(CAN1, (CanHardware::baudrates)Param::GetInt(Param::canspeed));
   CanMap cm(&c);
   CanSdo sdo(&c, &cm);
   can = &c;
   canMap = &cm;
   canSdo = &sdo;
   VehicleControl::SetCan(can);
   TerminalCommands::SetCanMap(canMap);

   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms10Task, 10);

   DigIo::prec_out.Set();

   Terminal t(USART3, TermCmds);
   terminal = &t;

   if (hwRev == HW_REV1)
      t.DisableTxDMA();

   UpgradeParameters();
   Param::Change(Param::PARAM_LAST);
   Param::Change(Param::nodeid);
   Param::Change(Param::outmode);
   write_bootloader_pininit(Param::GetBool(Param::bootprec), Param::GetBool(Param::pwmpol));

   while(1)
   {
      char c = 0;
      t.Run();
      if (canSdo->GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(canSdo, &c);
      }
   }

   return 0;
}

