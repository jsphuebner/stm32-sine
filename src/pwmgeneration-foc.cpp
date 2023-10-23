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
#include "foc.h"
#include "picontroller.h"

#define FRQ_TO_ANGLE(frq) FP_TOINT((frq << SineCore::BITS) / pwmfrq)
#define DIGIT_TO_DEGREE(a) FP_FROMINT(a) / (65536 / 360)

#ifndef QLIMIT_FREQUENCY
#define QLIMIT_FREQUENCY FP_FROMINT(30)
#endif // QLIMIT_FREQUENCY

static int initwait = 0;
static bool isIdle = false;
static const s32fp dcCurFac = FP_FROMFLT(0.81649658092772603273 * 1.05); //sqrt(2/3)*1.05 (inverter losses)
static const int32_t fwOutMax = 1024;
static const uint32_t shiftForFilter = 8;
static s32fp idMtpa = 0, iqMtpa = 0;
static tim_oc_id ocChannels[3];
static PiController qController;
static PiController dController;
static s32fp fwCurMax = 0;

void PwmGeneration::Run()
{
   if (opmode == MOD_RUN)
   {
      static s32fp idcFiltered = 0;
      static int amplitudeErrFiltered;
      int dir = Param::GetInt(Param::dir);
      s32fp id, iq;

      Encoder::UpdateRotorAngle(0);
      CalcNextAngleSync();
      FOC::SetAngle(angle);
      ProcessCurrents(id, iq);

      frqFiltered = IIRFILTER(frqFiltered, frq, 4);

      if (initwait == 0)
      {
         int amplitudeErr = (FOC::GetMaximumModulationIndex() - Param::GetInt(Param::vlimmargin)) - Param::GetInt(Param::amp);
         amplitudeErr = MIN(fwOutMax, amplitudeErr);
         amplitudeErr = MAX(fwOutMax / 20, amplitudeErr);
         amplitudeErrFiltered = IIRFILTER(amplitudeErrFiltered, amplitudeErr << shiftForFilter, Param::GetInt(Param::vlimflt));

         if (0 == frq) amplitudeErrFiltered = fwOutMax << shiftForFilter;

         int vlim = amplitudeErrFiltered >> shiftForFilter;
         s32fp ifw = ((fwOutMax - vlim) * fwCurMax) / fwOutMax;
         Param::SetFixed(Param::ifw, ifw);

         s32fp limitedIq = (vlim * iqMtpa) / fwOutMax;
         qController.SetRef(limitedIq + Param::Get(Param::manualiq));

         s32fp limitedId = -2 * ABS(limitedIq); //ratio between idMtpa and iqMtpa never > 2
         limitedId = MAX(idMtpa, limitedId);
         limitedId = MIN(ifw, limitedId);
         dController.SetRef(limitedId + Param::Get(Param::manualid));
      }

      int32_t ud = dController.Run(id);
      int32_t qlimit = FOC::GetQLimit(ud);

      if (frqFiltered < QLIMIT_FREQUENCY)
         qController.SetMinMaxY(dir <= 0 ? -qlimit : 0, dir >= 0 ? qlimit : 0);
      else
         qController.SetMinMaxY(-qlimit, qlimit);

      int32_t uq = qController.Run(iq);
      uint16_t advancedAngle = angle + dir * FP_TOINT(FP_MUL(Param::Get(Param::syncadv), frqFiltered));
      FOC::SetAngle(advancedAngle);
      FOC::InvParkClarke(ud, uq);

      s32fp idc = (iq * uq + id * ud) / FOC::GetMaximumModulationIndex();
      idc = FP_MUL(idc, dcCurFac);
      idcFiltered = IIRFILTER(idcFiltered, idc, Param::GetInt(Param::idcflt));

      uint32_t amp = FOC::GetTotalVoltage(ud, uq);

      Param::SetFixed(Param::fstat, frq);
      Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));
      Param::SetFixed(Param::idc, idcFiltered);
      Param::SetInt(Param::uq, uq);
      Param::SetInt(Param::ud, ud);
      Param::SetInt(Param::amp, amp);

      /* Shut down PWM on stopped motor or init phase */
      if (isIdle || initwait > 0)
      {
         timer_disable_break_main_output(PWM_TIMER);
         dController.ResetIntegrator();
         qController.ResetIntegrator();
         RunOffsetCalibration();
         amplitudeErrFiltered = fwOutMax << shiftForFilter;
      }
      else
      {
         timer_enable_break_main_output(PWM_TIMER);
      }

      for (int i = 0; i < 3; i++)
      {
         timer_set_oc_value(PWM_TIMER, ocChannels[i], FOC::DutyCycles[i] >> shiftForTimer);
      }
   }
   else if (opmode == MOD_BOOST || opmode == MOD_BUCK)
   {
      initwait = 0;
      Charge();
   }
   else if (opmode == MOD_ACHEAT)
   {
      initwait = 0;
      AcHeat();
   }
}

void PwmGeneration::SetFwCurMax(float cur)
{
   fwCurMax = FP_FROMFLT(cur);
}

void PwmGeneration::SetTorquePercent(float torquePercent)
{
   float is = Param::GetFloat(Param::throtcur) * torquePercent;
   float id, iq;

   FOC::Mtpa(is, id, iq);

   //This is used to disable PWM and do offset calibration at standstill
   isIdle = 0 == frq &&
            0 == torquePercent &&
            0 == Param::Get(Param::manualid) &&
            0 == Param::Get(Param::manualiq) &&
            hwRev != HW_PRIUS; //Don't do it for Prius Gen2 inverters

   iqMtpa = FP_FROMFLT(iq);
   idMtpa = FP_FROMFLT(id);
}

void PwmGeneration::SetControllerGains(int iqkp, int idkp, int ki)
{
   qController.SetGains(iqkp, ki);
   dController.SetGains(idkp, ki);
}

void PwmGeneration::PwmInit()
{
   int32_t maxVd = FOC::GetMaximumModulationIndex() - 1000;
   pwmfrq = TimerSetup(Param::GetInt(Param::deadtime), Param::GetInt(Param::pwmpol));
   Encoder::SetPwmFrequency(pwmfrq);
   initwait = pwmfrq / 2; //0.5s
   qController.ResetIntegrator();
   qController.SetCallingFrequency(pwmfrq);
   qController.SetMinMaxY(-maxVd, maxVd);
   dController.ResetIntegrator();
   dController.SetCallingFrequency(pwmfrq);
   dController.SetMinMaxY(-maxVd, maxVd);

   if ((Param::GetInt(Param::pinswap) & SWAP_PWM13) > 0)
   {
      ocChannels[0] = TIM_OC3;
      ocChannels[1] = TIM_OC2;
      ocChannels[2] = TIM_OC1;
   }
   else if ((Param::GetInt(Param::pinswap) & SWAP_PWM23) > 0)
   {
      ocChannels[0] = TIM_OC1;
      ocChannels[1] = TIM_OC3;
      ocChannels[2] = TIM_OC2;
   }
   else
   {
      ocChannels[0] = TIM_OC1;
      ocChannels[1] = TIM_OC2;
      ocChannels[2] = TIM_OC3;
   }

   if (opmode == MOD_ACHEAT)
      AcHeatTimerSetup();
}

s32fp PwmGeneration::ProcessCurrents(s32fp& id, s32fp& iq)
{
   if (initwait > 0)
   {
      initwait--;
   }

   s32fp il1 = GetCurrent(AnaIn::il1, ilofs[0], Param::Get(Param::il1gain));
   s32fp il2 = GetCurrent(AnaIn::il2, ilofs[1], Param::Get(Param::il2gain));

   if ((Param::GetInt(Param::pinswap) & SWAP_CURRENTS) > 0)
      FOC::ParkClarke(il2, il1);
   else
      FOC::ParkClarke(il1, il2);
   id = FOC::id;
   iq = FOC::iq;

   Param::SetFixed(Param::id, FOC::id);
   Param::SetFixed(Param::iq, FOC::iq);
   Param::SetFixed(Param::il1, il1);
   Param::SetFixed(Param::il2, il2);

   return 0;
}

void PwmGeneration::CalcNextAngleSync()
{
   if (Encoder::SeenNorthSignal())
   {
      uint16_t syncOfs = Param::GetInt(Param::syncofs);
      uint16_t rotorAngle = Encoder::GetRotorAngle();

      angle = polePairRatio * rotorAngle + syncOfs;
      frq = polePairRatio * Encoder::GetRotorFrequency();
   }
   else
   {
      frq = fslip;
      angle += FRQ_TO_ANGLE(fslip);
   }
}

void PwmGeneration::RunOffsetCalibration()
{
   static int il1Avg = 0, il2Avg = 0, samples = 0;
   const int offsetSamples = 512;

   if (samples < offsetSamples)
   {
      il1Avg += AnaIn::il1.Get();
      il2Avg += AnaIn::il2.Get();
      samples++;
   }
   else
   {
      SetCurrentOffset(il1Avg / offsetSamples, il2Avg / offsetSamples);
      il1Avg = il2Avg = 0;
      samples = 0;
   }
}

#define PHASE_SHIFT120   ((uint32_t)(     SINLU_ONEREV / 3))

enum test_state {INIT_CHECK,SET_CURRENT,INIT_P1,DEL_P1,CHECK_P1,INIT_P2,DEL_P2,CHECK_P2,INIT_P3,DEL_P3,CHECK_P3};
static test_state phaseCheckState;
static s32fp i1max=0, i2max=0, i3max=0;
static int checkCount = 0;
static char passResult[40] = "1:___ 2:___ 3:___ Inductance=ErrmH\r\n";
const char zeroCurrResult[] = "PhaseCheck Fail, Low Current\r\n";
const char overCurrResult[] = "PhaseCheck Fail, High Current\r\n";
const char* testResult = passResult;
static int testmode = TEST_OFF;
static uint16_t testAngle = 0; //used for driving rotation in manual mode
static int32_t syncErrorSum = 0, halfSyncErrAv = 0;

void PwmGeneration::InitTestMode()
{
    testmode = Param::GetInt(Param::testmode);
    testAngle = 0;
    phaseCheckState = INIT_CHECK;
    i1max=i2max=i3max=0;
    checkCount = 4000;
}

void PwmGeneration::TestModeRun()
{
   s32fp id, iq;
   static int32_t ud=0, uq=0;
   static s32fp i1sum=0, i2sum=0, i3sum=0;
   //static volatile s32fp results[9];
   static int LmH = 0;
   static s32fp testVolts=FP_FROMFLT(0.5f);
   static s32fp oldi1=0, oldi2=0, oldi3=0;
   static bool zeroCurrDone = false;
   static bool dirForward = true;

   s32fp imax = Param::Get(Param::maxtesti);
   s32fp imin = Param::Get(Param::mintesti);
   int32_t detThreshold = Param::GetInt(Param::testdetlev);

   //note - this will cause first run to be too slow to see phase check response in web interface
   if((zeroCurrDone == false) && (checkCount>2000))
   {
      checkCount--;
      timer_disable_break_main_output(PWM_TIMER);
      RunOffsetCalibration();
      if(checkCount==2000)
         zeroCurrDone=true;
      return;
   }

   Encoder::UpdateRotorAngle(0);
   CalcNextAngleSync();
   Param::SetFixed(Param::angle, DIGIT_TO_DEGREE(angle));

   ProcessCurrents(id, iq); //don't care about id and iq here but need phase currents
   s32fp i1 = Param::Get(Param::il1);
   s32fp i2 = Param::Get(Param::il2);
   s32fp i3 = -i1 - i2;

   if(ud>0)
   {
       i1sum += (i1-oldi1);
       i2sum += (i2-oldi2);
       i3sum += (i3-oldi3);
   }
   else
   {
       i1sum -= (i1-oldi1);
       i2sum -= (i2-oldi2);
       i3sum -= (i3-oldi3);
   }

   oldi1=i1; oldi2=i2; oldi3=i3;

   i1max = MAX(i1max, ABS(i1));
   i2max = MAX(i2max, ABS(i2));
   i3max = MAX(i3max, ABS(i3));

   if(TEST_PHASE == testmode)
   {
       if((i1max > imax) || (i2max > imax) || (i3max > imax))
       {
          timer_disable_break_main_output(PWM_TIMER);
          testResult = overCurrResult;
          SetOpmode(MOD_OFF);
          Param::SetInt(Param::opmode, MOD_OFF);
          phaseCheckState = INIT_CHECK;
       }

      checkCount++;
      if(checkCount>31)
      {
         checkCount = 0;
         switch(phaseCheckState)
         {
         case INIT_CHECK:
            timer_enable_break_main_output(PWM_TIMER);
            FOC::SetAngle(0);
            testVolts = FP_FROMFLT(0.5f);
            i1max = 0; i2max = 0; i3max = 0;
            phaseCheckState = SET_CURRENT;
            break;
         case SET_CURRENT:
            if((i1max > imin) || (i2max > imin) || (i3max > imin)) //big enough to proceed
            {
               phaseCheckState = INIT_P1;
               s32fp imax = MAX((MAX(i1max,i2max)),i3max);
               LmH = FP_TOINT(FP_DIV(FP_MUL(testVolts, FP_FROMFLT(18)),imax));
            }
            else
            {
               testVolts = FP_MUL(testVolts, FP_FROMFLT(1.25f));
               if(testVolts <= Param::Get(Param::maxtestud))
               {
                  i1max = 0; i2max = 0; i3max = 0;
               }
               else //already at max volts
               {
                  timer_disable_break_main_output(PWM_TIMER);
                  testResult = zeroCurrResult;
                  SetOpmode(MOD_OFF);
                  Param::SetInt(Param::opmode, MOD_OFF);
                  phaseCheckState = INIT_CHECK;
               }
            }
            break;
         case INIT_P1: //let currents stabilise
            phaseCheckState = DEL_P1;
            i1sum = 0; i2sum = 0; i3sum = 0;
            break;
         case DEL_P1:
            phaseCheckState = CHECK_P1;
            break;
         case CHECK_P1:
            //results[0] = i1sum; results[1] = i2sum; results[2] = i3sum;
            passResult[2] = (ABS(i1sum) < detThreshold) ? '0' : ((i1sum>0)?'^':'v');
            passResult[3] = (ABS(i2sum) < detThreshold) ? '0' : ((i2sum>0)?'^':'v');
            passResult[4] = (ABS(i3sum) < detThreshold) ? '0' : ((i3sum>0)?'^':'v');
            FOC::SetAngle(PHASE_SHIFT120);
            phaseCheckState = INIT_P2;
            break;
         case INIT_P2: //let currents stabilise
            phaseCheckState = DEL_P2;
            i1sum = 0; i2sum = 0; i3sum = 0;
            break;
         case DEL_P2:
            phaseCheckState = CHECK_P2;
            break;
         case CHECK_P2:
            //results[3] = i1sum; results[4] = i2sum; results[5] = i3sum;
            passResult[8]  = (ABS(i1sum) < detThreshold) ? '0' : ((i1sum>0)?'^':'v');
            passResult[9]  = (ABS(i2sum) < detThreshold) ? '0' : ((i2sum>0)?'^':'v');
            passResult[10] = (ABS(i3sum) < detThreshold) ? '0' : ((i3sum>0)?'^':'v');
            FOC::SetAngle(PHASE_SHIFT120 + PHASE_SHIFT120);
            phaseCheckState = INIT_P3;
            break;
         case INIT_P3: //let currents stabilise
            phaseCheckState = DEL_P3;
            i1sum = 0; i2sum = 0; i3sum = 0;
            break;
         case DEL_P3:
            phaseCheckState = CHECK_P3;
            break;
         case CHECK_P3:
            //results[6] = i1sum; results[7] = i2sum; results[8] = i3sum;
            passResult[14] = (ABS(i1sum) < detThreshold) ? '0' : ((i1sum>0)?'^':'v');
            passResult[15] = (ABS(i2sum) < detThreshold) ? '0' : ((i2sum>0)?'^':'v');
            passResult[16] = (ABS(i3sum) < detThreshold) ? '0' : ((i3sum>0)?'^':'v');
            timer_disable_break_main_output(PWM_TIMER);
            if(LmH<100)
            {
                passResult[29] = (((LmH/10)%10) + '0');
                passResult[30] = '.';
                passResult[31] = ((LmH%10) + '0');
            }
            else
            {
                passResult[29] = '>';
                passResult[30] = '1';
                passResult[31] = '0';
            }
            testResult = passResult;
            SetOpmode(MOD_OFF);
            Param::SetInt(Param::opmode, MOD_OFF);
            phaseCheckState = INIT_CHECK;
            break;
         }
      }

      s32fp udc = Param::Get(Param::udc)/2;
      uq=0;
      if(checkCount<16)
         ud = (testVolts * FOC::GetMaximumModulationIndex()) / udc; //result is an int so no FP_DIV
      else
         ud = -((testVolts * FOC::GetMaximumModulationIndex()) / udc);

   }
   else if((TEST_SPINF == testmode) || (TEST_SPINBIDIR == testmode))
   {
      timer_enable_break_main_output(PWM_TIMER);
      dController.SetRef(Param::Get(Param::manualid));
      qController.SetRef(Param::Get(Param::manualiq));
      FOC::SetAngle(testAngle);
      ProcessCurrents(id, iq); //assumes will be running as a stepper motor so aligned with id
      Param::SetFixed(Param::testangle, DIGIT_TO_DEGREE(testAngle));

      if(TEST_SPINF == testmode)
      {
         testAngle+=2; //get ~8sec per electrical rev or 32sec per mech rev (4pole pairs)
         Param::SetInt(Param::syncoferr,0);
         Param::SetInt(Param::syncoferrav,0);
      }
      else //reversing
      {
         int32_t syncError = (int32_t)((int16_t)(testAngle - angle));
         syncErrorSum += syncError;
         Param::SetInt(Param::syncoferr,syncError);

         if(dirForward)
            testAngle+=2;
         else
            testAngle-=2;
         if(((SINLU_ONEREV-1) == testAngle) || (0 == testAngle))
         {
            if((0 == testAngle))
               halfSyncErrAv = syncErrorSum>>16; //note 32768 points per pass so this also divides by 2
            else
               Param::SetInt(Param::syncoferrav,(int32_t)((uint16_t)((syncErrorSum>>16) + halfSyncErrAv))); //use average of the two
            syncErrorSum = 0;
            dirForward = !dirForward;
         }
      }

      ud = dController.Run(id);
      int32_t qlimit = FOC::GetQLimit(ud);
      qController.SetMinMaxY(-qlimit, qlimit);
      uq = qController.Run(iq);
   }
   else // TEST_RESLV
   {
      timer_disable_break_main_output(PWM_TIMER);
      ud = 0;
      uq = 0;
   }

   FOC::InvParkClarke(ud, uq);

   s32fp idc = (iq * uq + id * ud) / FOC::GetMaximumModulationIndex();
   idc = FP_MUL(idc, dcCurFac);

   uint32_t amp = FOC::GetTotalVoltage(ud, uq);

   Param::SetFixed(Param::fstat, frq);
   Param::SetInt(Param::uq, uq);
   Param::SetInt(Param::ud, ud);
   Param::SetInt(Param::amp, amp);
   Param::SetFixed(Param::ifw, 0);

   for (int i = 0; i < 3; i++)
   {
      timer_set_oc_value(PWM_TIMER, ocChannels[i], FOC::DutyCycles[i] >> shiftForTimer);
   }
}

int PwmGeneration::GetOpmode()
{
   return opmode;
}

const char* PwmGeneration::GetTestResult()
{
   return testResult;
}
