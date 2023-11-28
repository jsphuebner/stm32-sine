/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2012 Johannes Huebner <contact@johanneshuebner.com>
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

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin[2];
int Throttle::potmax[2];
float Throttle::brknom;
float Throttle::brknompedal;
float Throttle::brkmax;
float Throttle::brkcruise;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
float Throttle::speedkp;
float Throttle::holdkp;
int Throttle::speedflt;
int Throttle::speedFiltered;
float Throttle::idleThrotLim;
float Throttle::cruiseThrotLim;
float Throttle::potnomFiltered;
float Throttle::throtmax;
float Throttle::throtmin;
float Throttle::regenRamp;
float Throttle::throttleRamp;
float Throttle::throttleRamped;
int Throttle::bmslimhigh;
int Throttle::bmslimlow;
float Throttle::udcmin;
float Throttle::udcmax;
float Throttle::idcmin;
float Throttle::idcmax;
float Throttle::idckp;
float Throttle::fmax;
int Throttle::accelmax;
int Throttle::accelflt;
float Throttle::maxregentravelhz;

bool Throttle::CheckAndLimitRange(int* potval, int potIdx)
{
   int potMin = potmax[potIdx] > potmin[potIdx] ? potmin[potIdx] : potmax[potIdx];
   int potMax = potmax[potIdx] > potmin[potIdx] ? potmax[potIdx] : potmin[potIdx];

   if (((*potval + POT_SLACK) < potMin) || (*potval > (potMax + POT_SLACK)))
   {
      *potval = potMin;
      return false;
   }
   else if (*potval < potMin)
   {
      *potval = potMin;
   }
   else if (*potval > potMax)
   {
      *potval = potMax;
   }

   return true;
}

float Throttle::DigitsToPercent(int potval, int potidx)
{
   if (potidx > 1) return 0;
   if (potmax[potidx] == potmin[potidx]) return 100.0f;

   return (100 * (potval - potmin[potidx])) / (potmax[potidx] - potmin[potidx]);
}

float Throttle::CalcThrottle(float potnom, float pot2nom, bool brkpedal, float rotorfreq)
{
   static float brknomDynamic = 0.01;
   static float pedalPressPercentage = 0.0f;
   pedalPressPercentage = potnom;

   float scaledBrkMax = brkpedal ? brknompedal : brkmax;

   //Never reach 0, because that can spin up the motor
   scaledBrkMax = -0.1 + (scaledBrkMax * pot2nom) / 100.0f;
   
   //maxregentravelhz (speed) gives max brknomDynamic (brknom)
   float speedPercentage = MIN((rotorfreq / maxregentravelhz), 1.0); 

   //set max regen by speed
   if (brknomDynamic < brknom * speedPercentage)  
   {
      brknomDynamic = speedPercentage * brknom;
   }

   //reset to full pedal travel when stopped and pedal 0% on throttle
   if (potnom <= 0.1 && rotorfreq < 0.1)  
   {
      brknomDynamic = 0.01; 
   }

   if (brkpedal) 
   {
      potnom = scaledBrkMax;
   }
   else if (potnom < brknomDynamic) //calculate regen
   {
      potnom -= brknomDynamic;
      potnom = -(potnom * scaledBrkMax / brknomDynamic);
   }
   else //calculate thorttle  
   {
      potnom -= brknomDynamic;
      potnom = 100.0f * potnom / (100.0f - brknomDynamic);
   }

   return potnom > pedalPressPercentage ? pedalPressPercentage : potnom;
}

float Throttle::CalcThrottleBiDir(float potval, bool brkpedal)
{
   if (brkpedal) return 0; //in bidir mode brake pedal sends motor to idle

   if (ABS(potval - 50.0f) <= (brknom / 2)) potval = 50.0f;

   float bidirPotval = 2 * potval - 100.0f;

   if (bidirPotval < 0)
      bidirPotval += brknom;
   else if (bidirPotval > 0)
      bidirPotval -= brknom;

   bidirPotval *= (100.0f + brknom) / 100.0f;

   return bidirPotval;
}

float Throttle::RampThrottle(float potnom)
{
   potnom = MIN(potnom, throtmax);
   potnom = MAX(potnom, throtmin);

   if (potnom >= throttleRamped)
   {
      throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
   }
   else if (potnom < throttleRamped && throttleRamped > 5)
   {
      throttleRamped = RAMPDOWN(throttleRamped, potnom, throttleRamp);
   }
   else //potnom < throttleRamped && potnom <= 0
   {
      throttleRamped = RAMPDOWN(throttleRamped, potnom, regenRamp);
   }

   return throttleRamped;
}

float Throttle::CalcIdleSpeed(int speed)
{
   int speederr = idleSpeed - speed;
   return MIN(idleThrotLim, speedkp * speederr);
}

float Throttle::CalcCruiseSpeed(int speed)
{
   speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
   int speederr = cruiseSpeed - speedFiltered;

   float potnom = speedkp * speederr;
   potnom = MIN(cruiseThrotLim, potnom);
   potnom = MAX(brkcruise, potnom);

   return potnom;
}

bool Throttle::HoldPosition(int distance, float& finalSpnt)
{
   //do not act when rolling forward and exit hill hold mode
   if (distance > 10000) return false;
   finalSpnt = (holdkp * distance) / 1000;
   finalSpnt = MIN(idleThrotLim, finalSpnt);
   finalSpnt = MAX(0, finalSpnt);
   return true;
}

bool Throttle::TemperatureDerate(float temp, float tempMax, float& finalSpnt)
{
   float limit = 0;

   if (temp <= tempMax)
      limit = 100.0f;
   else if (temp < (tempMax + 2.0f))
      limit = 50.0f;

   if (finalSpnt >= 0)
      finalSpnt = MIN(finalSpnt, limit);
   else
      finalSpnt = MAX(finalSpnt, -limit);

   return limit < 100.0f;
}

void Throttle::BmsLimitCommand(float& finalSpnt, bool dinbms)
{
   if (dinbms)
   {
      if (finalSpnt >= 0)
         finalSpnt = (finalSpnt * bmslimhigh) / 100;
      else
         finalSpnt = -(finalSpnt * bmslimlow) / 100;
   }
}

void Throttle::UdcLimitCommand(float& finalSpnt, float udc)
{
   if (finalSpnt >= 0)
   {
      float udcErr = udc - udcmin;
      float res = udcErr * 5;
      res = MAX(0, res);
      finalSpnt = MIN(finalSpnt, res);
   }
   else
   {
      float udcErr = udc - udcmax;
      float res = udcErr * 5;
      res = MIN(0, res);
      finalSpnt = MAX(finalSpnt, res);
   }
}

void Throttle::IdcLimitCommand(float& finalSpnt, float idc)
{
   if (finalSpnt >= 0)
   {
      float idcerr = idcmax - idc;
      float res = idcerr * idckp;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
   else
   {
      float idcerr = idcmin - idc;
      float res = idcerr * idckp;

      res = MIN(0, res);
      finalSpnt = MAX(res, finalSpnt);
   }
}

void Throttle::AccelerationLimitCommand(float& finalSpnt, int speed)
{
   static int lastSpeed = 0, speedDiff = 0;

   speedDiff = IIRFILTER(speedDiff, speed - lastSpeed, accelflt);

   if (finalSpnt >= 0 && speed > 100)
   {
      int accelErr = accelmax - speedDiff;
      int res = 20 * accelErr;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
   lastSpeed = speed;
}

void Throttle::FrequencyLimitCommand(float& finalSpnt, float frequency)
{
   static float frqFiltered = 0;

   frqFiltered = IIRFILTERF(frqFiltered, frequency, 4);

   if (finalSpnt > 0)
   {
      float frqerr = fmax - frqFiltered;
      float res = frqerr * 4;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
}
