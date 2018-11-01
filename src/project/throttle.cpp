/*
 * This file is part of the tumanako_vc project.
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
int Throttle::brknom;
int Throttle::brknompedal;
int Throttle::brkmax;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
s32fp Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered;
s32fp Throttle::idleThrotLim;
int Throttle::throtmax;
int Throttle::brkPedalRamp;
int Throttle::brkRamped;
int Throttle::throttleRamp;
int Throttle::throttleRamped;

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

bool Throttle::CheckDualThrottle(int* potval, int pot2val)
{
   int potnom1, potnom2;
   //2nd input running inverse
   if (potmin[1] > potmax[1])
   {
      potnom2 = 100 - (100 * (pot2val - potmax[1])) / (potmin[1] - potmax[1]);
   }
   else
   {
      potnom2 = (100 * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
   }
   potnom1 = (100 * (*potval - potmin[0])) / (potmax[0] - potmin[0]);
   int diff = potnom2 - potnom1;
   diff = ABS(diff);

   if (diff > 10)
   {
      *potval = potmin[0];
      return false;
   }
   return true;
}

int Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
{
   int potnom;
   int scaledBrkMax = brkpedal ? brknompedal : -brkmax;

   if (pot2val > potmin[1])
   {
      potnom = (100 * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
      //Never reach 0, because that can spin up the motor
      scaledBrkMax = -1 + (scaledBrkMax * potnom) / 100;
   }

   potnom = potval - potmin[0];
   potnom = ((100 + brknom) * potnom) / (potmax[0] - potmin[0]);
   potnom -= brknom;

   if (potnom > 0)
   {
      throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
      potnom = (throttleRamped * throtmax) / 100;
   }
   else
   {
      throttleRamped = 0;
   }

   if (potnom < 0)
   {
      scaledBrkMax = -(potnom * scaledBrkMax) / brknom;
   }

   if (brkpedal || potnom < 0)
   {
      brkRamped = RAMPDOWN(brkRamped, scaledBrkMax, brkPedalRamp);
      potnom = brkRamped;
   }
   else
   {
      brkRamped = MIN(0, potnom); //reset ramp
   }

   return potnom;
}

int Throttle::CalcIdleSpeed(int speed)
{
   int speederr = idleSpeed - speed;
   return FP_TOINT(MIN(idleThrotLim, speedkp * speederr));
}

int Throttle::CalcCruiseSpeed(int speed)
{
   speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
   int speederr = cruiseSpeed - speedFiltered;
   return FP_TOINT(MAX(FP_FROMINT(-brkmax), MIN(FP_FROMINT(100), speedkp * speederr)));
}

bool Throttle::TemperatureDerate(s32fp tmphs, int& finalSpnt)
{
   int limit = 0;

   if (tmphs <= TMPHS_MAX)
      limit = 100;
   else if (tmphs < (TMPHS_MAX + FP_FROMINT(2)))
      limit = 50;

   if (finalSpnt >= 0)
      finalSpnt = MIN(finalSpnt, limit);
   else
      finalSpnt = MAX(finalSpnt, -limit);

   return limit < 100;
}
