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

#include "my_fp.h"
#include "my_math.h"
#include "test_list.h"
#include "throttle.h"

using namespace std;

static void TestSetup()
{
      Throttle::potmin[0] = 1000;
      Throttle::potmax[0] = 2000;
      Throttle::potmin[1] = 3000;
      Throttle::potmax[1] = 4000;
      Throttle::brknom = 30;
      Throttle::brknompedal = -60;
      Throttle::regenRamp = 25;
      Throttle::brkmax = 50;
      Throttle::idleSpeed = 100;
      Throttle::speedkp = FP_FROMFLT(0.25);
      Throttle::speedflt = 5;
      Throttle::idleThrotLim = FP_FROMFLT(30);
}

static void TestBrkPedal()
{
   int percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -25)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -50)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -60)
   percent = Throttle::CalcThrottle(1500, 3000, true);
   ASSERT(percent == -60)
}

static void TestRegen()
{
   Throttle::CalcThrottle(2000, 3000, false);
   int percent = Throttle::CalcThrottle(1000, 3000, false);
   ASSERT(percent == -25)
}
#if 0
static void TestDualThrottle()
{
   //0% on both channels
   int pot = 1000;
   bool result = Throttle::CheckDualThrottle(&pot, 3000);
   ASSERT(result == true);

   //50% on both channels
   pot = 1500;
   result = Throttle::CheckDualThrottle(&pot, 3500);
   ASSERT(result == true);

   //100% on both channels
   pot = 2000;
   result = Throttle::CheckDualThrottle(&pot, 4000);
   ASSERT(result == true);

   //111% on first channel, 100% on second
   pot = 2110;
   result = Throttle::CheckDualThrottle(&pot, 4000);
   ASSERT(result == false);
   ASSERT(pot == Throttle::potmin[0]);

   //50% on first channel, -50% on second
   pot = 1500;
   result = Throttle::CheckDualThrottle(&pot, 2500);
   ASSERT(result == false);

   //50% on first channel, 61% on second
   pot = 1500;
   result = Throttle::CheckDualThrottle(&pot, 3610);
   ASSERT(result == false);

   //No we test inverted 2nd channel
   Throttle::potmin[1] = 4000;
   Throttle::potmax[1] = 3000;

   //70% on both channels
   pot = 1700;
   result = Throttle::CheckDualThrottle(&pot, 3300);
   ASSERT(result == true);

   //30% on 1st, 20% on second
   pot = 1300;
   result = Throttle::CheckDualThrottle(&pot, 3800);
   ASSERT(result == true);

   //30% on 1st, 19% on second
   pot = 1300;
   result = Throttle::CheckDualThrottle(&pot, 3810);
   ASSERT(result == false);

   //30% on 1st, 41% on second
   pot = 1300;
   result = Throttle::CheckDualThrottle(&pot, 3590);
   ASSERT(result == false);

   //300% on 1st (stuck at max), 100% on second
   pot = 4000;
   result = Throttle::CheckDualThrottle(&pot, 3000);
   ASSERT(result == false);
   ASSERT(pot == Throttle::potmin[0]);

   Throttle::potmin[1] = 3000;
   Throttle::potmax[1] = 4000;
}
#endif
void ThrottleTest::RunTest()
{
   TestSetup();
   TestBrkPedal();
   TestRegen();
   //TestDualThrottle();
}
