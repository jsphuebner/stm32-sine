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

#include "fu.h"
#include "my_fp.h"
#include "test.h"

class FUTest: public UnitTest
{
   public:
      FUTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static void Setup(float fweak, int boost)
{
   MotorVoltage::SetMaxAmp(10000);
   MotorVoltage::SetBoost(boost);
   MotorVoltage::SetWeakeningFrq(fweak);
}

static void TestBoost1()
{
   Setup(10, 1000);
   ASSERT(MotorVoltage::GetAmp(FP_FROMINT(1))==1900);
}

static void TestBoost2()
{
   Setup(10, 1000);
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(0.1))==0);
}

static void TestFU1()
{
   Setup(10, 0);
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(5))==(5.0/10.0f * 10000));
}

static void TestFU2()
{
   Setup(10, 1000);
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(5))==(1000 + 5.0/10.0f * (10000-1000)));
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(9.5))==(1000 + 9.5/10.0f * (10000-1000)));
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(10))==10000);
   ASSERT(MotorVoltage::GetAmp(FP_FROMFLT(100))==10000);
}

static void TestFUPerc()
{
   Setup(10, 1000);
   ASSERT(MotorVoltage::GetAmpPerc(FP_FROMFLT(5), FP_FROMFLT(50))==((1000 + 5.0/10.0f * (10000-1000))/2));
   ASSERT(MotorVoltage::GetAmpPerc(FP_FROMFLT(22), FP_FROMFLT(50))==10000);
}

//This line registers the test
REGISTER_TEST(FUTest, TestBoost1, TestBoost2, TestFU1, TestFU2, TestFUPerc);

