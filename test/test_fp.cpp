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
#include "sine_core.h"
#include "test.h"
#include "string.h"

class FPTest: public UnitTest
{
   public:
      FPTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static void TestMacros()
{
   ASSERT(FP_MUL(FP_FROMFLT(5.5), FP_FROMFLT(2.03125)) == FP_FROMFLT(5.5 * 2.03125));
   ASSERT(FP_MUL(FP_FROMFLT(5.5), FP_FROMFLT(2.03225)) == FP_FROMFLT(5.5 * 2.03125));
   ASSERT(FP_DIV(FP_FROMFLT(5.5), FP_FROMFLT(2.03125)) == FP_FROMFLT(5.5 / 2.03125));
}

static void TestItoa()
{
   char buf[10];
   ASSERT(strcmp(fp_itoa(buf, FP_FROMFLT(2.03125)), "2.03") == 0);
   ASSERT(strcmp(fp_itoa(buf, FP_FROMFLT(-2.125)), "-2.12") == 0);
   ASSERT(strcmp(fp_itoa(buf, FP_FROMFLT(2.15624)), "2.12") == 0);
   ASSERT(strcmp(fp_itoa(buf, FP_FROMFLT(2.15625)), "2.15") == 0);
}

static void TestAtoi()
{
   ASSERT(fp_atoi("-2.5", 5) == FP_FROMFLT(-2.5));
   ASSERT(fp_atoi("2.155", 5) == FP_FROMFLT(2.16));
}

static void TestMedian3()
{
   ASSERT(MEDIAN3(1,2,3) == 2);
   ASSERT(MEDIAN3(3,2,1) == 2);
   ASSERT(MEDIAN3(1,3,2) == 2);
   ASSERT(MEDIAN3(2,3,1) == 2);
   ASSERT(MEDIAN3(2,1,3) == 2);
}

static void TestAtan2()
{
   uint16_t res;
   ASSERT(SineCore::Atan2(4096, 0) == 0); //0°
   ASSERT(SineCore::Atan2(2896, 2896) == 8192); //45°
   ASSERT(SineCore::Atan2(-4096, 0) == 32768); //180°
   ASSERT(SineCore::Atan2(2048, -3547) == 54613); //300°
   ASSERT(SineCore::Atan2(2048, 3547) == 10922); //60°
}

//This line registers the test
REGISTER_TEST(FPTest, TestMacros, TestItoa, TestAtoi, TestMedian3, TestAtan2);


