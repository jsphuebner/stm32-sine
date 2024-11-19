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
REGISTER_TEST(FPTest, TestAtan2);


