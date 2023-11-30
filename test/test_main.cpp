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
#include <iostream>
#include <list>
#include "test.h"

using namespace std;

int _failedAssertions = 0;
static int testIdx = 0;
static list<UnitTest*> testList;

int main()
{
   cout << "Starting unit Tests" << endl;

   for (UnitTest* currentTest: testList)
   {
      currentTest->TestSetup();

      for (VoidFunction testCase: currentTest->GetCases())
      {
         currentTest->TestCaseSetup();
         testCase();
      }
   }

   if (_failedAssertions > 0)
   {
      cout << _failedAssertions << " assertions failed" << endl;
      return -1;
   }

   cout << "All tests passed" << endl;

   return 0;
}

UnitTest::UnitTest(const list<VoidFunction>* cases)
: _cases(cases)
{
   testList.push_back(this);
}
