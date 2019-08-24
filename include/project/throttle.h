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

#ifndef THROTTLE_H
#define THROTTLE_H

#include "my_fp.h"

#define TMPHS_MAX FP_FROMINT(85)

class Throttle
{
   public:
      static bool CheckAndLimitRange(int* potval, int potIdx);
      static bool CheckDualThrottle(int* potval, int pot2val);
      static s32fp CalcThrottle(int potval, int pot2val, bool brkpedal);
      static s32fp CalcIdleSpeed(int speed);
      static s32fp CalcCruiseSpeed(int speed);
      static bool TemperatureDerate(s32fp tmphs, s32fp& finalSpnt);
      static void BmsLimitCommand(s32fp& finalSpnt, bool dinbms);
      static void UdcLimitCommand(s32fp& finalSpnt, s32fp udc);
      static void IdcLimitCommand(s32fp& finalSpnt, s32fp idc);
      static void FrequencyLimitCommand(s32fp& finalSpnt, s32fp frequency);
      static int potmin[2];
      static int potmax[2];
      static s32fp brknom;
      static s32fp brknompedal;
      static s32fp brkmax;
      static s32fp throtmax;
      static int idleSpeed;
      static int cruiseSpeed;
      static s32fp speedkp;
      static int speedflt;
      static s32fp idleThrotLim;
      static s32fp brkPedalRamp;
      static s32fp throttleRamp;
      static int bmslimhigh;
      static int bmslimlow;
      static int accelmax;
      static int accelflt;
      static s32fp udcmin;
      static s32fp udcmax;
      static s32fp idcmin;
      static s32fp idcmax;
      static s32fp fmax;

   private:
      static int speedFiltered;
      static s32fp potnomFiltered;
      static s32fp brkRamped;
      static s32fp throttleRamped;
};

#endif // THROTTLE_H
