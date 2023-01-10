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
#ifndef PWMGENERATION_H
#define PWMGENERATION_H

#include <stdint.h>
#include "my_fp.h"
#include "anain.h"

class PwmGeneration
{
   public:
      static void Run();
      static uint16_t GetAngle();
      static bool Tripped();
      static void SetOpmode(int opmode);
      static void SetAmpnom(s32fp amp);
      static void SetFslip(s32fp fslip);
      static void SetTorquePercent(float torque);
      static void SetCurrentOffset(int offset1, int offset2);
      static void SetCurrentLimitThreshold(s32fp ocurlim);
      static void SetControllerGains(int kp, int ki);
      static int GetCpuLoad();
      static void SetChargeCurrent(float cur);
      static void SetPolePairRatio(int ratio) { polePairRatio = ratio; }

   private:
      enum EdgeType { NoEdge, PosEdge, NegEdge };

      static void PwmInit();
      static void EnableOutput();
      static void DisableOutput();
      static void EnableChargeOutput();
      static uint16_t TimerSetup(uint16_t deadtime, bool activeLow);
      static void AcHeatTimerSetup();
      static s32fp ProcessCurrents();
      static s32fp ProcessCurrents(s32fp& id, s32fp& iq);
      static void CalcNextAngleSync();
      static void CalcNextAngleAsync(int dir);
      static void CalcNextAngleConstant(int dir);
      static void Charge();
      static void AcHeat();
      static s32fp GetIlMax(s32fp il1, s32fp il2);
      static s32fp GetCurrent(AnaIn& input, s32fp offset, s32fp gain);
      static s32fp LimitCurrent();
      static EdgeType CalcRms(s32fp il, EdgeType& lastEdge, s32fp& max, s32fp& rms, int& samples, s32fp prevRms);
      static void RunOffsetCalibration();

      static uint16_t pwmfrq;
      static uint16_t angle;
      static s32fp ampnom;
      static int16_t slipIncr;
      static s32fp fslip;
      static s32fp frq;
      static s32fp frqFiltered;
      static uint8_t shiftForTimer;
      static int opmode;
      static s32fp ilofs[2];
      static int polePairRatio;
};

#endif // PWMGENERATION_H
