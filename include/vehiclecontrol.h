/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef VCU_H
#define VCU_H
#include "canhardware.h"
#include "errormessage.h"

class VehicleControl
{
   public:
      /** Set CAN interface for VCU
       * \param val Pointer to CAN interface
       */
      static void SetCan(CanHardware* val);
      static void PostErrorIfRunning(ERROR_MESSAGE_NUM err);
      static void CruiseControl();
      static void SelectDirection();
      static void GetDigInputs();
      static void CalcAndOutputTemp();
      static float ProcessUdc();
      static float ProcessThrottle();
      static void SetContactorsOffState();

   private:
      static CanHardware* can; //!< Member variable "can"
      static bool lastCruiseSwitchState;
      static bool canIoActive;
      static bool spiEnabled;
      static float temphsFiltered;
      static float tempmFiltered;
      static int udcFiltered;
      static uint16_t bmwAdcNextChan;
      static uint16_t bmwAdcValues[4];
      static FunctionPointerCallback callback;
      static uint32_t lastCanRxTime;
      static uint8_t canErrors;
      static uint8_t seqCounter;

      static void GetTemps(float& tmphs, float &tmpm);
      static float GetUserThrottleCommand();
      static bool GetCruiseCreepCommand(float& finalSpnt, float throtSpnt);
      static void BmwAdcAcquire();
      static void CanClear();
      static bool CanReceive(uint32_t id, uint32_t data[2]);
};

#endif // VCU_H
