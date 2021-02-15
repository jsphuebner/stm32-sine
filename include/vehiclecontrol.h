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
#include "stm32_can.h"
#include "errormessage.h"

class VehicleControl
{
   public:
      /** Set CAN interface for VCU
       * \param val Pointer to CAN interface
       */
      static void SetCan(Can* val) { can = val; }
      static void PostErrorIfRunning(ERROR_MESSAGE_NUM err);
      static void CruiseControl();
      static void SelectDirection();
      static void GetDigInputs();
      static void CalcAndOutputTemp();
      static s32fp ProcessUdc();
      static s32fp ProcessThrottle();
      static void SetContactorsOffState();

   protected:

   private:
      static Can* can; //!< Member variable "can"
      static bool lastCruiseSwitchState;
      static bool canIoActive;
      static int temphsFiltered;
      static int tempmFiltered;
      static int udcFiltered;

      static void GetTemps(s32fp& tmphs, s32fp &tmpm);
      static s32fp GetUserThrottleCommand();
      static void GetCruiseCreepCommand(s32fp& finalSpnt, s32fp throtSpnt);
};

#endif // VCU_H