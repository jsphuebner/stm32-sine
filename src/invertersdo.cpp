/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2025 Johannes Huebner <dev@johanneshuebner.com>
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
#include "invertersdo.h"
#include "sdocommands.h"

#define START_COMMAND_SUBINDEX 4
#define STOP_COMMAND_SUBINDEX 5

InverterSdo::InverterSdo(CanHardware* hw, CanMap* cm)
:  CanSdo(hw, cm)
{

}

bool InverterSdo::ProcessUserSpaceSdo(SdoFrame* sdoFrame)
{
   bool processed = false;

   if (sdoFrame->index == SDO_INDEX_COMMANDS && sdoFrame->cmd == SDO_WRITE)
   {
      switch (sdoFrame->subIndex)
      {
      case START_COMMAND_SUBINDEX:
         sdoFrame->cmd = SDO_WRITE_REPLY;
         if (sdoFrame->data < MOD_LAST)
            Param::SetInt(Param::opmode, sdoFrame->data);
         else
         {
            sdoFrame->cmd = SDO_ABORT;
            sdoFrame->data = SDO_ERR_RANGE;
         }
         processed = true;
         break;
      case STOP_COMMAND_SUBINDEX:
         Param::SetInt(Param::opmode, 0);
         sdoFrame->cmd = SDO_WRITE_REPLY;
         processed = true;
         break;
      }
   }
   return processed;
}
