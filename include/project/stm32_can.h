/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef STM32_CAN_H_INCLUDED
#define STM32_CAN_H_INCLUDED
#include "params.h"

#define CAN_ERR_INVALID_ID -1
#define CAN_ERR_INVALID_OFS -2
#define CAN_ERR_INVALID_LEN -3
#define CAN_ERR_MAXMESSAGES -4
#define CAN_ERR_MAXITEMS -5

namespace Can
{
   enum baudrates
   {
      Baud250, Baud500, Baud800, Baud1000, BaudLast
   };

   void Clear(void);
   void Init(enum baudrates baudrate);
   void SetBaudrate(enum baudrates baudrate);
   void Send(uint32_t canId, uint32_t data[2]);
   void SendAll();
   void Save();
   uint32_t GetLastRxTimestamp();
   int AddSend(Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain);
   int AddRecv(Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain);
   int Remove(Param::PARAM_NUM param);
   bool FindMap(Param::PARAM_NUM param, int& canId, int& offset, int& length, s32fp& gain, bool& rx);
}


#endif
