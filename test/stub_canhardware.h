/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
 * Copyright (C) 2024 David J. Fiddes <D.J@fiddes.net>
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
#ifndef TEST_CANHARDWARE_H
#define TEST_CANHARDWARE_H

#include "canhardware.h"
#include <stdint.h>
#include <string.h>
#include <array>

class CanStub: public CanHardware
{
   void SetBaudrate(enum baudrates baudrate) {}
   void Send(uint32_t canId, uint32_t data[2], uint8_t len)
   {
      m_canId = canId;
      memcpy(&m_data[0], &data[0], sizeof(m_data));
      m_len = len;
   }
   virtual void ConfigureFilters() {}

public:
   std::array<uint8_t, 8>  m_data;
   uint8_t                 m_len;
   uint32_t                m_canId;
};

extern CanCallback* vcuCan;
extern uint32_t vcuCanId;

#endif // TEST_CANHARDWARE_H