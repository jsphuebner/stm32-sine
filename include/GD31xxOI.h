/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 * Copyright (C) 2019 Nail Guzel
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
 *
 * Simplified library for interfacing with the NXP GD3100 gate drivers
 * found on the MG inverter platform.
 */

#include "stddef.h"
#include <stdint.h>
#include "delay.h"
#include "digio.h"

class MGSPI {

public:
   static void Initialize();
   static void CyclicFunction();
   static uint16_t GetRawTemperature(int index) { return temps[index]; }
   static uint16_t GetUdc() { return udc; }

private:
   static uint16_t temps[3];
   static uint16_t udc;

   static uint32_t Send16(DigIo& cspin,uint16_t txData16);
   static uint32_t ConfigureGateDriver(DigIo& cspin);
};
