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
#include "stdbool.h"
#include <stdint.h>
#include "delay.h"

#define REQ_ADC_EXT 0x4001
#define REQ_STAT1   0x2800
#define REQ_STAT2   0x3000
#define REQ_STAT3   0x3800

/// SPI channels
enum SpiChannelMG
{
	MGSPI1_H = 1,
	MGSPI2_H = 2,
	MGSPI3_H = 3,
	MGSPI1_L = 4,
	MGSPI2_L = 5,
	MGSPI3_L = 6,
};

extern uint32_t Send16 (enum SpiChannelMG channel,uint16_t txData16);
extern uint32_t Config_MG (enum SpiChannelMG channel);
