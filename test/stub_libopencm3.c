/*
 * This file is part of the stm32-sine project.
 *
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
#include "stdint.h"

void flash_unlock(void)
{
}

void flash_lock(void)
{
}

void flash_set_ws(uint32_t ws)
{
}

void flash_program_word(uint32_t address, uint32_t data)
{
}

void flash_erase_page(uint32_t page_address)
{
}

uint16_t desig_get_flash_size(void)
{
    return 8;
}

uint32_t crc_calculate(uint32_t data)
{
    return 0xaa55;
}
