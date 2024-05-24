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
#include "stub_canhardware.h"

CanCallback* vcuCan = nullptr;
uint32_t vcuCanId;

CanHardware::CanHardware()
{}

bool CanHardware::AddCallback(CanCallback* cb)
{
   vcuCan = cb;
   return true;
}

bool CanHardware::RegisterUserMessage(uint32_t canId, uint32_t mask)
{
   vcuCanId = canId;
   return true;
}

void CanHardware::ClearUserMessages() {}

void CanHardware::HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc)
{
   vcuCan->HandleRx(canId, data, dlc);
}
