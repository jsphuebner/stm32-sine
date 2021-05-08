/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
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
#ifndef TESLAM3PMIC_H
#define TESLAM3PMIC_H

#include <stdint.h>

class TeslaM3PowerWatchdog
{
public:
    enum Error
    {
        OK = 0,
        WriteFail,
        ReadParityFail,
        StateTransitionFail
    };

public:
    static Error Init();

    static Error Strobe();

private:
    static void  InitSPIPort();
    static Error SetupPowerManagement();

    static Error WriteRegister(uint8_t reg, uint8_t value);
    static Error ReadRegister(uint8_t reg, uint8_t& value);
};

#endif // TESLAM3PMIC_H
