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
#ifndef TESLAM3GATEDRIVER_H
#define TESLAM3GATEDRIVER_H

#include <stdint.h>

class TeslaM3GateDriver
{
public:
    static bool Init();
    static bool IsFaulty();

private:
    enum ChipMask
    {
        All = 0x3F,
        Odd = 0x15,
        Even = 0x2A
    };
    struct Register
    {
        uint8_t  reg;
        uint8_t  value;
        ChipMask mask;
    };

private:
    static const uint8_t  NumDriverChips = 6;
    static const Register GateDriverRegisterSetup[];
    static const uint8_t  RegisterSetupSize;
    static const Register NullGateDriverRegister;

private:
    static void InitSPIPort();
    static void SetupGateDrivers();
    static bool VerifyGateDriverConfig();

    static void SendCommand(uint8_t cmd);
    static void WriteRegister(const Register& reg);
    static bool VerifyRegister(
        const Register& readReg,
        const Register& verifyValue);
};

#endif // TESLAM3GATEDRIVER_H
