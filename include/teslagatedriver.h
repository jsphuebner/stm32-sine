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
#ifndef TESLAGATEDRIVER_H
#define TESLAGATEDRIVER_H

#include "hw/stgap1as_gate_driver.h"
#include <assert.h>
#include <crc8.h>
#include <cstddef>
#include <stdint.h>
#include <string.h>

#ifndef DEVICE_DELAY_US
#error                                                                         \
    "Please define DEVICE_DELAY_US(x) as a macro which will delay the specified number of micro-seconds"
#endif

namespace tesla {

//
//! \brief Control the STGAP1AS gate driver chain on the Tesla M3 inverter
//!
//! \param SpiDriverT    SPI interface driver for the gate driver
//!
template <typename SpiDriverT>
class GateDriver
{
public:
    //! \brief Which phase a given gate driver chip corresponds to
    enum Phase
    {
        PhaseAHigh = 0,
        PhaseALow = 1,
        PhaseBHigh = 2,
        PhaseBLow = 3,
        PhaseCHigh = 4,
        PhaseCLow = 5
    };

    //! \brief Possible gate driver chip status bitfield values
    enum Status
    {
        Status_RxCRC = 0x0001,
        Status_Temp_Warn = 0x0002,
        Status_Temp_Shdn = 0x0004,
        Status_UVLOL = 0x0008,
        Status_UVLOH = 0x0010,
        Status_SENSE = 0x0020,
        Status_DESAT = 0x0040,
        Status_OVLOL = 0x0080,
        Status_OVLOH = 0x0100,
        Status_ASC = 0x0200,
        Status_REGERRR = 0x0400,
        Status_UVLOD = 0x0800,
        Status_OVLOD = 0x1000,
        Status_REGERRL = 0x2000,
        Status_SPI_ERR = 0x4000,
        Status_DT_ERR = 0x8000
    };

public:
    static bool     Init();
    static bool     IsFaulty();
    static void     Enable();
    static void     Disable();
    static uint16_t GetStatus(Phase chip);

private:
    enum ChipMask
    {
        All = 0x3F,
        LowSide = 0x15,
        HighSide = 0x2A
    };
    struct Register
    {
        uint16_t reg;
        uint16_t value;
        ChipMask mask;
        uint16_t validBitMask;
    };

private:
    static const uint16_t NumDriverChips = 6;
    static const Register GateDriverRegisterSetup[];
    static const uint16_t RegisterSetupSize;
    static const Register NullGateDriverRegister;

private:
    typedef uint16_t DataBuffer[NumDriverChips];
    typedef uint16_t DriverStatus[NumDriverChips];

    typedef uint16_t (*StatusParser)(uint16_t statusRegister);

private:
    static void SetupGateDrivers();
    static bool VerifyGateDriverConfig();

    static void SendCommand(uint16_t cmd);
    static void WriteRegister(const Register& reg);
    static void ReadRegister(const Register& reg, uint16_t* values);
    static void PipelinedReadRegister(uint16_t regNum, uint16_t* values);
    static bool VerifyRegister(const DataBuffer values, StatusParser parser);
    static uint16_t ParseStatus1(uint16_t statusRegister);
    static uint16_t ParseStatus2(uint16_t statusRegister);
    static uint16_t ParseStatus3(uint16_t statusRegister);
    static void     ClearStatus();

    static uint16_t InvertByte(uint16_t input);
    static uint16_t BuildCommand(uint16_t cmd);

private:
    static DriverStatus s_status;
};

//
//! \brief STGAP1AS gate driver register set up sequence
//!
//! The register set up sequence for each of 6 chips on the Tesla Model 3
//! Inverter. Settings are applied to all chips or high/low-side drivers as
//! required
//!
template <typename SpiDriverT>
const typename GateDriver<SpiDriverT>::Register
    GateDriver<SpiDriverT>::GateDriverRegisterSetup[] = {
        { STGAP1AS_REG_CFG1,
          STGAP1AS_REG_CFG1_CRC_SPI | STGAP1AS_REG_CFG1_SD_FLAG |
              STGAP1AS_REG_CFG1_DT_800NS | STGAP1AS_REG_CFG1_IN_FILTER_500NS,
          All,
          STGAP1AS_REG_CFG1_MASK },
        { STGAP1AS_REG_CFG2,
          STGAP1AS_REG_CFG2_DESAT_CUR_500UA | STGAP1AS_REG_CFG2_DESAT_TH_8V,
          All,
          STGAP1AS_REG_CFG2_MASK },
        { STGAP1AS_REG_CFG3,
          STGAP1AS_REG_CFG3_2LTO_TH_10V | STGAP1AS_REG_CFG3_2LTO_TIME_DISABLED,
          All,
          STGAP1AS_REG_CFG3_MASK },
        { STGAP1AS_REG_CFG4,
          STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_NEG_3V |
              STGAP1AS_REG_CFG4_VHON_TH_12V,
          LowSide,
          STGAP1AS_REG_CFG4_MASK },
        { STGAP1AS_REG_CFG4,
          STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_DISABLED |
              STGAP1AS_REG_CFG4_VHON_TH_12V,
          HighSide,
          STGAP1AS_REG_CFG4_MASK },
        { STGAP1AS_REG_CFG5,
          STGAP1AS_REG_CFG5_2LTO_EN | STGAP1AS_REG_CFG5_DESAT_EN,
          All,
          STGAP1AS_REG_CFG5_MASK },
        { STGAP1AS_REG_DIAG1CFG,
          STGAP1AS_REG_DIAG1CFG_UVLOD_OVLOD |
              STGAP1AS_REG_DIAG1CFG_UVLOH_UVLOL |
              STGAP1AS_REG_DIAG1CFG_OVLOH_OVLOL |
              STGAP1AS_REG_DIAG1CFG_DESAT_SENSE | STGAP1AS_REG_DIAG1CFG_TSD,
          All,
          STGAP1AS_REG_DIAG1CFG_MASK },
        { STGAP1AS_REG_DIAG2CFG, 0, All, STGAP1AS_REG_DIAG2CFG }
    };

template <typename SpiDriverT>
const uint16_t GateDriver<SpiDriverT>::RegisterSetupSize =
    sizeof(GateDriver<SpiDriverT>::GateDriverRegisterSetup) /
    sizeof(GateDriverRegisterSetup[0]);

// Delays from STGAP1AS datasheet Table 6. DC operation electrical
// characteristics - SPI Section
static const __attribute__((__unused__)) int ResetStatusDelay = 50;   // uSec
static const __attribute__((__unused__)) int LocalRegReadDelay = 1;   // uSec
static const __attribute__((__unused__)) int RemoteRegReadDelay = 30; // uSec
static const __attribute__((__unused__)) int StartConfigDelay = 22;   // uSec
static const __attribute__((__unused__)) int StopConfigDelay = 5;     // uSec
static const __attribute__((__unused__)) int OtherCommandDelay = 1;   // uSec

//
//! \brief Set up the isolated gate drivers
//!
//! \return bool - True if gate drivers successfully initialised and verified
//!
template <typename SpiDriverT>
bool GateDriver<SpiDriverT>::Init()
{
    SpiDriverT::Init();
    ClearStatus();
    SetupGateDrivers();
    if (VerifyGateDriverConfig())
    {
        return !IsFaulty();
    }
    else
    {
        return false;
    }
}

//
//! \brief Check for a fault on all gate drivers
//!
//! \return bool - There is a fault on one or more gate drivers
//!
template <typename SpiDriverT>
bool GateDriver<SpiDriverT>::IsFaulty()
{
    ClearStatus();

    PipelinedReadRegister(STGAP1AS_REG_STATUS1, NULL);

    DEVICE_DELAY_US(LocalRegReadDelay);
    DataBuffer values;
    PipelinedReadRegister(STGAP1AS_REG_STATUS2, values);

    bool status1 = VerifyRegister(values, ParseStatus1);

    DEVICE_DELAY_US(LocalRegReadDelay);
    PipelinedReadRegister(STGAP1AS_REG_STATUS3, values);

    bool status2 = VerifyRegister(values, ParseStatus2);

    DEVICE_DELAY_US(LocalRegReadDelay);
    PipelinedReadRegister(STGAP1AS_REG_STATUS3, values);

    bool status3 = VerifyRegister(values, ParseStatus3);

    return !(status1 && status2 && status3);
}

//
//! \brief Enable the gate drivers
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::Enable()
{
    // De-assert the gate driver shutdown line
    SpiDriverT::Resume();
}

//
//! \brief Disable the gate drivers
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::Disable()
{
    // Assert the gate driver shutdown line
    SpiDriverT::Shutdown();
}

//
//! \brief Return the detailed fault status of a specific gate driver chip
//!
//! \note The results of this function are only valid after a call to IsFaulty()
//!
//! \param chip The gate driver chip to query
//!
//! \return A bitfield of Status values indicating the fault(s) on this chip
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::GetStatus(Phase chip)
{
    return s_status[chip];
}

//
//! \brief Run through the set up sequence for all gate driver chips
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::SetupGateDrivers()
{
    SendCommand(STGAP1AS_CMD_RESET_STATUS);
    DEVICE_DELAY_US(ResetStatusDelay);

    SendCommand(STGAP1AS_CMD_START_CONFIG);
    DEVICE_DELAY_US(StartConfigDelay);

    for (uint16_t i = 0; i < RegisterSetupSize; i++)
    {
        WriteRegister(GateDriverRegisterSetup[i]);
        DEVICE_DELAY_US(OtherCommandDelay);
    }

    SendCommand(STGAP1AS_CMD_STOP_CONFIG);
    DEVICE_DELAY_US(StopConfigDelay);
}

//
//! \brief Verify the configuration has been correctly set up
//!
//! \return bool - true if the configuration has been verified
//!
template <typename SpiDriverT>
bool GateDriver<SpiDriverT>::VerifyGateDriverConfig()
{
    DataBuffer regValues;
    bool       result = true;
    for (uint16_t i = 0; i < RegisterSetupSize; i++)
    {
        const Register& reg = GateDriverRegisterSetup[i];
        ReadRegister(reg, regValues);
        DEVICE_DELAY_US(RemoteRegReadDelay);

        uint16_t mask = 1;
        for (int chip = 0; chip < NumDriverChips; chip++)
        {
            if (reg.mask & mask)
            {
                // Validate the CRC of the returned value. The "don't care"
                // bits in some registers can vary from read to read and are
                // included in the CRC. We can't precompute the CRC once and
                // compare.
                uint16_t value = regValues[chip] >> 8;
                uint16_t crc = regValues[chip] & 0xFF;

                uint16_t computedCrc = crc8(value, STGAP1AS_SPI_CRC_INIT_VALUE);

                // Mask off the "don't care" bits from the value before
                // comparing
                value = value & reg.validBitMask;

                result = result && (crc == computedCrc) && (value == reg.value);
            }
            mask = mask << 1;
        }
    }

    return result;
}

//
//! \brief Send a command to all driver chips
//!
//! \param cmd STGAP1AS_CMD_xx command to send
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::SendCommand(uint16_t cmd)
{
    DataBuffer cmdBuffer;
    for (uint16_t chip = 0; chip < NumDriverChips; chip++)
    {
        cmdBuffer[chip] = BuildCommand(cmd);
    }

    SpiDriverT::SendData(cmdBuffer, NULL);
}

//
//! \brief Write a specific register
//!
//! \param reg Register structure detailing the register, value and required
//! chips
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::WriteRegister(const Register& reg)
{
    uint16_t cmd = STGAP1AS_CMD_WRITE_REG(reg.reg);
    uint16_t cmdCrc = crc8(cmd, STGAP1AS_SPI_CRC_INIT_VALUE);
    uint16_t dataCrc = crc8(reg.value, cmdCrc);

    // Invert the CRC as required by the STGAP1AS SPI protocol
    cmdCrc = InvertByte(cmdCrc);
    dataCrc = InvertByte(dataCrc);

    cmd = cmd << 8 | cmdCrc;

    const uint16_t nop = BuildCommand(STGAP1AS_CMD_NOP);

    // Assemble a command buffer with all the commands or nops required for
    // each chip
    DataBuffer cmdBuffer;
    uint16_t   mask = 1;
    for (int chip = 0; chip < NumDriverChips; chip++)
    {
        cmdBuffer[chip] = reg.mask & mask ? cmd : nop;
        mask = mask << 1;
    }

    // Send the register write command ignoring any response (which is
    // undefined)
    SpiDriverT::SendData(cmdBuffer, NULL);

    DEVICE_DELAY_US(OtherCommandDelay);

    // Send the register data to be written ignoring any response (which is
    // undefined)
    DataBuffer cmdDataBuffer;

    uint16_t data = reg.value << 8 | dataCrc;
    mask = 1;
    for (int chip = 0; chip < NumDriverChips; chip++)
    {
        cmdDataBuffer[chip] = reg.mask & mask ? data : nop;
        mask = mask << 1;
    }

    SpiDriverT::SendData(cmdDataBuffer, NULL);
}

//
//! \brief Read a specific register
//!
//! \param regNum Register number to read
//! \param values Register values array retrieved from all chips
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::ReadRegister(const Register& reg, uint16_t* values)
{
    // Build a command  buffer  to read from the chips we are interested in and
    // ignore those we are not
    DataBuffer cmdBuffer;

    uint16_t cmd = BuildCommand(STGAP1AS_CMD_READ_REG(reg.reg));
    uint16_t nop = BuildCommand(STGAP1AS_CMD_NOP);

    uint16_t mask = 1;
    for (uint16_t chip = 0; chip < NumDriverChips; chip++)
    {
        cmdBuffer[chip] = reg.mask & mask ? cmd : nop;
        mask = mask << 1;
    }

    // Send the register read command. We are not interested in a data that
    // comes back until the next SPI transaction.
    SpiDriverT::SendData(cmdBuffer, NULL);

    // Pessimistic for local reg reads but we'll assume that's not
    // performance critical
    DEVICE_DELAY_US(RemoteRegReadDelay);

    // Send a NOP while reading the data back from the register
    DataBuffer nopBuffer;

    for (uint16_t i = 0; i < NumDriverChips; i++)
    {
        nopBuffer[i] = nop;
    }

    SpiDriverT::SendData(nopBuffer, values);
}

//
//! \brief Read a specific register and retrieve a previous read response
//!
//! \param regNum Register number to read
//! \param previousValues Array to store the results of the last read operation.
//! May be NULL if the results are not required.
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::PipelinedReadRegister(
    uint16_t  regNum,
    uint16_t* previousValues)
{
    // Send the register read command
    DataBuffer cmdBuffer;

    for (uint16_t i = 0; i < NumDriverChips; i++)
    {
        cmdBuffer[i] = BuildCommand(STGAP1AS_CMD_READ_REG(regNum));
    }

    SpiDriverT::SendData(cmdBuffer, previousValues);
}

//
//! \brief Verify the status of a given register
//!
//! \param values Array of data values read from the gate driver chips
//! \param parser Function to parse the specific register status bits
//!
//! \return True if the status matches the expected value
//!
template <typename SpiDriverT>
bool GateDriver<SpiDriverT>::VerifyRegister(
    const DataBuffer values,
    StatusParser     parser)
{
    assert(parser);

    bool result = true;
    for (int chip = 0; chip < NumDriverChips; chip++)
    {
        uint16_t actualValue = values[chip] >> 8;
        uint16_t actualCrc = values[chip] & 0xFF;

        uint16_t computedCrc = crc8(actualValue, STGAP1AS_SPI_CRC_INIT_VALUE);

        uint16_t& status = s_status[chip];
        if (computedCrc == actualCrc)
        {
            status |= parser(actualValue);
        }
        else
        {
            status |= Status_RxCRC;
        }

        if (status)
        {
            result = false;
        }
    }

    return result;
}

//
//! \brief Parse the STATUS1 register values retrieved from the gate drivers
//!
//! \param values The raw status register value read from STATUS1
//!
//! \return The new Status bitfield
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::ParseStatus1(uint16_t statusRegister)
{
    uint16_t status = 0;

    status |= statusRegister & STGAP1AS_REG_STATUS1_TWN ? Status_Temp_Warn : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_TSD ? Status_Temp_Shdn : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_UVLOL ? Status_UVLOL : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_UVLOH ? Status_UVLOH : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_SENSE ? Status_SENSE : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_DESAT ? Status_DESAT : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_OVLOL ? Status_OVLOL : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS1_OVLOH ? Status_OVLOH : 0;

    return status;
}

//
//! \brief Parse the STATUS2 register values retrieved from the gate drivers
//!
//! \param values The raw status register value read from STATUS2
//!
//! \return The new Status bitfield
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::ParseStatus2(uint16_t statusRegister)
{
    uint16_t status = 0;

    status |= statusRegister & STGAP1AS_REG_STATUS2_ASC ? Status_ASC : 0;
    status |=
        statusRegister & STGAP1AS_REG_STATUS2_REGERRR ? Status_REGERRR : 0;

    return status;
}

//
//! \brief Parse the STATUS3 register values retrieved from the gate drivers
//!
//! \param values The raw status register value read from STATUS3
//!
//! \return The new Status bitfield
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::ParseStatus3(uint16_t statusRegister)
{
    uint16_t status = 0;

    status |= statusRegister & STGAP1AS_REG_STATUS3_UVLOD ? Status_UVLOD : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS3_OVLOD ? Status_OVLOD : 0;
    status |=
        statusRegister & STGAP1AS_REG_STATUS3_REGERRL ? Status_REGERRL : 0;
    status |=
        statusRegister & STGAP1AS_REG_STATUS3_SPI_ERR ? Status_SPI_ERR : 0;
    status |= statusRegister & STGAP1AS_REG_STATUS3_DT_ERR ? Status_DT_ERR : 0;

    return status;
}

//
//! \brief Clear the status for each gate driver chip
//!
template <typename SpiDriverT>
void GateDriver<SpiDriverT>::ClearStatus()
{
    memset(s_status, 0, sizeof(s_status));
}

//
//! \brief Invert and mask off the lower 8 bits only
//!
//! \return The inverted value
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::InvertByte(uint16_t input)
{
    return ~input & 0xFF;
}

//
//! \brief Build a command with it's CRC
//!
//! \return The complete command
//!
template <typename SpiDriverT>
uint16_t GateDriver<SpiDriverT>::BuildCommand(uint16_t cmd)
{
    return cmd << 8 | InvertByte(crc8(STGAP1AS_SPI_CRC_INIT_VALUE, cmd));
}

// instance of the status array
template <typename SpiDriverT>
typename GateDriver<SpiDriverT>::DriverStatus GateDriver<SpiDriverT>::s_status;

} // namespace tesla

#endif // TESLAGATEDRIVER_H
