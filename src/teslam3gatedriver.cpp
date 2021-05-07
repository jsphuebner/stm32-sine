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

#include "teslam3gatedriver.h"
#include "crc8.h"
#include "delay.h"
#include "digio.h"
#include "hwdefs.h"
#include "hw/stgap1as_gate_driver.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

/**
 * \brief STGAP1AS gate driver register set up sequence
 *
 *  The register set up sequence for each of 6 chips on the Tesla Model 3
 * Inverter. Settings are applied to all chips or high/low-side drivers as
 * required
 */
const TeslaM3GateDriver::Register TeslaM3GateDriver::GateDriverRegisterSetup[]{
    { STGAP1AS_REG_CFG1,
      STGAP1AS_REG_CFG1_CRC_SPI | STGAP1AS_REG_CFG1_SD_FLAG |
          STGAP1AS_REG_CFG1_DT_800NS | STGAP1AS_REG_CFG1_IN_FILTER_500NS,
      TeslaM3GateDriver::ChipMask::All },
    { STGAP1AS_REG_CFG2,
      STGAP1AS_REG_CFG2_DESAT_CUR_500UA | STGAP1AS_REG_CFG2_DESAT_TH_8V,
      TeslaM3GateDriver::ChipMask::All },
    { STGAP1AS_REG_CFG3,
      STGAP1AS_REG_CFG3_2LTO_TH_10V | STGAP1AS_REG_CFG3_2LTO_TIME_DISABLED,
      TeslaM3GateDriver::ChipMask::All },
    { STGAP1AS_REG_CFG4,
      STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_NEG_3V |
          STGAP1AS_REG_CFG4_VHON_TH_12V,
      TeslaM3GateDriver::ChipMask::Odd },
    { STGAP1AS_REG_CFG4,
      STGAP1AS_REG_CFG4_UVLO_LATCHED | STGAP1AS_REG_CFG4_VLON_TH_DISABLED |
          STGAP1AS_REG_CFG4_VHON_TH_12V,
      TeslaM3GateDriver::ChipMask::Even },
    { STGAP1AS_REG_CFG5,
      STGAP1AS_REG_CFG5_2LTO_EN | STGAP1AS_REG_CFG5_DESAT_EN,
      TeslaM3GateDriver::ChipMask::All },
    { STGAP1AS_REG_DIAG1CFG,
      STGAP1AS_REG_DIAG1CFG_UVLOD_OVLOD | STGAP1AS_REG_DIAG1CFG_UVLOH_UVLOL |
          STGAP1AS_REG_DIAG1CFG_OVLOH_OVLOL |
          STGAP1AS_REG_DIAG1CFG_DESAT_SENSE | STGAP1AS_REG_DIAG1CFG_TSD,
      TeslaM3GateDriver::ChipMask::All },
    { STGAP1AS_REG_DIAG2CFG, 0, TeslaM3GateDriver::ChipMask::All }
};

const uint8_t TeslaM3GateDriver::RegisterSetupSize =
    sizeof(TeslaM3GateDriver::GateDriverRegisterSetup) /
    sizeof(GateDriverRegisterSetup[0]);

const TeslaM3GateDriver::Register TeslaM3GateDriver::NullGateDriverRegister{
    0,
    0,
    ChipMask::All
};

// Delays from STGAP1AS datasheet Table 6. DC operation electrical
// characteristics - SPI Section
static const int ResetStatusDelay = 50;   // uSec
static const int LocalRegReadDelay = 1;   // uSec
static const int RemoteRegReadDelay = 30; // uSec
static const int StartConfigDelay = 22;   // uSec
static const int StopConfigDelay = 5;     // uSec
static const int OtherCommandDelay = 1;   // uSec

/**
 * \brief Define a scoped SPI transaction that manually asserts the ~CS line for
 * the duration of a SPI transaction.
 */
struct SPITransaction
{
    SPITransaction()
    {
        gpio_clear(GPIO_GATE_CS_BANK, GPIO_GATE_CS);
    }

    ~SPITransaction()
    {
        gpio_set(GPIO_GATE_CS_BANK, GPIO_GATE_CS);
    }
};

/**
 * \brief Set up the isolated gate drivers
 *
 * \return bool - True if gate drivers successfully initialised and verified
 */
bool TeslaM3GateDriver::Init()
{
    // Disable the ~SD line before we configure anything
    DigIo::vtg_out.Set();
    InitSPIPort();
    SetupGateDrivers();
    return VerifyGateDriverConfig();
}

/**
 * \brief Check for a fault on all gate drivers
 *
 * \return bool - There is a fault on one or more gate drivers
 */
bool TeslaM3GateDriver::IsFaulty()
{
    // Request the first status register - ignore the first result
    VerifyRegister(
        { STGAP1AS_REG_STATUS1, 0, ChipMask::All }, NullGateDriverRegister);

    // All status registers should be zero for correct normal operation
    bool result = VerifyRegister(
        { STGAP1AS_REG_STATUS2, 0, ChipMask::All }, NullGateDriverRegister);

    result = result && VerifyRegister(
                           { STGAP1AS_REG_STATUS3, 0, ChipMask::All },
                           NullGateDriverRegister);

    // Request STATUS3 twice to get the value
    result = result && VerifyRegister(
                           { STGAP1AS_REG_STATUS3, 0, ChipMask::All },
                           NullGateDriverRegister);

    // Invert to indicate a fault if the verifications fail
    return !result;
}

/**
 * \brief Initialise the SPI port and associated clocks and GPIO ports
 */
void TeslaM3GateDriver::InitSPIPort()
{
    rcc_periph_clock_enable(RCC_GATE_SPI);

    // Assert the SPI ~CS line before we turn off the pull up to avoid glitches
    gpio_set(GPIO_GATE_CS_BANK, GPIO_GATE_CS);
    gpio_set_mode(
        GPIO_GATE_CS_BANK,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO_GATE_CS);

    // Configure the SPI hardware ports
    gpio_set_mode(
        GPIO_GATE_SPI_BANK,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_GATE_SPI_SCK | GPIO_GATE_SPI_MOSI);
    gpio_set_mode(
        GPIO_GATE_SPI_BANK,
        GPIO_MODE_INPUT,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_GATE_SPI_MISO);

    // SPI initialization;
    // We get 4.5MHz clock with a 72MHz system frequency (on SPI2 which has an
    // additional /2 for APB1 peripherals)
    // The STGAP1AS requires CPOL = 0, CPHA = 1 and 16-bit MSB transfers
    spi_reset(GATE_SPI);
    spi_init_master(
        GATE_SPI,
        SPI_CR1_BAUDRATE_FPCLK_DIV_8,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2,
        SPI_CR1_DFF_16BIT,
        SPI_CR1_MSBFIRST);
    spi_set_full_duplex_mode(GATE_SPI);

    // Enable software slave management so we have correct behaviour for the 6
    // daisy-chained gate driver chips
    spi_enable_software_slave_management(GATE_SPI);
    spi_set_nss_high(GATE_SPI);
    spi_set_unidirectional_mode(GATE_SPI);
    spi_enable(GATE_SPI);
}

/**
 * \brief Run through the set up sequence for all gate driver chips
 */
void TeslaM3GateDriver::SetupGateDrivers()
{
    SendCommand(STGAP1AS_CMD_RESET_STATUS);
    uDelay(ResetStatusDelay);
    SendCommand(STGAP1AS_CMD_START_CONFIG);
    uDelay(StartConfigDelay);

    for (uint8_t i = 0; i < RegisterSetupSize; i++)
    {
        WriteRegister(GateDriverRegisterSetup[i]);
        uDelay(OtherCommandDelay);
    }

    SendCommand(STGAP1AS_CMD_STOP_CONFIG);
    uDelay(StopConfigDelay);
}

/**
 * \brief Verify the configuration has been correctly set up
 *
 * \return bool - true if the configuration has been verified
 */
bool TeslaM3GateDriver::VerifyGateDriverConfig()
{
    // Ask for the first register to verify we don't actually care about the
    // result as it will be invalid
    VerifyRegister(GateDriverRegisterSetup[0], NullGateDriverRegister);
    uDelay(RemoteRegReadDelay);

    bool result = true;
    for (uint8_t i = 1; i < RegisterSetupSize; i++)
    {
        // Request the next register read whilst verifying the value we were
        // requested last time
        result = result && VerifyRegister(
                               GateDriverRegisterSetup[i],
                               GateDriverRegisterSetup[i - 1]);
        uDelay(RemoteRegReadDelay);
    }

    // Verify the final register
    result = result && VerifyRegister(
                           NullGateDriverRegister,
                           GateDriverRegisterSetup[RegisterSetupSize - 1]);

    return result;
}

/**
 * \brief Send a command to all driver chips
 *
 * \param cmd STGAP1AS_CMD_xx command to send
 */
void TeslaM3GateDriver::SendCommand(uint8_t cmd)
{
    uint8_t  crc = ~crc8(STGAP1AS_SPI_CRC_INIT_VALUE, cmd);
    uint16_t cmdOut = cmd << 8 | crc;

    SPITransaction transaction;
    for (int i = 0; i < NumDriverChips; i++)
    {
        spi_xfer(GATE_SPI, cmdOut);
    }
}

/**
 * \brief Write a specific register
 *
 * \param reg Register structure detailing the register, value and required
 * chips
 */
void TeslaM3GateDriver::WriteRegister(const Register& reg)
{
    uint16_t cmd = STGAP1AS_CMD_WRITE_REG(reg.reg);
    uint8_t  cmdCrc = crc8(cmd, STGAP1AS_SPI_CRC_INIT_VALUE);
    uint8_t  dataCrc = crc8(reg.value, cmdCrc);

    // Invert the CRC as required by the STGAP1AS SPI protocol
    cmdCrc = ~cmdCrc;
    dataCrc = ~dataCrc;

    cmd = cmd << 8 | cmdCrc;

    const uint16_t nop = STGAP1AS_CMD_NOP << 8 |
                         (~crc8(STGAP1AS_CMD_NOP, STGAP1AS_SPI_CRC_INIT_VALUE));

    // Send the register write command ignoring any response (which is
    // undefined)
    {
        SPITransaction transaction;

        uint8_t mask = 1;
        for (int i = 0; i < NumDriverChips; i++)
        {
            if (reg.mask & mask)
            {
                spi_xfer(GATE_SPI, cmd);
            }
            else
            {
                spi_xfer(GATE_SPI, nop);
            }
            mask = mask << 1;
        }
    }

    uDelay(OtherCommandDelay);

    // Send the register data to be written ignoring any response (which is
    // undefined)
    {
        SPITransaction transaction;

        uint16_t data = reg.value << 8 | dataCrc;
        uint8_t  mask = 1;
        for (int i = 0; i < NumDriverChips; i++)
        {
            if (reg.mask & mask)
            {
                spi_xfer(GATE_SPI, data);
            }
            else
            {
                spi_xfer(GATE_SPI, nop);
            }
            mask = mask << 1;
        }
    }
}

/**
 * \brief Read back a register and verify against an expected value
 *
 * \param readReg Register we want to read back for later verification
 * \param verifyValue Register value to verify (was requested previously)
 *
 * \return True if value retrieved matches the expected value
 */
bool TeslaM3GateDriver::VerifyRegister(
    const Register& readReg,
    const Register& verifyValue)
{
    uint16_t cmd = STGAP1AS_CMD_READ_REG(readReg.reg);
    uint8_t  cmdCrc = ~crc8(cmd, STGAP1AS_SPI_CRC_INIT_VALUE);
    cmd = cmd << 8 | cmdCrc;

    uint16_t value = verifyValue.value;
    uint8_t  valueCrc = crc8(value, STGAP1AS_SPI_CRC_INIT_VALUE);
    value = value << 8 | valueCrc;

    bool result = true;

    {
        SPITransaction transaction;

        uint8_t mask = 1;
        for (int i = 0; i < NumDriverChips; i++)
        {
            // Always send the same command to all chips as reading is harmless
            uint16_t readValue = spi_xfer(GATE_SPI, cmd);

            // Check if we are to verify this value
            if (verifyValue.mask & mask)
            {
                result = result && (readValue == value);
            }
            mask = mask << 1;
        }
    }
    return result;
}
