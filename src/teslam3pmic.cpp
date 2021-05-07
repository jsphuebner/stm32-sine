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

#include "teslam3pmic.h"
#include "delay.h"
#include "hwdefs.h"
#include "hw/tlf35584_safety_psu.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

struct Register
{
    uint8_t reg;
    uint8_t value;
};

/**
 * \brief TLF35584 safety power supply and watchdog register set up sequence
 *
 */
static const Register RegisterConfig[]{
    { TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY1 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY2 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY3 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_UNLOCK_KEY4 },
    { TLF35584_WDCFG1, TLF35584_WDCFG1_WDSLPEN | TLF35584_WDCFG1_FWDETHR(14) },
    { TLF35584_WDCFG0,
      TLF35584_WDCFG0_WWDETHR(14) | TLF35584_WDCFG0_WWDEN |
          TLF35584_WDCFG0_FWDEN | TLF35584_WDCFG0_WWDTSEL |
          TLF35584_WDCFG0_WDCYC_1MS },
    { TLF35584_SYSPCFG1,
      TLF35584_SYSPCFG1_SS2DEL_0MS | TLF35584_SYSPCFG1_ERRREC_1MS },
    { TLF35584_FWDCFG, TLF35584_FWDCFG_WDHBTP_CYCLES(250) },
    { TLF35584_WWDCFG0, TLF35584_WWDCFG0_CW_CYCLES(50) },
    { TLF35584_WWDCFG1, TLF35584_WWDCFG1_OW_CYCLES(100) },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY1 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY2 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY3 },
    { TLF35584_PROTCFG, TLF35584_PROTCFG_LOCK_KEY4 }
};

static const uint8_t RegisterConfigSize =
    sizeof(RegisterConfig) / sizeof(RegisterConfig[0]);

/**
 * Functional Watchdog response sequence structure
 */
struct WatchdogResponse
{
    uint8_t resp3;
    uint8_t resp2;
    uint8_t resp1;
    uint8_t resp0;
};

/**
 * Canned Functional Watchdog response sequences from Table 26 in Section 15.3
 * functional Watchdog of the TLF35584 datasheet
 */
static const WatchdogResponse WatchdogResponses[]{
    // clang-format off
    { 0xFF, 0x0F, 0xF0, 0x00 },
    { 0xB0, 0x40, 0xBF, 0x4F },
    { 0xE9, 0x19, 0xE6, 0x16 },
    { 0xA6, 0x56, 0xA9, 0x59 },
    { 0x75, 0x85, 0x7A, 0x8A },
    { 0x3A, 0xCA, 0x35, 0xC5 },
    { 0x63, 0x93, 0x6C, 0x9C },
    { 0x2C, 0xDC, 0x23, 0xD3 },
    { 0xD2, 0x22, 0xDD, 0x2D },
    { 0x9D, 0x6D, 0x92, 0x62 },
    { 0xC4, 0x34, 0xCB, 0x3B },
    { 0x8B, 0x7B, 0x84, 0x74 },
    { 0x58, 0xA8, 0x57, 0xA7 },
    { 0x17, 0xE7, 0x18, 0xE8 },
    { 0x4E, 0xBE, 0x41, 0xB1 },
    { 0x01, 0xF1, 0x0E, 0xFE },
    // clang-format on
};

static const uint8_t WatchdogResponsesSize =
    sizeof(WatchdogResponses) / sizeof(WatchdogResponses[0]);

static const int StateTransitionDelay = 100; // uS

/**
 * \brief Verify that function x didn't fail. Return immediately if it did.
 * Assumes a local variable "result" of type TeslaM3PowerWatchdog::Error
 */
#define CHECK(x)                                                               \
    if ((result = x) != Error::OK)                                             \
    {                                                                          \
        return result;                                                         \
    }

/**
 * \brief Define a scoped SPI transaction that manually asserts the ~CS line for
 * the duration of a SPI transaction.
 */
struct SPITransaction
{
    SPITransaction()
    {
        gpio_clear(GPIO_PMIC_CS_BANK, GPIO_PMIC_CS);
    }

    ~SPITransaction()
    {
        gpio_set(GPIO_PMIC_CS_BANK, GPIO_PMIC_CS);
    }
};

/**
 * \brief Set up the power management watchdog
 *
 * \return Error - Error code if initialisation failed otherwise Error::OK
 */
TeslaM3PowerWatchdog::Error TeslaM3PowerWatchdog::Init()
{
    InitSPIPort();
    return SetupPowerManagement();
}

/**
 * \brief Strobe the watchdog
 *
 * \return Error - Error code if stobe failed otherwise Error::OK
 */
TeslaM3PowerWatchdog::Error TeslaM3PowerWatchdog::Strobe()
{
    Error result;

    uint8_t windowStatus;
    CHECK(ReadRegister(TLF35584_WWDSCMD, windowStatus));

    // Invert the window watchdog status and write back in the lowest bit
    CHECK(WriteRegister(
        TLF35584_WWDSCMD,
        windowStatus & TLF35584_WWDSCMD_TRIG_STATUS ? 0 :
                                                      TLF35584_WWDSCMD_TRIG));

    uint8_t functionalStatus;
    CHECK(ReadRegister(TLF35584_FWDSTAT0, functionalStatus));

    // Determine which response the functional watchdog is expecting from us
    const WatchdogResponse& response =
        WatchdogResponses[functionalStatus & TLF35584_FWDSTAT0_FWDQUEST_MASK];

    CHECK(WriteRegister(TLF35584_FWDRSP, response.resp3));
    CHECK(WriteRegister(TLF35584_FWDRSP, response.resp2));
    CHECK(WriteRegister(TLF35584_FWDRSP, response.resp1));
    CHECK(WriteRegister(TLF35584_FWDRSPSYNC, response.resp0));

    return Error::OK;
}

/**
 * \brief Initialise the SPI port and associated clocks and GPIO ports
 */
void TeslaM3PowerWatchdog::InitSPIPort()
{
    rcc_periph_clock_enable(RCC_SPI2);

    // Assert the SPI ~CS enable line before we turn off the
    // pull up to avoid glitches
    gpio_set(GPIO_PMIC_CS_BANK, GPIO_PMIC_CS);
    gpio_set_mode(
        GPIO_PMIC_CS_BANK,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO_PMIC_CS);

    // Configure the SPI hardware ports
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI1_REMAP);
    gpio_set_mode(
        GPIO_PMIC_SPI_BANK,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_PMIC_SPI_SCK | GPIO_PMIC_SPI_MOSI);
    gpio_set_mode(
        GPIO_PMIC_SPI_BANK, GPIO_MODE_INPUT, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_PMIC_SPI_MISO);

    // SPI initialization;
    // We get 2.25MHz clock with a 72MHz system frequency. Tesla run the device at 2.4MHz so we
    // also run slower than rated 5MHz to be on the safe side. The TLF35584
    // requires CPOL = 0, CPHA = 0 and 16-bit MSB transfers with software
    // controlled chip-select control around each transfer.
    spi_reset(PMIC_SPI);
    spi_init_master(
        PMIC_SPI,
        SPI_CR1_BAUDRATE_FPCLK_DIV_32,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_DFF_16BIT,
        SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(PMIC_SPI);
    spi_set_full_duplex_mode(PMIC_SPI);
    spi_set_unidirectional_mode(PMIC_SPI);
    spi_set_nss_high(PMIC_SPI);

    spi_enable(PMIC_SPI);
}

/**
 * \brief Run through the set up sequence for the power supply and watchdog
 */
TeslaM3PowerWatchdog::Error TeslaM3PowerWatchdog::SetupPowerManagement()
{
    Error result;

    // Write the initial device configuration
    for (const auto reg : RegisterConfig)
    {
        CHECK(WriteRegister(reg.reg, reg.value));
    }

    // Strobe the watchdog so that everything is happy before changing the
    // initial state. This closes the Long Open Window of the Window Watchdog.
    // We need to be careful not to strobe for 50ms until the Closed Window
    // period finishes
    CHECK(Strobe());

    // Move the device into the NORMAL state
    const uint8_t NewState = TLF35584_DEVCTRL_TRK2EN | TLF35584_DEVCTRL_TRK1EN |
                             TLF35584_DEVCTRL_COMEN | TLF35584_DEVCTRL_VREFEN |
                             TLF35584_DEVCTRL_STATEREQ_NORMAL;
    CHECK(WriteRegister(TLF35584_DEVCTRL, NewState));
    CHECK(WriteRegister(TLF35584_DEVCTRLN, TLF35584_DEVCTRLN_STATEREQ_NORMAL));

    // Blocking wait until the state has had time to change
    uDelay(StateTransitionDelay);

    const uint8_t ExpectedState =
        TLF35584_DEVSTAT_TRK2EN | TLF35584_DEVSTAT_TRK1EN |
        TLF35584_DEVSTAT_COMEN | TLF35584_DEVSTAT_VREFEN |
        TLF35584_DEVSTAT_NORMAL;
    uint8_t state;
    CHECK(ReadRegister(TLF35584_DEVSTAT, state));

    if (state == ExpectedState)
    {
        return OK;
    }
    else
    {
        return StateTransitionFail;
    }
}

/**
 * \brief Evaluate the parity
 *
 * Algorithm from
 * https://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
 *
 * \param value Value to checked for parity
 *
 * \return True if there are an odd number of bits set
 */
static bool HasOddParity(uint16_t value)
{
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

/**
 * \brief Write a specific register
 *
 * \param reg Register to write to
 * \param value Value to be written
 *
 * \return Communication failure
 */
TeslaM3PowerWatchdog::Error TeslaM3PowerWatchdog::WriteRegister(
    uint8_t reg,
    uint8_t value)
{
    uint16_t out = TLF35584_SPI_REG_WRITE | TLF35584_SPI_CMD(reg) |
                   TLF35584_SPI_DATA(value);

    if (HasOddParity(out))
    {
        out = out | TLF35584_SPI_PARITY_MASK;
    }

    SPITransaction transaction;

    // A working TLF35584 will echo back any write commands so we can use this
    // to verify communication somewhat
    if (spi_xfer(PMIC_SPI, out) == out)
    {
        return OK;
    }
    else
    {
        return WriteFail;
    }
}

/**
 * \brief Read back a register
 *
 * \param reg Register we want to read back
 *
 * \return Register value
 */
TeslaM3PowerWatchdog::Error TeslaM3PowerWatchdog::ReadRegister(
    uint8_t  reg,
    uint8_t& value)
{
    uint16_t request = TLF35584_SPI_CMD(reg);

    if (HasOddParity(request))
    {
        request = request | TLF35584_SPI_PARITY_MASK;
    }

    SPITransaction transaction;
    uint16_t       response = spi_xfer(PMIC_SPI, request);

    bool parityOk =
        (response & TLF35584_SPI_PARITY_MASK) == HasOddParity(response >> 1);

    if (parityOk)
    {
        value = (response & TLF35584_SPI_DATA_MASK) >> TLF35584_SPI_DATA_SHIFT;
        return OK;
    }
    else
    {
        return ReadParityFail;
    }
}
