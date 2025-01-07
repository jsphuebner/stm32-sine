/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2025 David J. Fiddes <D.J@fiddes.net>
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

#include "teslamodel3.h"
#include "digio.h"
#include "hwinit.h"
#include "errormessage.h"
#include <libopencm3/stm32/spi.h>

// Define the delay macro needed by the tesla::GateDriver header-only class
#include "delay.h"
#define DEVICE_DELAY_US(x) uDelay(x)

#include "teslagatedriver.h"

/**
 * \brief Low level hardware interface to the STGAP1AS gate driver chain in the
 * Tesla M3 inverter
 */
class GateDriverInterface
{
public:
    static const uint16_t NumDriverChips = 6;

    typedef uint16_t DataBuffer[NumDriverChips];

public:
    static void Init();
    static void SendData(DataBuffer writeData, DataBuffer readData);
    static void Shutdown();
    static void Resume();
    static bool IsShutdown();
};

/**
 * \brief Assemble a Gate Driver using the STM32 specific SPI driver and the
 * generic Tesla Gate Driver class
 */
typedef tesla::GateDriver<GateDriverInterface> TeslaM3GateDriver;

/**
 * \brief Set up the SPI bus connected to the STGAP1AS gate drivers
 */
void GateDriverInterface::Init()
{
    // Start with the ~SD line asserted (i.e. in Shutdown state)
    DigIo::gate_sd_hi.Clear();

    // De-assert the SPI ~CS line before configuration to avoid glitches
    DigIo::gate_cs_hi.Set();
    spi_setup_teslam3();
}

/**
 * \brief Initiate an SPI transaction with the gate drivers
 *
 * Encapsulates all of the hardware dependent work required to communicate with
 * an array of daisy-chained STGAP1AS gate driver chips. Inter-message timing
 * delays are not included.
 *
 * \param writeData Data to be sent to the STGAP1AS driver chips
 * \param readData Data buffer for data read from the chips. May be NULL if the
 * received data is not required
 */
void GateDriverInterface::SendData(DataBuffer writeData, DataBuffer readData)
{
    // Manually assert the ~CS pin and add a delay to allow it to settle and
    // match the required set-up time for the STGAP1AS
    DigIo::gate_cs_hi.Clear();
    uDelay(1);

    // Run the SPI transaction
    for (uint16_t i = 0; i < NumDriverChips; i++)
    {
        uint16_t result = spi_xfer(SPI1, writeData[i]);

        if (readData)
        {
            readData[i] = result;
        }

        // Safe but pessimistic delay between words. Tesla use a 2 clock-cycle
        // gap on their inverter.
        uDelay(1);
    }

    // Manually de-assert the ~CS pin and ensure that we have waited sufficient
    // time for the data being sent by the chips to arrive
    uDelay(1);
    DigIo::gate_cs_hi.Set();
}

/**
 * \brief Assert the ~SD line on the STGAP1AS gate drivers allowing them to be
 * configured
 */
void GateDriverInterface::Shutdown()
{
    DigIo::gate_sd_hi.Clear();
}

/**
 * \brief De-assert the ~SD line on the STGAP1AS gate drivers to allow them to
 * run normally
 */
void GateDriverInterface::Resume()
{
    DigIo::gate_sd_hi.Set();
}

/**
 * \brief Return whether the STGAP1AS gate drivers are enabled
 */
bool GateDriverInterface::IsShutdown()
{
    return DigIo::gate_sd_hi.Get();
}

/**
 * \brief Set up the the Tesla Model 3 gate drivers
 */
void TeslaModel3::Initialize()
{
    if (!TeslaM3GateDriver::Init())
    {
        ErrorMessage::Post(ERR_GATEDRIVEINITFAIL);
    }
}

/**
 * \brief Periodically check the gate drivers are happy
 */
void TeslaModel3::CyclicFunction()
{
    if (TeslaM3GateDriver::IsFaulty())
    {
        ErrorMessage::Post(ERR_GATEDRIVEFAULT);
        //TODO: Shutdown the inverter?
    }
}
