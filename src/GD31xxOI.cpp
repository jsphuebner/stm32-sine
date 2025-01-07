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

#include "GD31xxOI.h"
#include "hwinit.h"
#include <libopencm3/stm32/spi.h>

#define REQ_ADC_EXT 0x4001
#define REQ_STAT1   0x2800
#define REQ_STAT2   0x3000
#define REQ_STAT3   0x3800

// CRC table for polynomial 0x2F.
static const uint8_t MGcrc_table[256] = {
   0x00, 0x2f, 0x5e, 0x71, 0xbc, 0x93, 0xe2, 0xcd, 0x57, 0x78, 0x09, 0x26, 0xeb, 0xc4, 0xb5, 0x9a,
   0xae, 0x81, 0xf0, 0xdf, 0x12, 0x3d, 0x4c, 0x63, 0xf9, 0xd6, 0xa7, 0x88, 0x45, 0x6a, 0x1b, 0x34,
   0x73, 0x5c, 0x2d, 0x02, 0xcf, 0xe0, 0x91, 0xbe, 0x24, 0x0b, 0x7a, 0x55, 0x98, 0xb7, 0xc6, 0xe9,
   0xdd, 0xf2, 0x83, 0xac, 0x61, 0x4e, 0x3f, 0x10, 0x8a, 0xa5, 0xd4, 0xfb, 0x36, 0x19, 0x68, 0x47,
   0xe6, 0xc9, 0xb8, 0x97, 0x5a, 0x75, 0x04, 0x2b, 0xb1, 0x9e, 0xef, 0xc0, 0x0d, 0x22, 0x53, 0x7c,
   0x48, 0x67, 0x16, 0x39, 0xf4, 0xdb, 0xaa, 0x85, 0x1f, 0x30, 0x41, 0x6e, 0xa3, 0x8c, 0xfd, 0xd2,
   0x95, 0xba, 0xcb, 0xe4, 0x29, 0x06, 0x77, 0x58, 0xc2, 0xed, 0x9c, 0xb3, 0x7e, 0x51, 0x20, 0x0f,
   0x3b, 0x14, 0x65, 0x4a, 0x87, 0xa8, 0xd9, 0xf6, 0x6c, 0x43, 0x32, 0x1d, 0xd0, 0xff, 0x8e, 0xa1,
   0xe3, 0xcc, 0xbd, 0x92, 0x5f, 0x70, 0x01, 0x2e, 0xb4, 0x9b, 0xea, 0xc5, 0x08, 0x27, 0x56, 0x79,
   0x4d, 0x62, 0x13, 0x3c, 0xf1, 0xde, 0xaf, 0x80, 0x1a, 0x35, 0x44, 0x6b, 0xa6, 0x89, 0xf8, 0xd7,
   0x90, 0xbf, 0xce, 0xe1, 0x2c, 0x03, 0x72, 0x5d, 0xc7, 0xe8, 0x99, 0xb6, 0x7b, 0x54, 0x25, 0x0a,
   0x3e, 0x11, 0x60, 0x4f, 0x82, 0xad, 0xdc, 0xf3, 0x69, 0x46, 0x37, 0x18, 0xd5, 0xfa, 0x8b, 0xa4,
   0x05, 0x2a, 0x5b, 0x74, 0xb9, 0x96, 0xe7, 0xc8, 0x52, 0x7d, 0x0c, 0x23, 0xee, 0xc1, 0xb0, 0x9f,
   0xab, 0x84, 0xf5, 0xda, 0x17, 0x38, 0x49, 0x66, 0xfc, 0xd3, 0xa2, 0x8d, 0x40, 0x6f, 0x1e, 0x31,
   0x76, 0x59, 0x28, 0x07, 0xca, 0xe5, 0x94, 0xbb, 0x21, 0x0e, 0x7f, 0x50, 0x9d, 0xb2, 0xc3, 0xec,
   0xd8, 0xf7, 0x86, 0xa9, 0x64, 0x4b, 0x3a, 0x15, 0x8f, 0xa0, 0xd1, 0xfe, 0x33, 0x1c, 0x6d, 0x42
};

//Config data for all 6 gate drivers.
static const uint16_t MG_Gate_Config[24] = {
   0x8444, 0x2800, 0x2800, 0xABFF, 0x2800, 0x2800, 0X3000, 0X3000, 0XB3FF, 0X3000, 0X3000, 0XBB00,
   0X82B3, 0XA05A, 0XA470, 0X88DB, 0X8CD4, 0X9024, 0X964B, 0X9906, 0X9C3F, 0XAC00, 0XB400, 0X8440
};

uint16_t MGSPI::temps[3];
uint16_t MGSPI::udc;

void MGSPI::Initialize() {
   DigIo::v5_ctrl.Set(); //Turn on MG gate driver logic side 5v power after 2 seconds
   DigIo::cs1_hi.Set();  //Disable all SPI CS lines on MG
   DigIo::cs2_hi.Set();
   DigIo::cs3_hi.Set();
   DigIo::cs1_lo.Set();
   DigIo::cs2_lo.Set();
   DigIo::cs3_lo.Set();

   tim5_setup(); //Enable gate drive psu on MG board
   spi_setup();  //MG gate drivers
   uDelay(20);

   ConfigureGateDriver(DigIo::cs1_hi);  //configure all 6 MG gate drivers
   ConfigureGateDriver(DigIo::cs1_hi);  //must be sent twice to 1H as its a bit dumb
   ConfigureGateDriver(DigIo::cs2_hi);
   ConfigureGateDriver(DigIo::cs3_hi);
   ConfigureGateDriver(DigIo::cs1_lo);
   ConfigureGateDriver(DigIo::cs2_lo);
   ConfigureGateDriver(DigIo::cs2_lo);
}

void MGSPI::CyclicFunction() {
   static uint8_t MG_Cycler = 0;

   switch (MG_Cycler){
   case 0:
      temps[0] = Send16(DigIo::cs1_hi, REQ_ADC_EXT);//HS Temp 1
      break;
   case 1:
      temps[1] = Send16(DigIo::cs2_hi, REQ_ADC_EXT);//HS Temp 2
      break;
   case 2:
      temps[2] = Send16(DigIo::cs3_hi, REQ_ADC_EXT);//HS Temp 3
      break;
   case 3:
      udc = Send16(DigIo::cs3_lo, REQ_ADC_EXT);//HS Temp 1
      break;
   }

   MG_Cycler = (MG_Cycler + 1) & 0x7; //count from 0-7
}

//////////////////////////////////////////////////////////////////////////////////////////
//  Function name: Calculate_MGSPI_CRC
//
/// @brief         Calculates the required GD3100 MG SPI CRC value.
//
/// @param         uint16_t inData - SPI word.
//
/// @return        uint8_t - CRC value.
//////////////////////////////////////////////////////////////////////////////////////////
static uint8_t Calculate_SPI_CRC(uint16_t inData) {
   uint8_t tbl_idx;
   uint8_t crc;
   uint16_t len = 2;

   uint8_t data[2];
   data[0] = inData & 0xFF;
   data[1]= (inData >> 8) & 0xFF;

   crc = 0x42;  // seed

   while (len) {
      len--;
      tbl_idx = (crc ^ *(data + len));
      crc = MGcrc_table[tbl_idx];
   }

   return crc;
}



//////////////////////////////////////////////////////////////////////////////////////////
//  Function name: Send16
//
/// @brief         Sends a 24 bit SPI word to the selected driver.
//
/// @param         channel - SPI channel for the device , 16 bit data.
//
/// @return        uint32_t - Returns 16 bit answer from device
//////////////////////////////////////////////////////////////////////////////////////////
uint32_t MGSPI::Send16 (DigIo& cspin, uint16_t txData16) {
   uint8_t  i=0;
   uint8_t rxData8[3];
   uint16_t outData=0;
   uint8_t txData8[3];
   // Set chip select low
   cspin.Clear();

   txData8[0]=txData16>>8;
   txData8[1]=txData16 & 0xFF;
   txData8[2]=Calculate_SPI_CRC(txData16);

   for(i=0; i<3; i++) {
      rxData8[i]=spi_xfer(SPI1,txData8[i]);
   }

   cspin.Set();
   outData=(rxData8[1] | rxData8[0]<<8) & 0x3FF;
   return outData;
}

//////////////////////////////////////////////////////////////////////////////////////////
//  Function name: Config_MG
//
/// @brief         Configures all 6 MG gate drivers as per OEM.
//
/// @param         channel - SPI channel for the device.
//
/// @return        uint32_t - Returns 16 bit answer from device
//////////////////////////////////////////////////////////////////////////////////////////
uint32_t MGSPI::ConfigureGateDriver(DigIo& cspin) {
   uint32_t dummyData;
   for(int i=0; i<=23; i++) {
      uDelay(20);// need to allow 20us per command as per datasheet
      dummyData=Send16(cspin, MG_Gate_Config[i]);
   }

   return dummyData;
}
