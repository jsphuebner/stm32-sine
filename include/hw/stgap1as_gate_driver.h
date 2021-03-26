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

#ifndef HW_STGAP1AS_GATE_DRIVER_H
#define HW_STGAP1AS_GATE_DRIVER_H

/*
 * Defined Constants and Types for the ST Micro STGAP1AS Automotive
 * galvanically isolated advanced single gate driver
 *
 * Definitions from datasheet DocID029344 Rev 4 from
 * https://www.st.com/en/power-management/stgap1as.html
 *
 */

/** @defgroup spi_crc SPI Communication CRC Constants
@{*/

/** SPI communication crc polynomial corresponding to x^8 + x^2 + x + 1 */
#define STGAP1AS_SPI_CRC_POLYNOMIAL             0x07

/** SPI communication CRC initial value */
#define STGAP1AS_SPI_CRC_INIT_VALUE             0xFF

/**@}*/

/** @defgroup spi_commands SPI Commands
@{*/

/** Device configuration start */
#define STGAP1AS_CMD_START_CONFIG               (0b00101010)

/** Device configuration/check completed */
#define STGAP1AS_CMD_STOP_CONFIG                (0b00111010)

/** No operation */
#define STGAP1AS_CMD_NOP                        (0b00000000)

#define STGAP1AS_CMD_REG_MASK			        (0b11100000)
#define STGAP1AS_CMD_WRITE_REG_VALUE			(0b10000000)
#define STGAP1AS_CMD_WRITE_REG_MASK			    0x1F
/** Write register */
#define STGAP1AS_CMD_WRITE_REG(x)               (STGAP1AS_CMD_WRITE_REG_VALUE | (x & STGAP1AS_CMD_WRITE_REG_MASK))

#define STGAP1AS_CMD_READ_REG_VALUE			    (0b10100000)
#define STGAP1AS_CMD_READ_REG_MASK			    0x1F
/** Write register */
#define STGAP1AS_CMD_READ_REG(x)                (STGAP1AS_CMD_READ_REG_VALUE | (x & STGAP1AS_CMD_READ_REG_MASK))

/** Reset all the status registers */
#define STGAP1AS_CMD_RESET_STATUS               (0b11010000)

/** Global reset */
#define STGAP1AS_CMD_GLOBAL_RESET               (0b11101010)

/** Device enters in standby mode */
#define STGAP1AS_CMD_SLEEP                      (0b11110101)

/**@}*/

/** @defgroup registers Registers
@{*/

#define STGAP1AS_REG_CFG1_MASK                  0xFF
/** CFG1 register (low voltage side) */
#define STGAP1AS_REG_CFG1                       0x0C

#define STGAP1AS_REG_CFG2_MASK                  0xFF
/** CFG2 register (isolated side) */
#define STGAP1AS_REG_CFG2                       0x1D

#define STGAP1AS_REG_CFG3_MASK                  0xFF
/** CFG3 register (isolated side) */
#define STGAP1AS_REG_CFG3                       0x1E

#define STGAP1AS_REG_CFG4_MASK                  0x3F
/** CFG4 register (isolated side) */
#define STGAP1AS_REG_CFG4                       0x1F

#define STGAP1AS_REG_CFG5_MASK                  0x0F
/** CFG5 register (isolated side) */
#define STGAP1AS_REG_CFG5                       0x19

#define STGAP1AS_REG_STATUS1_MASK               0xFF
/** STATUS1 register (low voltage side) */
#define STGAP1AS_REG_STATUS1                    0x02

#define STGAP1AS_REG_STATUS2_MASK               0x06
/** STATUS2 register (low voltage side) */
#define STGAP1AS_REG_STATUS2                    0x01

#define STGAP1AS_REG_STATUS3_MASK               0x1F
/** STATUS3 register (low voltage side) */
#define STGAP1AS_REG_STATUS3                    0x0A

#define STGAP1AS_REG_TEST1_MASK                 0x1F
/** TEST1 register (isolated side) */
#define STGAP1AS_REG_TEST1                      0x11

#define STGAP1AS_REG_DIAG1CFG_MASK              0xFF
/** DIAG1CFG register (low voltage side) */
#define STGAP1AS_REG_DIAG1CFG                   0x05

#define STGAP1AS_REG_DIAG2CFG_MASK              0xFF
/** DIAG2CFG register (low voltage side) */
#define STGAP1AS_REG_DIAG2CFG                   0x06

/**@}*/

/** @defgroup reg_cfg1 CFG1 Register Flags
@{*/

/** SPI communication protocol CRC enable */
#define STGAP1AS_REG_CFG1_CRC_SPI               (1 << 7)

/** Supply voltage UVLOD enable */
#define STGAP1AS_REG_CFG1_UVLOD_EN              (1 << 6)

/** SD pin reset STATUS registers */
#define STGAP1AS_REG_CFG1_SD_FLAG               (1 << 5)

/** DIAG2 pin work as open drain output */
#define STGAP1AS_REG_CFG1_DIAG_EN               (1 << 4)

/** Deadtime in ns */
#define STGAP1AS_REG_CFG1_DT_DISABLED           ((0b00) << 2)
#define STGAP1AS_REG_CFG1_DT_250NS              ((0b01) << 2)
#define STGAP1AS_REG_CFG1_DT_800NS              ((0b10) << 2)
#define STGAP1AS_REG_CFG1_DT_1200NS             ((0b11) << 2)

/** Input deglitch time in ns */
#define STGAP1AS_REG_CFG1_IN_FILTER_DISABLED    ((0b00) << 0)
#define STGAP1AS_REG_CFG1_IN_FILTER_160NS       ((0b01) << 0)
#define STGAP1AS_REG_CFG1_IN_FILTER_500NS       ((0b10) << 0)
#define STGAP1AS_REG_CFG1_IN_FILTER_70NS        ((0b11) << 0)

/**@}*/

/** @defgroup reg_cfg2 CFG2 Register Flags
@{*/

/** Sense threshold */
#define STGAP1AS_REG_CFG2_SENSE_100MV           ((0b000) << 5)
#define STGAP1AS_REG_CFG2_SENSE_125MV           ((0b001) << 5)
#define STGAP1AS_REG_CFG2_SENSE_150MV           ((0b010) << 5)
#define STGAP1AS_REG_CFG2_SENSE_175MV           ((0b011) << 5)
#define STGAP1AS_REG_CFG2_SENSE_200MV           ((0b100) << 5)
#define STGAP1AS_REG_CFG2_SENSE_250MV           ((0b101) << 5)
#define STGAP1AS_REG_CFG2_SENSE_300MV           ((0b110) << 5)
#define STGAP1AS_REG_CFG2_SENSE_400MV           ((0b111) << 5)

/** Desaturation current */
#define STGAP1AS_REG_CFG2_DESAT_CUR_250UA       ((0b00) << 3)
#define STGAP1AS_REG_CFG2_DESAT_CUR_500UA       ((0b01) << 3)
#define STGAP1AS_REG_CFG2_DESAT_CUR_750UA       ((0b10) << 3)
#define STGAP1AS_REG_CFG2_DESAT_CUR_1000UA      ((0b11) << 3)

/** Desaturation threshold */
#define STGAP1AS_REG_CFG2_DESAT_TH_3V           ((0b000) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_4V           ((0b001) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_5V           ((0b010) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_6V           ((0b011) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_7V           ((0b100) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_8V           ((0b101) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_9V           ((0b110) << 0)
#define STGAP1AS_REG_CFG2_DESAT_TH_10V          ((0b111) << 0)

/**@}*/

/** @defgroup reg_cfg3 CFG3 Register Flags
@{*/

/** 2-level turn off threshold in V */
#define STGAP1AS_REG_CFG3_2LTO_TH_7V            ((0b0000) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_7_5V          ((0b0001) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_8V            ((0b0010) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_8_5V          ((0b0011) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_9V            ((0b0100) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_9_5V          ((0b0101) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_10V           ((0b0110) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_10_5V         ((0b0111) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_11V           ((0b1000) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_11_5V         ((0b1001) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_12V           ((0b1010) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_12_5V         ((0b1011) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_13V           ((0b1100) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_13_5V         ((0b1101) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_14V           ((0b1110) << 4)
#define STGAP1AS_REG_CFG3_2LTO_TH_14_5V         ((0b1111) << 4)

/** 2-level turn off time in us */
#define STGAP1AS_REG_CFG3_2LTO_TIME_DISABLED    ((0b0000) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_0_75US      ((0b0001) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_1_00US      ((0b0010) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_1_50US      ((0b0011) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_2_00US      ((0b0100) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_2_50US      ((0b0101) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_3_00US      ((0b0110) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_3_50US      ((0b0111) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_3_75US      ((0b1000) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_4_00US      ((0b1001) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_4_25US      ((0b1010) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_4_50US      ((0b1011) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_4_75US      ((0b1100) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_5_00US      ((0b1101) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_5_25US      ((0b1110) << 0)
#define STGAP1AS_REG_CFG3_2LTO_TIME_5_50US      ((0b1111) << 0)

/**@}*/

/** @defgroup reg_cfg4 CFG4 Register Flags
@{*/

/** VH and VL supply voltages OVLO enable */
#define STGAP1AS_REG_CFG4_OVLO_EN               (1 << 5)

/** UVLO protection management */
#define STGAP1AS_REG_CFG4_UVLO_LATCHED          (1 << 4)

/** VL negative supply voltage UVLO threshold in V */
#define STGAP1AS_REG_CFG4_VLON_TH_DISABLED      ((0b00) << 2)
#define STGAP1AS_REG_CFG4_VLON_TH_NEG_3V        ((0b01) << 2)
#define STGAP1AS_REG_CFG4_VLON_TH_NEG_5V        ((0b10) << 2)
#define STGAP1AS_REG_CFG4_VLON_TH_NEG_7V        ((0b11) << 2)

/** VH positive supply voltage UVLO threshold in V */
#define STGAP1AS_REG_CFG4_VHON_TH_DISABLED      ((0b00) << 0)
#define STGAP1AS_REG_CFG4_VHON_TH_10V           ((0b01) << 0)
#define STGAP1AS_REG_CFG4_VHON_TH_12V           ((0b10) << 0)
#define STGAP1AS_REG_CFG4_VHON_TH_14V           ((0b11) << 0)

/**@}*/

/** @defgroup reg_cfg5 CFG5 Register Flags
@{*/

/** 2LTO active only after a fault event */
#define STGAP1AS_REG_CFG5_2LTO_EN               (1 << 3)

/** Miller clamp feature enable */
#define STGAP1AS_REG_CFG5_CLAMP_EN              (1 << 2)

/** Desaturation comparator eanble */
#define STGAP1AS_REG_CFG5_DESAT_EN              (1 << 1)

/** Sense comparator enable */
#define STGAP1AS_REG_CFG5_SENSE_EN              (1 << 0)

/**@}*/

/** @defgroup reg_status1 STATUS1 Register Flags
@{*/

/** VH overvoltage flag */
#define STGAP1AS_REG_STATUS1_OVLOH              (1 << 7)

/** VL overvoltage flag */
#define STGAP1AS_REG_STATUS1_OVLOL              (1 << 6)

/** Desaturation flag */
#define STGAP1AS_REG_STATUS1_DESAT              (1 << 5)

/** Sense flag */
#define STGAP1AS_REG_STATUS1_SENSE              (1 << 4)

/** VH undervoltage flag */
#define STGAP1AS_REG_STATUS1_UVLOH              (1 << 3)

/** VL undervoltage flag */
#define STGAP1AS_REG_STATUS1_UVLOL              (1 << 2)

/** Thermal shutdown protection flag */
#define STGAP1AS_REG_STATUS1_TSD                (1 << 1)

/** Thermal warning flag */
#define STGAP1AS_REG_STATUS1_TWN                (1 << 0)

/**@}*/

/** @defgroup reg_status2 STATUS2 Register Flags
@{*/

/** Register or communication error on isolated side */
#define STGAP1AS_REG_STATUS2_REGERRR            (1 << 2)

/** ASC pin status */
#define STGAP1AS_REG_STATUS2_ASC                (1 << 1)

/**@}*/

/** @defgroup reg_status3 STATUS3 Register Flags
@{*/

/** Deadtime error flag */
#define STGAP1AS_REG_STATUS3_DT_ERR             (1 << 4)

/** SPI communication error flag */
#define STGAP1AS_REG_STATUS3_SPI_ERR            (1 << 3)

/** Register or communication error on low voltage side */
#define STGAP1AS_REG_STATUS3_REGERRL            (1 << 2)

/** VDD overvoltage flag */
#define STGAP1AS_REG_STATUS3_OVLOD              (1 << 1)

/** VDD undervoltage flag */
#define STGAP1AS_REG_STATUS3_UVLOD              (1 << 0)

/**@}*/

/** @defgroup reg_test1 TEST1 Register Flags
@{*/

/** GOFF to gate path check */
#define STGAP1AS_REG_TEST1_GOFFCHK              (1 << 4)

/** GON to gate path check */
#define STGAP1AS_REG_TEST1_GONCHK               (1 << 3)

/** DESAT comparator check */
#define STGAP1AS_REG_TEST1_DESCHK               (1 << 2)

/** SENSE comparator check */
#define STGAP1AS_REG_TEST1_SNSCHK               (1 << 1)

/** SENSE resistor check */
#define STGAP1AS_REG_TEST1_RCHK                 (1 << 0)

/**@}*/

/** @defgroup reg_diag1 DIAG1CFG Register Flags
@{*/

/** DIAG1 on SPI communication error */
#define STGAP1AS_REG_DIAG1CFG_SPI_REG_ERR       (1 << 7)

/** DIAG1 on VDD power supply failure */
#define STGAP1AS_REG_DIAG1CFG_UVLOD_OVLOD       (1 << 6)

/** DIAG1 on Undervoltage failure */
#define STGAP1AS_REG_DIAG1CFG_UVLOH_UVLOL       (1 << 5)

/** DIAG1 on Overvoltage failure */
#define STGAP1AS_REG_DIAG1CFG_OVLOH_OVLOL       (1 << 4)

/** DIAG1 on Desaturation and sense detection */
#define STGAP1AS_REG_DIAG1CFG_DESAT_SENSE       (1 << 3)

/** DIAG1 on ASC feedback */
#define STGAP1AS_REG_DIAG1CFG_ASC_DT_ERR        (1 << 2)

/** DIAG1 on Thermal shutdown protection flag */
#define STGAP1AS_REG_DIAG1CFG_TSD               (1 << 1)

/** DIAG1 on Thermal warning flag */
#define STGAP1AS_REG_DIAG1CFG_TWN               (1 << 0)

/**@}*/

/** @defgroup reg_diag2 DIAG2CFG Register Flags
@{*/

/** DIAG2 on SPI communication error */
#define STGAP1AS_REG_DIAG2CFG_SPI_REG_ERR       (1 << 7)

/** DIAG2 on VDD power supply failure */
#define STGAP1AS_REG_DIAG2CFG_UVLOD_OVLOD       (1 << 6)

/** DIAG2 on Undervoltage failure */
#define STGAP1AS_REG_DIAG2CFG_UVLOH_UVLOL       (1 << 5)

/** DIAG2 on Overvoltage failure */
#define STGAP1AS_REG_DIAG2CFG_OVLOH_OVLOL       (1 << 4)

/** DIAG2 on Desaturation and sense detection */
#define STGAP1AS_REG_DIAG2CFG_DESAT_SENSE       (1 << 3)

/** DIAG2 on ASC feedback */
#define STGAP1AS_REG_DIAG2CFG_ASC_DT_ERR        (1 << 2)

/** DIAG2 on Thermal shutdown protection flag */
#define STGAP1AS_REG_DIAG2CFG_TSD               (1 << 1)

/** DIAG2 on Thermal warning flag */
#define STGAP1AS_REG_DIAG2CFG_TWN               (1 << 0)

/**@}*/

#endif /* HW_STGAP1AS_GATE_DRIVER_H */
