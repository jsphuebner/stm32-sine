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

#ifndef HW_TLF35584_SAFETY_PSU_H
#define HW_TLF35584_SAFETY_PSU_H

/*
 * Defined Constants and Types for the Infineon TLF35584 Multi Voltage Safety
 * Micro Processor Supply
 *
 * Definitions from datasheet Rev 2 from
 * https://uk.farnell.com/infineon/tlf35584qvvs1xuma2/multi-volt-pwr-supply-ic-40-to/dp/3155085
 *
 */

/** @defgroup spi_comms SPI Communication Helpers
@{*/

/** SPI Register Write Command Bit */
#define TLF35584_SPI_REG_WRITE      0x8000

/** SPI Register Command Mask */
#define TLF35584_SPI_CMD_MASK       0x7E00

/** SPI Register Command Shift */
#define TLF35584_SPI_CMD_SHIFT      9

/** SPI Register Command */
#define TLF35584_SPI_CMD(x)         (((x) << TLF35584_SPI_CMD_SHIFT) & TLF35584_SPI_CMD_MASK)

/** SPI Register Data Mask */
#define TLF35584_SPI_DATA_MASK      0x01FE

/** SPI Register Data Shift */
#define TLF35584_SPI_DATA_SHIFT     1

/** SPI Register Data */
#define TLF35584_SPI_DATA(x)        (((x) << TLF35584_SPI_DATA_SHIFT) & TLF35584_SPI_DATA_MASK)

/** SPI Parity Bit Mask */
#define TLF35584_SPI_PARITY_MASK    0x0001

/**@}*/

/** @defgroup spi_register SPI Registers
@{*/

/** Device configuration 0 */
#define TLF35584_DEVCFG0                        0x00

/** Device configuration 1 */
#define TLF35584_DEVCFG1                        0x01

/** Device configuration 2 */
#define TLF35584_DEVCFG2                        0x02

/** Protection register */
#define TLF35584_PROTCFG                        0x03

/** Protected System configuration request 0 */
#define TLF35584_SYSPCFG0                       0x04

/** Protected System configuration request 1 */
#define TLF35584_SYSPCFG1                       0x05

/** Protected Watchdog configuration request 0 */
#define TLF35584_WDCFG0                         0x06

/** Protected Watchdog configuration request 1 */
#define TLF35584_WDCFG1                         0x07

/** Protected Functional watchdog configuration request 0 */
#define TLF35584_FWDCFG                         0x08

/** Protected Window watchdog configuration request 0 */
#define TLF35584_WWDCFG0                        0x09

/** Protected Window watchdog configuration request 1 */
#define TLF35584_WWDCFG1                        0x0A

/** System configuration 0 status */
#define TLF35584_RSYSPCFG0                      0x0B

/** System configuration 1 status */
#define TLF35584_RSYSPCFG1                      0x0C

/** Watchdog configuration 0 status */
#define TLF35584_RWDCFG0                        0x0D

/** Watchdog configuration 1 status */
#define TLF35584_RWDCFG1                        0x0E

/** Functional watchdog configuration status */
#define TLF35584_RFWDCFG                        0x0F

/** Window watchdog configuration 0 status */
#define TLF35584_RWWDCFG0                       0x10

/** Window watchdog configuration 1 status */
#define TLF35584_RWWDCFG1                       0x11

/** Wake timer configuration 0 */
#define TLF35584_WKTIMCFG0                      0x12

/** Wake timer configuration 1 */
#define TLF35584_WKTIMCFG1                      0x13

/** Wake timer configuration 2 */
#define TLF35584_WKTIMCFG2                      0x14

/** Device control request */
#define TLF35584_DEVCTRL                        0x15

/** Device control inverted request */
#define TLF35584_DEVCTRLN                       0x16

/** Window watchdog service command */
#define TLF35584_WWDSCMD                        0x17

/** Functional watchdog response command */
#define TLF35584_FWDRSP                         0x18

/** Functional watchdog response command with synchronization */
#define TLF35584_FWDRSPSYNC                     0x19

/** Failure status flags */
#define TLF35584_SYSFAIL                        0x1A

/** Init error status flags */
#define TLF35584_INITERR                        0x1B

/** Interrupt flags */
#define TLF35584_IF                             0x1C

/** System status flags  */
#define TLF35584_SYSSF                          0x1D

/** Wakeup status flags */
#define TLF35584_WKSF                           0x1E

/** SPI status flags */
#define TLF35584_SPISF                          0x1F

/** Monitor status flags 0 */
#define TLF35584_MONSF0                         0x20

/** Monitor status flags 1 */
#define TLF35584_MONSF1                         0x21

/** Monitor status flags 2 */
#define TLF35584_MONSF2                         0x22

/** Monitor status flags 3 */
#define TLF35584_MONSF3                         0x23

/** Over temperature failure status flags */
#define TLF35584_OTFAIL                         0x24

/** Over temperature warning status flags */
#define TLF35584_OTWRNSF                        0x25

/** Voltage monitor status */
#define TLF35584_VMONSTAT                       0x26

/** Device status */
#define TLF35584_DEVSTAT                        0x27

/** Protection status  */
#define TLF35584_PROTSTAT                       0x28

/** Window watchdog status */
#define TLF35584_WWDSTAT                        0x29

/** Functional watchdog status 0 */
#define TLF35584_FWDSTAT0                       0x2A

/** Functional watchdog status 1 */
#define TLF35584_FWDSTAT1                       0x2B

/** ABIST control0  */
#define TLF35584_ABIST_CTRL0                    0x2C

/** ABIST control1 */
#define TLF35584_ABIST_CTRL1                    0x2D

/** ABIST select 0  */
#define TLF35584_ABIST_SELECT0                  0x2E

/** ABIST select 1 */
#define TLF35584_ABIST_SELECT1                  0x2F

/** ABIST select 2 */
#define TLF35584_ABIST_SELECT2                  0x30

/** Global testmode */
#define TLF35584_GTM                            0x3F

/** Buck switching frequency change */
#define TLF35584_BCK_FREQ_CHANGE                0x31

/** Buck Frequency spread */
#define TLF35584_BCK_FRE_SPREAD                 0x32

/** Buck main control */
#define TLF35584_BCK_MAIN_CTRL                  0x33

/**@}*/

/** @defgroup reg_devcfg0 DEVCFG0 Register Flags
@{*/

/** Wake timer enable */
#define TLF35584_DEVCFG0_WKTIMEN                (1 << 7)

/** Wake timer cycle period  */
#define TLF35584_DEVCFG0_WKTIMCYC_10US          (0 << 6)
#define TLF35584_DEVCFG0_WKTIMCYC_10MS          (1 << 6)

/** Transition delay into low power states in us
 *  100us - 1600us
 */
#define TLF35584_DEVCFG0_TRDEL_US(x)            (((x) / 100) - 1)

/**@}*/

/** @defgroup reg_devcfg1 DEVCFG1 Register Flags
@{*/

/** Reset release delay time */
#define TLF35584_DEVCFG1_RESDEL_200US           0x0
#define TLF35584_DEVCFG1_RESDEL_400US           0x1
#define TLF35584_DEVCFG1_RESDEL_800US           0x2
#define TLF35584_DEVCFG1_RESDEL_1MS             0x3
#define TLF35584_DEVCFG1_RESDEL_2MS             0x4
#define TLF35584_DEVCFG1_RESDEL_4MS             0x5
#define TLF35584_DEVCFG1_RESDEL_10MS            0x6
#define TLF35584_DEVCFG1_RESDEL_15MS            0x7

/**@}*/

/** @defgroup reg_devcfg2 DEVCFG2 Register Flags
@{*/

/** External core supply enable status */
#define TLF35584_DEVCFG2_EVCEN                  (1 << 7)

/** Step-up converter enable status */
#define TLF35584_DEVCFG2_STU                    (1 << 6)

/** Step-down converter frequency selection status */
#define TLF35584_DEVCFG2_FRE_LOW                (0 << 5)
#define TLF35584_DEVCFG2_FRE_HIGH               (1 << 5)

/** QUC current monitor enable for transition to a low power state */
#define TLF35584_DEVCFG2_CMONEN                 (1 << 4)

/** QUC current monitoring threshold value */
#define TLF35584_DEVCFG2_CTHR_10MA              ((0x0) << 2)
#define TLF35584_DEVCFG2_CTHR_30MA              ((0x1) << 2)
#define TLF35584_DEVCFG2_CTHR_60MA              ((0x2) << 2)
#define TLF35584_DEVCFG2_CTHR_100MA             ((0x3) << 2)

/** External synchronization output phase */
#define TLF35584_DEVCFG2_ESYNPHA_0DEG           (0 << 1)
#define TLF35584_DEVCFG2_ESYNPHA_180DEG         (1 << 1)

/** Synchronization output for external switchmode regulator enable */
#define TLF35584_DEVCFG2_ESYNEN                 (1 << 0)

/**@}*/

/** @defgroup reg_protcfg PROTCFG Register Flags
@{*/

/** Protection key unlock sequence. To unlock write key 1 - 4 in successive
 * writes to PROTCFG register
 */
#define TLF35584_PROTCFG_UNLOCK_KEY1            0xAB
#define TLF35584_PROTCFG_UNLOCK_KEY2            0xEF
#define TLF35584_PROTCFG_UNLOCK_KEY3            0x56
#define TLF35584_PROTCFG_UNLOCK_KEY4            0x12

/** Protection key lock sequence. To lock write key 1 - 4 in successive
 * writes to PROTCFG register
 */
#define TLF35584_PROTCFG_LOCK_KEY1              0xDF
#define TLF35584_PROTCFG_LOCK_KEY2              0x34
#define TLF35584_PROTCFG_LOCK_KEY3              0xBE
#define TLF35584_PROTCFG_LOCK_KEY4              0xCA

/**@}*/

/** @defgroup reg_syspcfg0 SYSPCFG0 Register Flags
@{*/

/** Request standby regulator QST enable */
#define TLF35584_SYSPCFG0_STBYEN                (1 << 0)

/**@}*/

/** @defgroup reg_syspcfg1 SYSPCFG1 Register Flags
@{*/

/** Request safe state 2 delay */
#define TLF35584_SYSPCFG1_SS2DEL_0MS            ((0x0) << 5)
#define TLF35584_SYSPCFG1_SS2DEL_10MS           ((0x1) << 5)
#define TLF35584_SYSPCFG1_SS2DEL_50MS           ((0x2) << 5)
#define TLF35584_SYSPCFG1_SS2DEL_100MS          ((0x3) << 5)
#define TLF35584_SYSPCFG1_SS2DEL_250MS          ((0x4) << 5)

/** Request ERR pin monitor functionality enable while the system is in
 *  SLEEP
 */
#define TLF35584_SYSPCFG1_ERRSLPEN              (1 << 4)

/** Request ERR pin monitor enable */
#define TLF35584_SYSPCFG1_ERREN                 (1 << 3)

/** Request ERR pin monitor recovery enable */
#define TLF35584_SYSPCFG1_ERRRECEN              (1 << 2)

/** Request ERR pin monitor recovery time */
#define TLF35584_SYSPCFG1_ERRREC_1MS            ((0x0) << 1)
#define TLF35584_SYSPCFG1_ERRREC_2_5MS          ((0x1) << 1)
#define TLF35584_SYSPCFG1_ERRREC_5MS            ((0x2) << 1)
#define TLF35584_SYSPCFG1_ERRREC_10MS           ((0x3) << 1)

/**@}*/

/** @defgroup reg_wdcfg0 WDCFG0 Register Flags
@{*/

#define TLF35584_WDCFG0_WWDETHR_MASK            0xF
/** Request window watchdog error threshold */
#define TLF35584_WDCFG0_WWDETHR(x)              (((x) & TLF35584_WDCFG0_WWDETHR_MASK) << 4)

/** Request window watchdog enable */
#define TLF35584_WDCFG0_WWDEN                   (1 << 3)

/** Request functional watchdog enable */
#define TLF35584_WDCFG0_FWDEN                   (1 << 2)

/** Request window watchdog trigger selection */
#define TLF35584_WDCFG0_WWDTSEL                 (1 << 1)

/** Request watchdog cycle time in ms */
#define TLF35584_WDCFG0_WDCYC_0_1MS             (0 << 0)
#define TLF35584_WDCFG0_WDCYC_1MS               (1 << 0)

/**@}*/

/** @defgroup reg_wdcfg1 WDCFG1 Register Flags
@{*/

/** Request watchdog functionality enable while the device is in SLEEP */
#define TLF35584_WDCFG1_WDSLPEN                 (1 << 4)

#define TLF35584_WDCFG1_FWDETHR_MASK            0xF
/** Request functional watchdog error threshold */
#define TLF35584_WDCFG1_FWDETHR(x)      (((x) & TLF35584_WDCFG1_FWDETHR_MASK) << 0)

/**@}*/

/** @defgroup reg_fwdcfg FWDCFG Register Flags
@{*/

#define TLF35584_FWDCFG_WDHBTP_MASK            0x1F
/** Request functional watchdog heartbeat timer period in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
*/
#define TLF35584_FWDCFG_WDHBTP_CYCLES(x)    ((((x) / 50) - 1) & TLF35584_FWDCFG_WDHBTP_MASK)

/**@}*/

/** @defgroup reg_wwdcfg0 WWDCFG0 Register Flags
@{*/

#define TLF35584_WWDCFG0_CW_MASK                0x1F
/** Request window watchdog closed window time in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
*/
#define TLF35584_WWDCFG0_CW_CYCLES(x)   ((((x) / 50) - 1) & TLF35584_WWDCFG0_CW_MASK)

/**@}*/

/** @defgroup reg_wwdcfg1 WWDCFG1 Register Flags
@{*/

#define TLF35584_WWDCFG1_OW_MASK                0x1F
/** Request window watchdog open window time in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
*/
#define TLF35584_WWDCFG1_OW_CYCLES(x)   ((((x) / 50) - 1) & TLF35584_WWDCFG1_OW_MASK)

/**@}*/

/** @defgroup reg_rsyspcfg0 RSYSPCFG0 Register Flags
@{*/

/** Standby regulator QST enable status */
#define TLF35584_RSYSPCFG0_STBYEN               (1 << 0)

/**@}*/

/** @defgroup reg_rsyspcfg1 RSYSPCFG1 Register Flags
@{*/

/** Safe state 2 delay status */
#define TLF35584_RSYSPCFG1_SS2DEL_0MS           ((0x0) << 5)
#define TLF35584_RSYSPCFG1_SS2DEL_10MS          ((0x1) << 5)
#define TLF35584_RSYSPCFG1_SS2DEL_50MS          ((0x2) << 5)
#define TLF35584_RSYSPCFG1_SS2DEL_100MS         ((0x3) << 5)
#define TLF35584_RSYSPCFG1_SS2DEL_250MS         ((0x4) << 5)

/** ERR pin monitor functionality enable status while the device is in SLEEP */
#define TLF35584_RSYSPCFG1_ERRSLPEN             (1 << 4)

/** ERR pin monitor enable status */
#define TLF35584_RSYSPCFG1_ERREN                (1 << 3)

/** ERR pin monitor recovery enable status */
#define TLF35584_RSYSPCFG1_ERRRECEN             (1 << 2)

/** ERR pin monitor recovery time status */
#define TLF35584_RSYSPCFG1_ERRREC_1MS           0x0
#define TLF35584_RSYSPCFG1_ERRREC_2_5MS         0x1
#define TLF35584_RSYSPCFG1_ERRREC_5MS           0x2
#define TLF35584_RSYSPCFG1_ERRREC_10MS          0x3

/**@}*/

/** @defgroup reg_rwdcfg0 RWDCFG0 Register Flags
@{*/

#define TLF35584_RWDCFG0_WWDETHR_MASK           0xF
/** Window watchdog error threshold status */
#define TLF35584_RWDCFG0_WWDETHR(x)     (((x) & TLF35584_RWDCFG0_WWDETHR_MASK) << 4)

/** Window watchdog enable status */
#define TLF35584_RWDCFG0_WWDEN                  (1 << 3)

/** Functional watchdog enable status */
#define TLF35584_RWDCFG0_FWDEN                  (1 << 2)

/** Window watchdog trigger selection status */
#define TLF35584_RWDCFG0_WWDTSEL                (1 << 1)

/** Watchdog cycle time status */
#define TLF35584_RWDCFG0_WDCYC_0_1MS            (0 << 0)
#define TLF35584_RWDCFG0_WDCYC_1MS              (1 << 0)

/**@}*/

/** @defgroup reg_rwdcfg1 RWDCFG1 Register Flags
@{*/

/** Watchdog functionality enable status while the device is in SLEEP */
#define TLF35584_RWDCFG1_WDSLPEN                (1 << 4)

#define TLF35584_RWDCFG1_FWDETHR_MASK           0xF
/** Window watchdog error threshold status */
#define TLF35584_RWDCFG1_FWDETHR(x)     (((x) & TLF35584_RWDCFG1_FWDETHR_MASK) << 0)

/**@}*/

/** @defgroup reg_rfwdcfg RFWDCFG Register Flags
@{*/

#define TLF35584_RFWDCFG_WDHBTP_MASK            0x1F
/** Functional watchdog heartbeat timer period status in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
 */
#define TLF35584_RFWDCFG_WDHBTP_CYCLES(x)   ((((x) / 50) - 1) & TLF35584_RFWDCFG_WDHBTP_MASK)

/**@}*/

/** @defgroup reg_rwwdcfg0 RWWDCFG0 Register Flags
@{*/

#define TLF35584_RWWDCFG0_CW_MASK               0x1F
/** Window watchdog closed window time status in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
 */
#define TLF35584_RWWDCFG0_CW_CYCLES(x)   ((((x) / 50) - 1) & TLF35584_RWWDCFG0_CW_MASK)

/**@}*/

/** @defgroup reg_rwwdcfg1 RWWDCFG1 Register Flags
@{*/

#define TLF35584_RWWDCFG1_OW_MASK               0x1F
/** Window watchdog closed window time status in cycles with
 *  50 cycle granularity. Range 50 cycles to 1600 cycles.
 */
#define TLF35584_RWWDCFG1_OW_CYCLES(x)   ((((x) / 50) - 1) & TLF35584_RWWDCFG1_OW_MASK)

/**@}*/

/** @defgroup reg_wktimcfg WKTIMCFG0/1/2 Register Flags
@{*/

/** Wake timer value. 24bit value in 8 bit chunks */
#define TLF35584_WKTIMCFG0_TIME(x)              ((x) & 0xFF)
#define TLF35584_WKTIMCFG1_TIME(x)              (((x) >> 8) & 0xFF)
#define TLF35584_WKTIMCFG2_TIME(x)              (((x) >> 16) & 0xFF)

/**@}*/

/** @defgroup reg_devctrl DEVCTRL Register Flags
@{*/

/** Request tracker2 QT2 enable */
#define TLF35584_DEVCTRL_TRK2EN                 (1 << 7)

/** Request tracker1 QT1 enable */
#define TLF35584_DEVCTRL_TRK1EN                 (1 << 6)

/** Request communication ldo QCO enable */
#define TLF35584_DEVCTRL_COMEN                  (1 << 5)

/** Request voltage reference QVR enable */
#define TLF35584_DEVCTRL_VREFEN                 (1 << 3)

/** Request for device state transition */
#define TLF35584_DEVCTRL_STATEREQ_NONE          0x0
#define TLF35584_DEVCTRL_STATEREQ_INIT          0x1
#define TLF35584_DEVCTRL_STATEREQ_NORMAL        0x2
#define TLF35584_DEVCTRL_STATEREQ_SLEEP         0x3
#define TLF35584_DEVCTRL_STATEREQ_STANDBY       0x4
#define TLF35584_DEVCTRL_STATEREQ_WAKE          0x5
#define TLF35584_DEVCTRL_STATEREQ_RESERVED1     0x6
#define TLF35584_DEVCTRL_STATEREQ_RESERVED2     0x7

/**@}*/

/** @defgroup reg_devctrl DEVCTRLN Register Flags (Inverse of DEVCTRL)
@{*/

/** Request tracker2 QT2 disable */
#define TLF35584_DEVCTRLN_TRK2DIS               (1 << 7)

/** Request tracker1 QT1 disable */
#define TLF35584_DEVCTRLN_TRK1DIS               (1 << 6)

/** Request communication ldo QCO disable */
#define TLF35584_DEVCTRLN_COMDIS                (1 << 5)

/** Request voltage reference QVR disable */
#define TLF35584_DEVCTRLN_VREFDIS               (1 << 3)

/** Request for device state transition */
#define TLF35584_DEVCTRLN_STATEREQ_NONE         0x7
#define TLF35584_DEVCTRLN_STATEREQ_INIT         0x6
#define TLF35584_DEVCTRLN_STATEREQ_NORMAL       0x5
#define TLF35584_DEVCTRLN_STATEREQ_SLEEP        0x4
#define TLF35584_DEVCTRLN_STATEREQ_STANDBY      0x3
#define TLF35584_DEVCTRLN_STATEREQ_WAKE         0x2
#define TLF35584_DEVCTRLN_STATEREQ_RESERVED1    0x1
#define TLF35584_DEVCTRLN_STATEREQ_RESERVED2    0x0

/**@}*/

/** @defgroup reg_wwdscmd WWDSCMD Register Flags
@{*/

/** Last SPI trigger received */
#define TLF35584_WWDSCMD_TRIG_STATUS            (1 << 7)

/** Window watchdog SPI trigger command */
#define TLF35584_WWDSCMD_TRIG                   (1 << 0)

/**@}*/

/** @defgroup reg_sysfail SYSFAIL Register Flags
@{*/

/** INIT failure flag */
#define TLF35584_SYSFAIL_INITF                  (1 << 7)

/** ABIST operation interrupted flag */
#define TLF35584_SYSFAIL_ABISTERR               (1 << 6)

/** Voltage monitor failure flag */
#define TLF35584_SYSFAIL_VMONF                  (1 << 2)

/** Over temperature failure flag */
#define TLF35584_SYSFAIL_OTF                    (1 << 1)

/** Double Bit error on voltage selection flag */
#define TLF35584_SYSFAIL_VOLTSELERR             (1 << 0)

/**@}*/

/** @defgroup reg_initerr INITERR Register Flags
@{*/

/** Hard reset flag */
#define TLF35584_INITERR_HARDRES                (1 << 7)

/** Soft reset flag */
#define TLF35584_INITERR_SOFTRES                (1 << 6)

/** MCU error monitor failure flag */
#define TLF35584_INITERR_ERRF                   (1 << 5)

/** Functional watchdog error counter overflow failure flag */
#define TLF35584_INITERR_FWDF                   (1 << 4)

/** Window watchdog error counter overflow failure flag */
#define TLF35584_INITERR_WWDF                   (1 << 3)

/** Voltage monitor failure flag */
#define TLF35584_INITERR_VMONF                  (1 << 2)

/**@}*/

/** @defgroup reg_if Interrupt(IF) Register Flags
@{*/

/** Interrupt not serviced in time flag */
#define TLF35584_IF_INTMISS                     (1 << 7)

/** Requested ABIST operation performed flag */
#define TLF35584_IF_ABIST                       (1 << 6)

/** Over temperature failure interrupt flag */
#define TLF35584_IF_OTF                         (1 << 5)

/** Over temperature warning interrupt flag */
#define TLF35584_IF_OTW                         (1 << 4)

/** Monitor interrupt flag */
#define TLF35584_IF_MON                         (1 << 3)

/** SPI interrupt flag */
#define TLF35584_IF_SPI                         (1 << 2)

/** Wake interrupt flag */
#define TLF35584_IF_WK                          (1 << 1)

/** System interrupt flag */
#define TLF35584_IF_SYS                         (1 << 0)

/**@}*/

/** @defgroup reg_syssf SYSSF Register Flags
@{*/

/** State transition request failure flag */
#define TLF35584_SYSSF_NO_OP                    (1 << 5)

/** Transition to low power failed flag */
#define TLF35584_SYSSF_TRFAIL                   (1 << 4)

/** MCU error miss status flag */
#define TLF35584_SYSSF_ERRMISS                  (1 << 3)

/** Functional watchdog error interrupt flag */
#define TLF35584_SYSSF_FWDE                     (1 << 2)

/** Window watchdog error interrupt flag */
#define TLF35584_SYSSF_WWDE                     (1 << 1)

/** Protected configuration double bit error flag */
#define TLF35584_SYSSF_CFGE                     (1 << 0)

/**@}*/

/** @defgroup reg_wksf WKSF Register Flags
@{*/

/** Wakeup from SLEEP by SPI flag (GoToWAKE) */
#define TLF35584_WKSF_WKSPI                     (1 << 4)

/** Wake timer wakeup flag */
#define TLF35584_WKSF_WKTIM                     (1 << 3)

/** QUC current monitor threshold wakeup flag */
#define TLF35584_WKSF_CMON                      (1 << 2)

/** ENA signal wakeup flag */
#define TLF35584_WKSF_ENA                       (1 << 1)

/** WAK signal wakeup flag */
#define TLF35584_WKSF_WAK                       (1 << 0)

/**@}*/

/** @defgroup reg_spisf SPISF Register Flags
@{*/

/** LOCK or UNLOCK procedure error flag */
#define TLF35584_SPISF_LOCK                     (1 << 4)

/** SPI frame duration error flag */
#define TLF35584_SPISF_DURE                     (1 << 3)

/** SPI address invalid flag */
#define TLF35584_SPISF_ADDRE                    (1 << 2)

/** SPI frame length invalid flag */
#define TLF35584_SPISF_LENE                     (1 << 1)

/** SPI frame parity error flag */
#define TLF35584_SPISF_PARE                     (1 << 0)

/**@}*/

/** @defgroup reg_monsf0 MONSF0 Register Flags
@{*/

/** Tracker2 short to ground status flag */
#define TLF35584_MONSF0_TRK2SG                  (1 << 7)

/** Tracker1 short to ground status flag */
#define TLF35584_MONSF0_TRK1SG                  (1 << 6)

/** Voltage reference short to ground status flag */
#define TLF35584_MONSF0_VREFSG                  (1 << 5)

/** Communication LDO short to ground status flag */
#define TLF35584_MONSF0_COMSG                   (1 << 4)

/** Core voltage short to ground status flag */
#define TLF35584_MONSF0_VCORESG                 (1 << 3)

/** Standby LDO short to ground status flag */
#define TLF35584_MONSF0_STBYSG                  (1 << 2)

/** uC LDO short to ground status flag */
#define TLF35584_MONSF0_UCSG                    (1 << 1)

/** Pre-regulator voltage short to ground status flag */
#define TLF35584_MONSF0_PREGSG                  (1 << 0)

/**@}*/

/** @defgroup reg_monsf1 MONSF1 Register Flags
@{*/

/** Tracker2 over voltage status flag */
#define TLF35584_MONSF1_TRK2OV                  (1 << 7)

/** Tracker1 over voltage status flag */
#define TLF35584_MONSF1_TRK1OV                  (1 << 6)

/** Voltage reference over voltage status flag */
#define TLF35584_MONSF1_VREFOV                  (1 << 5)

/** Communication LDO over voltage status flag */
#define TLF35584_MONSF1_COMOV                   (1 << 4)

/** Core voltage over voltage status flag */
#define TLF35584_MONSF1_VCOREOV                 (1 << 3)

/** Standby LDO over voltage status flag */
#define TLF35584_MONSF1_STBYOV                  (1 << 2)

/** uC LDO over voltage status flag */
#define TLF35584_MONSF1_UCOV                    (1 << 1)

/** Pre-regulator voltage over voltage status flag */
#define TLF35584_MONSF1_PREGOV                  (1 << 0)

/**@}*/

/** @defgroup reg_monsf2 MONSF2 Register Flags
@{*/

/** Tracker2 under voltage status flag */
#define TLF35584_MONSF2_TRK2UV                  (1 << 7)

/** Tracker1 under voltage status flag */
#define TLF35584_MONSF2_TRK1UV                  (1 << 6)

/** Voltage reference under voltage status flag */
#define TLF35584_MONSF2_VREFUV                  (1 << 5)

/** Communication LDO under voltage status flag */
#define TLF35584_MONSF2_COMUV                   (1 << 4)

/** Core voltage under voltage status flag */
#define TLF35584_MONSF2_VCOREUV                 (1 << 3)

/** Standby LDO under voltage status flag */
#define TLF35584_MONSF2_STBYUV                  (1 << 2)

/** uC LDO under voltage status flag */
#define TLF35584_MONSF2_UCUV                    (1 << 1)

/** Pre-regulator voltage under voltage status flag */
#define TLF35584_MONSF2_PREGUV                  (1 << 0)

/**@}*/

/** @defgroup reg_monsf3 MONSF3 Register Flags
@{*/

/** Bias current too high flag */
#define TLF35584_MONSF3_BIASHI                  (1 << 7)

/** Bias current too low flag */
#define TLF35584_MONSF3_BIASLOW                 (1 << 6)

/** Bandgap comparator over voltage condition flag */
#define TLF35584_MONSF3_BG12OV                   (1 << 5)

/** Bandgap comparator under voltage condition flag */
#define TLF35584_MONSF3_BG12UV                  (1 << 4)

/** Supply voltage VSx over voltage flag */
#define TLF35584_MONSF3_VBATOV                  (1 << 0)

/**@}*/

/** @defgroup reg_otfail OTFAIL Register Flags
@{*/

/** Monitoring over temperature flag */
#define TLF35584_OTFAIL_MON                     (1 << 7)

/** Communication LDO over temperature flag */
#define TLF35584_OTFAIL_COM                     (1 << 4)

/** uC LDO over temperature flag */
#define TLF35584_OTFAIL_UC                      (1 << 1)

/** Pre-regulator over temperature flag */
#define TLF35584_OTFAIL_PREG                    (1 << 0)

/**@}*/

/** @defgroup reg_otwrnsf OTWRNSF Register Flags
@{*/

/** Voltage reference over load flag */
#define TLF35584_OTWRNSF_VREF                   (1 << 5)

/** Communication LDO over temperature warning flag */
#define TLF35584_OTWRNSF_COM                    (1 << 4)

/** Standby LDO over load flag */
#define TLF35584_OTWRNSF_STDBY                  (1 << 2)

/** uC LDO over temperature warning flag */
#define TLF35584_OTWRNSF_UC                     (1 << 1)

/** Pre-regulator over temperature warning flag */
#define TLF35584_OTWRNSF_PREG                   (1 << 0)

/**@}*/

/** @defgroup reg_vmonstat VMONSTAT Register Flags
@{*/

/** Tracker2 voltage ready status */
#define TLF35584_VMONSTAT_TRK2ST                (1 << 7)

/** Tracker1 voltage ready status */
#define TLF35584_VMONSTAT_TRK1ST                (1 << 6)

/** Voltage reference voltage ready status */
#define TLF35584_VMONSTAT_VREFST                (1 << 5)

/** Communication LDO voltage ready status */
#define TLF35584_VMONSTAT_COMST                 (1 << 4)

/** Core voltage ready status */
#define TLF35584_VMONSTAT_VCOREST               (1 << 3)

/** Standby LDO voltage ready status */
#define TLF35584_VMONSTAT_STBYST                (1 << 2)

/**@}*/

/** @defgroup reg_devstat DEVSTAT Register Flags
@{*/

/** Tracker2 voltage enable status */
#define TLF35584_DEVSTAT_TRK2EN                 (1 << 7)

/** Tracker1 voltage enable status */
#define TLF35584_DEVSTAT_TRK1EN                 (1 << 6)

/** Communication LDO enable status */
#define TLF35584_DEVSTAT_COMEN                  (1 << 5)

/** Standby LDO enable status */
#define TLF35584_DEVSTAT_STBYEN                 (1 << 4)

/** Reference voltage enable status */
#define TLF35584_DEVSTAT_VREFEN                 (1 << 3)

/** Device state */
#define TLF35584_DEVSTAT_RESERVED2              0x7
#define TLF35584_DEVSTAT_RESERVED1              0x6
#define TLF35584_DEVSTAT_WAKE                   0x5
#define TLF35584_DEVSTAT_STANDBY                0x4
#define TLF35584_DEVSTAT_SLEEP                  0x3
#define TLF35584_DEVSTAT_NORMAL                 0x2
#define TLF35584_DEVSTAT_INIT                   0x1
#define TLF35584_DEVSTAT_NONE                   0x0

/**@}*/

/** @defgroup reg_protstat PROTSTAT Register Flags
@{*/

/** Key4 ok status */
#define TLF35584_PROTSTAT_KEY4OK                (1 << 7)

/** Key3 ok status */
#define TLF35584_PROTSTAT_KEY3OK                (1 << 6)

/** Key2 ok status */
#define TLF35584_PROTSTAT_KEY2OK                (1 << 5)

/** Key1 ok status */
#define TLF35584_PROTSTAT_KEY1OK                (1 << 4)

/** Protected register lock status */
#define TLF35584_PROTSTAT_LOCK                  (1 << 0)

/**@}*/

/** @defgroup reg_fwdstat0 FWDSTAT0 Register Flags
@{*/

/** Functional watchdog response check error status */
#define TLF35584_FWDSTAT0_FWDRSPOK              (1 << 6)

/** Functional watchdog response counter value */
#define TLF35584_FWDSTAT0_FWDRSPC_MASK          (0x3 << 4)
#define TLF35584_FWDSTAT0_FWDRSPC_SHIFT         4

/** Functional watchdog question */
#define TLF35584_FWDSTAT0_FWDQUEST_MASK         0xF

/**@}*/

/** @defgroup reg_abist_ctrl0 ABIST_CTRL0 Register Flags
@{*/

/** Functional watchdog response check error status */
#define TLF35584_ABIST_CTRL0_STATUS_NO_ERROR    (5 << 4)
#define TLF35584_ABIST_CTRL0_STATUS_ERROR       (10 << 4)

/** Safety path selection */
#define TLF35584_ABIST_CTRL0_INT                (1 << 3)

/** ABIST Sequence selection */
#define TLF35584_ABIST_CTRL0_SINGLE             (1 << 2)

/** Full path test selection */
#define TLF35584_ABIST_CTRL0_PATH               (1 << 1)

/** Start ABIST operation */
#define TLF35584_ABIST_CTRL0_START              (1 << 0)

/**@}*/

/** @defgroup reg_abist_ctrl1 ABIST_CTRL1 Register Flags
@{*/

/** ABIST clock check enable */
#define TLF35584_ABIST_CTRL1_ABIST_CLK_EN       (1 << 1)

/** Overvoltage trigger for secondary internal monitor enable */
#define TLF35584_ABIST_CTRL1_OV_TRIG            (1 << 0)

/**@}*/

/** @defgroup reg_abist_select0 ABIST_SELECT0 Register Flags
@{*/

/** Select TRK2 OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_TRK2OV           (1 << 7)

/** Select TRK1 OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_TRK1OV           (1 << 6)

/** Select VREF OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_VREFOV           (1 << 5)

/** Select COM OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_COMOV            (1 << 4)

/** Select Core voltage OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_VCOREOV          (1 << 3)

/** Select Standby LDO OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_STBYOV           (1 << 2)

/** Select uC LDO OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_UCOV             (1 << 1)

/** Select Pre-regulator OV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT0_PREGOV           (1 << 0)

/**@}*/

/** @defgroup reg_abist_select1 ABIST_SELECT1 Register Flags
@{*/

/** Select TRK2 UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_TRK2UV           (1 << 7)

/** Select TRK1 UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_TRK1UV           (1 << 6)

/** Select VREF UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_VREFUV           (1 << 5)

/** Select COM UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_COMUV            (1 << 4)

/** Select Core voltage UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_VCOREUV          (1 << 3)

/** Select Standby LDO UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_STBYUV           (1 << 2)

/** Select uC LDO UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_UCUV             (1 << 1)

/** Select Pre-regulator UV comparator for ABIST operation */
#define TLF35584_ABIST_SELECT1_PREGUV           (1 << 0)

/**@}*/

/** @defgroup reg_abist_select2 ABIST_SELECT2 Register Flags
@{*/

/** Select bias current too high */
#define TLF35584_ABIST_SELECT2_BIASHI           (1 << 7)

/** Select bias current too low */
#define TLF35584_ABIST_SELECT2_BIASLOW          (1 << 6)

/** Select bandgap comparator OV condition */
#define TLF35584_ABIST_SELECT2_BG12OV           (1 << 5)

/** Select bandgap comparator UV condition */
#define TLF35584_ABIST_SELECT2_BG12UV           (1 << 4)

/** uSelect internal supply OV condition */
#define TLF35584_ABIST_SELECT2_INTOV            (1 << 3)

/** Select supply VSx overvoltage */
#define TLF35584_ABIST_SELECT2_VBATOV           (1 << 0)

/**@}*/

/** @defgroup reg_gtm GTM Register Flags
@{*/

/** Test mode inverted status */
#define TLF35584_GTM_NTM                        (1 << 1)

/** Test mode status */
#define TLF35584_GTM_TM                         (1 << 0)

/**@}*/

/** @defgroup reg_bck_freq_change BCK_FREQ_CHANGE Register Flags
@{*/

/** BUCK switching frequency change */
#define TLF35584_BCK_FREQ_CHANGE_NEG_4_5_PCT    0x7
#define TLF35584_BCK_FREQ_CHANGE_NEG_3_0_PCT    0x6
#define TLF35584_BCK_FREQ_CHANGE_NEG_1_5_PCT    0x5
#define TLF35584_BCK_FREQ_CHANGE_NO_CHANGE      0x4
#define TLF35584_BCK_FREQ_CHANGE_4_5_PCT        0x3
#define TLF35584_BCK_FREQ_CHANGE_3_0_PCT        0x2
#define TLF35584_BCK_FREQ_CHANGE_1_5_PCT        0x1
#define TLF35584_BCK_FREQ_CHANGE_NO_CHANGE2     0x0

/**@}*/

/** @defgroup reg_bck_fre_spread BCK_FRE_SPREAD Register Flags
@{*/

/** Spread spectrum */
#define TLF35584_BCK_FRE_SPREAD_NO_SPREAD       0x00
#define TLF35584_BCK_FRE_SPREAD_1_PCT           0x2B
#define TLF35584_BCK_FRE_SPREAD_2_PCT           0x55
#define TLF35584_BCK_FRE_SPREAD_3_PCT           0x80
#define TLF35584_BCK_FRE_SPREAD_4_PCT           0xAA
#define TLF35584_BCK_FRE_SPREAD_5_PCT           0xD5
#define TLF35584_BCK_FRE_SPREAD_6_PCT           0xFF

/**@}*/

/** @defgroup reg_bck_main_ctrl BCK_MAIN_CTRL Register Flags
@{*/

/** DATA_VALID parameter update ready status */
#define TLF35584_BCK_MAIN_CTRL_BUSY             (1 << 7)

/** Enable buck update */
#define TLF35584_BCK_MAIN_CTRL_DATA_VALID       (1 << 6)

/**@}*/

#endif /* HW_TLF35584_SAFETY_PSU_H */
