#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO2,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(start_in,    GPIOD, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,    GPIOE, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(mprot_in,    GPIOE, GPIO5,  PinMode::INPUT_PU)    \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,      GPIOA, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bk_in,       GPIOB, GPIO12, PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(ocur_in,     GPIOE, GPIO2,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(desat_in,    GPIOC, GPIO9,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(fan_out,     GPIOC, GPIO15, PinMode::OUTPUT)  /* map to unused pin by default */    \
    DIG_IO_ENTRY(vtg_out,     GPIOC, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOD, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOD, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(temp0_out,   GPIOD, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(speed_out,   GPIOD, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(brk_out,     GPIOD, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs1_hi,      GPIOE, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs2_hi,      GPIOE, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs3_hi,      GPIOE, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs1_lo,      GPIOE, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs2_lo,      GPIOE, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(cs3_lo,      GPIOE, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(v5_ctrl,     GPIOD, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(intb_in,     GPIOE, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(inta_in,     GPIOE, GPIO9,  PinMode::INPUT_FLT)   \


#define DIG_IO_BLUEPILL \
    DIG_IO_ENTRY(brake_in,    GPIOB, GPIO9,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,      GPIOB, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,      GPIOB, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(mprot_in,    GPIOB, GPIO1,  PinMode::INPUT_FLT)    \
    DIG_IO_ENTRY(emcystop_in, GPIOB, GPIO1,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(bk_in,       GPIOB, GPIO12, PinMode::INPUT_PU)   \
    DIG_IO_ENTRY(bms_in,      GPIOD, GPIO15, PinMode::INPUT_FLT) /* non-existant */  \
    DIG_IO_ENTRY(ocur_in,     GPIOD, GPIO15, PinMode::INPUT_FLT) /* non-existant */  \
    DIG_IO_ENTRY(desat_in,    GPIOD, GPIO15, PinMode::INPUT_FLT) /* non-existant */  \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(fan_out,     GPIOA, GPIO0,  PinMode::OUTPUT)  /* map to unused pin by default */    \
    DIG_IO_ENTRY(vtg_out,     GPIOD, GPIO14, PinMode::OUTPUT) /* non-existant */     \
    DIG_IO_ENTRY(prec_out,    GPIOB, GPIO4,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOD, GPIO14, PinMode::OUTPUT) /* non-existant */     \
    DIG_IO_ENTRY(temp0_out,   GPIOD, GPIO14, PinMode::OUTPUT) /* non-existant */     \
    DIG_IO_ENTRY(speed_out,   GPIOD, GPIO14, PinMode::OUTPUT) /* non-existant */     \
    DIG_IO_ENTRY(brk_out,     GPIOD, GPIO14, PinMode::OUTPUT) /* non-existant */     \


/*    DIG_IO_ENTRY(test1_out,   GPIOB, GPIO3,  PinMode::OUTPUT)     \
    DIG_IO_ENTRY(test2_out,   GPIOA, GPIO13, PinMode::OUTPUT)     \
    DIG_IO_ENTRY(test3_out,   GPIOA, GPIO14, PinMode::OUTPUT)     \
    DIG_IO_ENTRY(test4_out,   GPIOA, GPIO15, PinMode::OUTPUT)     \*/

#define temp1_out bms_in  //is configured to output dynamically on Tesla hardware
#define spi_cs_out vtg_out

#endif // PinMode_PRJ_H_INCLUDED
