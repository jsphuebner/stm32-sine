#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#if CONTROL == CTRL_SINE
#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC
#elif CONTROL == CTRL_FOC
#define NUM_SAMPLES 3
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC
#endif // CONTROL

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOC, 1) \
   ANA_IN_ENTRY(throttle2, GPIOC, 0) \
   ANA_IN_ENTRY(udc,       GPIOC, 3) \
   ANA_IN_ENTRY(tmpm,      GPIOC, 2) \
   ANA_IN_ENTRY(tmphs,     GPIOC, 4) \
   ANA_IN_ENTRY(uaux,      GPIOA, 3) \
   ANA_IN_ENTRY(il1,       GPIOA, 5) \
   ANA_IN_ENTRY(il2,       GPIOB, 0)

//Alternative list. Must contain exactly the same names and number of
//entries as ANA_IN_LIST but may contain different IO pins

#define ANA_IN_LIST_BLUEPILL \
   ANA_IN_ENTRY(throttle1, GPIOA, 0) \
   ANA_IN_ENTRY(throttle2, GPIOA, 1) \
   ANA_IN_ENTRY(udc,       GPIOA, 2) \
   ANA_IN_ENTRY(tmpm,      GPIOA, 3) \
   ANA_IN_ENTRY(tmphs,     GPIOA, 4) \
   ANA_IN_ENTRY(uaux,      GPIOB, 1) \
   ANA_IN_ENTRY(il1,       GPIOA, 5) \
   ANA_IN_ENTRY(il2,       GPIOB, 0)

#endif // ANAIN_PRJ_H_INCLUDED
