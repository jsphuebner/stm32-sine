#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

//Maximum value for over current limit timer
#define OCURMAX 4096
#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11

#define PWM_TIMER     TIM1
#define PWM_TIMRST    RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define REV_CNT_RCC_ENR    RCC_APB1ENR_TIM3EN
#define rev_timer_isr      tim3_isr
#define REV_CNT_TIMER      TIM3
#define REV_CNT_TIMRST     RST_TIM3
#define OVER_CUR_TIMER     hwRev == HW_BLUEPILL ? TIM2 : TIM4
#define OVER_CUR_NEG       hwRev == HW_BLUEPILL ? TIM_OC1 : TIM_OC2
#define OVER_CUR_POS       hwRev == HW_BLUEPILL ? TIM_OC2 : TIM_OC3

//Address of parameter block in flash
#define PARAM_BLKSIZE 1024
#define PARAM_BLKNUM  1   //last block of 1k
#define CAN_BLKSIZE   1024
#define CAN_BLKNUM    2   //second to last block of 1k

#define REV_CNT_IC         hwRev == HW_REV1 ? TIM_IC3 : TIM_IC1
#define REV_CNT_OC         hwRev == HW_REV1 ? TIM_OC3 : TIM_OC1
#define REV_CNT_CCR        hwRev == HW_REV1 ? TIM3_CCR3 : TIM3_CCR1
#define REV_CNT_CCR_PTR    hwRev == HW_REV1 ? (uint32_t)&TIM3_CCR3 : (uint32_t)&TIM3_CCR1
#define REV_CNT_SR         hwRev == HW_REV1 ? TIM_SR_CC3IF : TIM_SR_CC1IF
#define REV_CNT_DMAEN      hwRev == HW_REV1 ? TIM_DIER_CC3DE : TIM_DIER_CC1DE
#define REV_CNT_DMACHAN    hwRev == HW_REV1 ? DMA_CHANNEL2 : DMA_CHANNEL6

#define NORTH_EXC_PORT     hwRev == HW_BLUEPILL ? GPIOC : GPIOD
#define NORTH_EXC_PIN      hwRev == HW_BLUEPILL ? GPIO14 : GPIO2
#define NORTH_EXC_EXTI     hwRev == HW_BLUEPILL ? EXTI14 : EXTI2

typedef enum
{
   HW_REV1, HW_REV2, HW_REV3, HW_TESLA, HW_BLUEPILL, HW_PRIUS
} HWREV;

extern HWREV hwRev;

#endif // HWDEFS_H_INCLUDED
