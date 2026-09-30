#ifndef _MYPWM_H_
#define _MYPWM_H_

#include "stdint.h"
#include "driver/ledc.h"

//舵机引脚
#define   RF_HIP_SERVO    GPIO_NUM_16     // RF_HIP
#define   RF_KNEE_SERVO   GPIO_NUM_18     // RF_KNEE
#define   LF_HIP_SERVO    GPIO_NUM_12     // LF_HIP
#define   LF_KNEE_SERVO   GPIO_NUM_25     // LF_KNEE
#define   RB_HIP_SERVO    GPIO_NUM_27     // RB_HIP
#define   RB_KNEE_SERVO   GPIO_NUM_19     // RB_KNEE
#define   LB_HIP_SERVO    GPIO_NUM_13     // LB_HIP
#define   LB_KNEE_SERVO   GPIO_NUM_26     // LB_KNEE


void mypwm_init(void);
void pwm_duty_set(ledc_channel_t channel, uint16_t duty);

#endif
