#ifndef _SERVO_H_
#define _SERVO_H_

#include "stdint.h"
#include "mypwm.h"

#define SERVO_INIT_ANGLE   90.0f    //舵机初始角度

typedef enum
{
    RF_HIP,
    RF_KNEE,

    LF_HIP,
    LF_KNEE,

    RB_HIP,
    RB_KNEE,

    LB_HIP,
    LB_KNEE,

    SERVO_MAX
}servo_id_t;

//舵机对应pwm通道
static const ledc_channel_t servo_pwm_channel[SERVO_MAX]=
{
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
    LEDC_CHANNEL_7
};

void servo_angle_init(void);
void servo_init(void);
void servo_angle_set(servo_id_t id, float theta);

#endif
