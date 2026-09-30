#include "servo.h"

void servo_angle_init(void)
{
    servo_angle_set(RF_HIP, HIP_SERVO_INIT_ANGLE);
    servo_angle_set(RF_KNEE, RFLB_SERVO_INIT_ANGLE);
    servo_angle_set(LF_HIP, HIP_SERVO_INIT_ANGLE);
    servo_angle_set(LF_KNEE, LFRB_SERVO_INIT_ANGLE);
    servo_angle_set(RB_HIP, HIP_SERVO_INIT_ANGLE);
    servo_angle_set(RB_KNEE, LFRB_SERVO_INIT_ANGLE);
    servo_angle_set(LB_HIP, HIP_SERVO_INIT_ANGLE);
    servo_angle_set(LB_KNEE, RFLB_SERVO_INIT_ANGLE);
}

void servo_init(void)
{
    mypwm_init();
    //角度初始化
    servo_angle_init();
}

//角度映射占空比
void servo_angle_set(servo_id_t id, float theta)
{
    if (id >= SERVO_MAX)
        return;

    ledc_channel_t channel = servo_pwm_channel[id];

    if(theta > 180)
    {
        theta = 180;
    }else if(theta < 0)
    {
        theta = 0;
    }

    uint16_t duty = ((float)theta / 1800.0f + 0.025f) * 1024;
    pwm_duty_set(channel, duty);
}
