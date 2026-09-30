#include "test.h"
#include "leg.h"
#include "servo.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==================== 舵机转动方向测试 ====================
void servo_direction_test(void)
{
       for(servo_id_t i = RF_HIP; i < SERVO_MAX; i++)
       {
              for(int j = 30; j <= 180; j+=30)
              {
                     servo_angle_set(i, j);
                     vTaskDelay(pdMS_TO_TICKS(500));
              }
       }
}

// ==================== 运动学测试 ==========================
//测试运动学函数   正向测试
void leg_FKtest(void)
{
    leg_joint_angle_t input = {
        .theta1 = 45.0f,
        .theta2 = -45.0f,
        .theta3 = 1.69f
    };

    leg_pos_t pos;
    leg_joint_angle_t output;

    leg_FK(&pos,&input);
    leg_IK(&pos,&output);

    printf("input: %.2f %.2f\n", input.theta1, input.theta2);

    printf("output: %.2f %.2f\n", output.theta1, output.theta2);

    printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);
}
//逆向测试
void leg_IKtest(void)
{
    leg_pos_t pos = {
        .x = 28.3f,
        .y = 28.3f,
        .z = -53.0f
    };

    leg_pos_t test_pos;
    leg_joint_angle_t output;

    leg_IK(&pos,&output);
    leg_FK(&test_pos,&output);

    printf("test_pos: %.2f %.2f %.2f\n", test_pos.x,test_pos.y,test_pos.z);
           
    printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);

    printf("output: %.2f %.2f %.2f\n", output.theta1, output.theta2, output.theta3);

}