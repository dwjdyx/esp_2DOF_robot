/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "stdint.h"

//#include "oled.h"
#include "servo.h"
#include "leg.h"
#include "gait.h"
#include "test.h"

//static const char *TAG = "MAIN";

/*
void oled_show(leg_id_t n)
{
    OLED_Printf(32*n,0,OLED_6X8,"%03.0f", leg_array[n].last_joint_angle.theta1);
    OLED_Printf(32*n,8,OLED_6X8,"%03.0f", leg_array[n].last_joint_angle.theta2);

    OLED_Printf(32*n,24,OLED_6X8,"%04.1f",  leg_array[n].target_pos.x);
    OLED_Printf(32*n,32,OLED_6X8,"%04.1f",  leg_array[n].target_pos.y);
    OLED_Printf(32*n,40,OLED_6X8,"%04.1f", leg_array[n].target_pos.z);
}

void oled_task(void * param)
{
    while(1)
    {
        for(leg_id_t i = RF; i < LEG_MAX_NUM; i++)
        {
            oled_show(i);
        }

        OLED_Update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}*/

void gait_task(void *param)
{
    TickType_t last_log = xTaskGetTickCount();
    
        gait_pos_update(&POSE_STAND, leg_array);

    for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        bool ret = leg_control_task(&leg_array[i]);

        if (!ret)
        {
            ESP_LOGW("GAIT", "Leg %d control failed", i);
            continue;
        }
        //theta_to_angle(leg_array[i].joint_angle.theta1, 
        //               leg_array[i].joint_angle.theta2, 
        //               i);
    }

    while (1)
    {

        //数据打印
        if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(500))
        {
            last_log = xTaskGetTickCount();

            for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
            {
                ESP_LOGI("GAIT",
                         "Leg %d: target(%.1f, %.1f, %.1f), theta(%.1f, %.1f)",
                         i,
                         leg_array[i].target_pos.x,
                         leg_array[i].target_pos.y,
                         leg_array[i].target_pos.z,
                         leg_array[i].joint_angle.theta1,
                         leg_array[i].joint_angle.theta2);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
//    oled_init();
    servo_init();

//    xTaskCreatePinnedToCore(oled_task,           //函数名称
//                            "oled_task",         //任务名称
//                            2048,               //堆栈大小
//                            NULL,               //传入参数
//                            1,                  //任务优先级
//                            NULL,               //任务句柄
//                            0 );                //分配内核

    xTaskCreatePinnedToCore(gait_task, "gait_task", 2048, NULL, 1, NULL, 0 );                     
}
    