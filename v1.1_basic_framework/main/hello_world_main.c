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

#include "servo.h"
#include "leg.h"
#include "gait.h"
#include "test.h"

//static const char *TAG = "MAIN";

void gait_task(void *param)
{
    // 记录当前时间
    TickType_t last_log = xTaskGetTickCount();
    
    // 1.坐标更新
    gait_pos_update(&POSE_STAND, leg_array);
    // 2.计算theta并转换成舵机angle
    for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        // 计算theta
        bool ret = leg_control_task(&leg_array[i]);

        if (!ret)
        {
            ESP_LOGW("GAIT", "Leg %d control failed", i);
            continue;
        }
        // 转换成舵机angle
        theta_to_angle(leg_array[i].joint_angle.theta1, 
                       leg_array[i].joint_angle.theta2, 
                       i, 
                       &leg_array[i].servo_angle.hip_angle, &leg_array[i].servo_angle.knee_angle);
    }
    // 3.更新占空比
    for(leg_id_t j = RF; j < LEG_MAX_NUM; j++)
    {
        servo_angle_set(leg_config[j].hip, leg_array[j].servo_angle.hip_angle);
        servo_angle_set(leg_config[j].knee, leg_array[j].servo_angle.knee_angle);
        // 显示角度
        ESP_LOGI("SERVO",
                    "Leg %d: angle(%.1f, %.1f)",
                    j,
                    leg_array[j].servo_angle.hip_angle,
                    leg_array[j].servo_angle.knee_angle);
    }

    while (1)
    {

        //数据打印
       /* if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(500))
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
        }*/

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
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
    