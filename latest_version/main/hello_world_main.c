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
#include "nvs_flash.h"

#include "ble.h"
#include "servo.h"
#include "gait.h"
#include "test.h"

void app_main(void)
{
    nvs_flash_init();
    ble_init();
    servo_init();
// ================== 验证FKIK ======================
//    leg_FKtest();
//    leg_IKtest();
// =====================================================

//    xTaskCreatePinnedToCore(oled_task,           //函数名称
//                            "oled_task",         //任务名称
//                            2048,               //堆栈大小
//                            NULL,               //传入参数
//                            1,                  //任务优先级
//                            NULL,               //任务句柄
//                            0 );                //分配内核

    xTaskCreatePinnedToCore(gait_task, "gait_task", 4096, NULL, 1, NULL, 0 );
}
    