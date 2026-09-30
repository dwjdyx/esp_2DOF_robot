#include "gait.h"
#include "servo.h"
#include "math.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

gait_state_t gait_state = CREEP;
// ====================== 预设姿态坐标，根据实际修改 =====================
const gait_pose_t POSE_STAND =
{
    .leg_pos =
    {
        [RF] = {40, 0, -53},
        [LF] = {40, 0, -53},
        [LB] = {40, 0, -53},
        [RB] = {40, 0, -53}
    }
};
const gait_pose_t POSE_CREEP =
{
    .leg_pos =
    {
        [RF] = {93, 0, 0},
        [LF] = {93, 0, 0},
        [LB] = {40, 0, -53},
        [RB] = {40, 0, -53}
    }
};
gait_traj_t TRAJ_WALK =
{
    .end = {20*1.7, 20*1.7, -53},
    .lift_height = -10,
    .duration = 0.1f,
    .running = false
};
gait_traj_t TRAJ_SPIN =
{
    .end = {20*1.7, 20*1.7, -53},
    .lift_height = -23,
    .duration = 0.1f,
    .running = false
};

// ===================== 坐标更新 =========================
/**
 * @brief 静态动作坐标更新
 *
 * 预设坐标直接赋值
 * 
 * @param pose            预设好的坐标
 * @param leg_array       腿部数组
 */
static void gait_pose_update(const gait_pose_t *pose, leg_t *leg_array)
{
    for(leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        leg_array[i].target_pos = pose->leg_pos[i];
    }
}

/**
 * @brief 更新足端轨迹坐标
 *
 * 根据当前轨迹运行时间，计算足端当前应该到达的坐标。
 * 当前采用三维空间中的线性插值。
 *
 * @param traj      轨迹参数
 * @param leg       需要更新的腿
 */
static void gait_traj_update(gait_traj_t *traj, leg_t *leg)
{
    if (traj == NULL || leg == NULL)
        return;

    // 轨迹未运行，直接返回
    if (!traj->running)
        return;

    // 防止除零
    if (traj->duration <= 0.0f)
        return;

    // 计算当前轨迹进度 t：0.0 ~ 1.0
    float t = traj->elapsed / traj->duration;

    // 限制 t 的范围
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f)  t = 1.0f;
    
    // 三维坐标线性插值
    if(t <= LIFT_TIME_RATIO)
    {
        //阶段1:原地抬腿
        float ratio = t / LIFT_TIME_RATIO;

        leg->target_pos.x = traj->start.x;
        leg->target_pos.y = traj->start.y;

        leg->target_pos.z =
            traj->start.z +
            traj->lift_height * ratio;
    }
    else if(t <= LIFT_TIME_RATIO + MOVE_TIME_RATIO)
    {
        //阶段2:保持高度，水平移动
        float ratio = (t - LIFT_TIME_RATIO) / MOVE_TIME_RATIO;

        leg->target_pos.x =
            traj->start.x +
            (traj->end.x - traj->start.x)
            * ratio;

        leg->target_pos.y =
            traj->start.y +
            (traj->end.y - traj->start.y)
            * ratio;

        leg->target_pos.z =
            traj->start.z +
            traj->lift_height;
    }else
    {
        //阶段3:下降落地
        float ratio =
            (t - LIFT_TIME_RATIO - MOVE_TIME_RATIO)
            / DOWN_TIME_RATIO;


        leg->target_pos.x = traj->end.x;
        leg->target_pos.y = traj->end.y;

        leg->target_pos.z = traj->start.z + traj->lift_height
            - traj->lift_height * ratio;
    }

    // 到达终点
    if (traj->elapsed >= traj->duration)
    {
        leg->target_pos = traj->end;
        traj->running = false;
    }
}

// ========================== 以下为gait_task调用函数，层级由上至下 ============================
// 1.根据不同姿态选择坐标更新函数
static void gait_update_target(void)
{
    switch (gait_state)
    {
        case STAND:
            gait_pose_update(&POSE_STAND, leg_array);
            break;

        case CREEP:
            gait_pose_update(&POSE_CREEP, leg_array);
            break;

        case WALK:
            gait_traj_update(&TRAJ_WALK, &leg_array[0]);
            break;

        case SPIN:
            gait_traj_update(&TRAJ_SPIN, &leg_array[0]);
            break;

        default:
            break;
    }
}
// 2.更新关节角并转换成舵机角
static void gait_update_joint(void)
{
    for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        // 计算关节角
        if (!leg_control_task(&leg_array[i]))
        {
            ESP_LOGW("GAIT", "Leg %d control failed", i);
            continue;
        }
        // 角度转换
        theta_to_angle(leg_array[i].joint_angle.theta1,
                       leg_array[i].joint_angle.theta2,
                       i,
                       &leg_array[i].servo_angle.hip_angle,
                       &leg_array[i].servo_angle.knee_angle);
    }
}
// 3.统一输入占空比
static void gait_update_servo(void)
{
    for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        servo_angle_set(leg_config[i].hip,
                        leg_array[i].servo_angle.hip_angle);

        servo_angle_set(leg_config[i].knee,
                        leg_array[i].servo_angle.knee_angle);
    }
}
// ====================== 任务函数 ===========================
void gait_task(void *param)
{
    while (1)
    {
        // 记录当前时间
        TickType_t last_log = xTaskGetTickCount();

        // 1. 更新足端目标坐标
        gait_update_target();

        // 2. 足端坐标 -> 关节角 -> 舵机角
        gait_update_joint();

        // 3. 舵机角 -> PWM
        gait_update_servo();
        
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
                         leg_array[i].servo_angle.hip_angle,
                         leg_array[i].servo_angle.knee_angle);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
