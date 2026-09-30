#include "gait.h"
#include "servo.h"
#include "math.h"
#include "esp_log.h"

gait_state_t gait_state = WALK;
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
const gait_traj_t TRAJ_WALK =
{
    .traj_pos = {
        .start      = {40.0f, 0.0f, -53.0f},
        .lift_start = {77.48f, 0.0f, -37.48f},
        .lift_end   = {54.78f, 54.78f, -37.48f},
        .end        = {28.28f, 28.28f, -53.0f}
    },
    
    .traj_state = {
        .duration = 1000.0f,     // ms
        .elapsed = 0.0f,
        .last_tick = 0,
        .running = true
    }
};
const gait_traj_t TRAJ_SPIN =
{
    .traj_pos = {
        .start      = {40.0f, 0.0f, -53.0f},
        .lift_start = {77.48f, 0.0f, -37.48f},
        .lift_end   = {54.78f, 54.78f, -37.48f},
        .end        = {28.28f, 28.28f, -53.0f}
    },
    
    .traj_state = {
        .duration = 1000.0f,
        .elapsed = 0.0f,
        .last_tick = 0,
        .running = true
    }
};

gait_pose_t g_gait_pose = POSE_STAND;
gait_traj_t g_gait_traj = TRAJ_WALK;

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

//计算坐标随时间变化关系式，平面直角坐标系转换三轴
/*static void gait_trochoid_update(float cx, float cy, float x0, float y0, float x1, float y1, float T)
{
    float dt;
    // 坐标平移到圆心点
    float x2 = x0 - cx;
    float y2 = y0 - cy;
    float x3 = x1 - cx;
    float y3 = y1 - cy;

    // 计算起点终点极角
    float theta1 = atan2f(y2, x2);
    float theta2 = atan2f(y3, x3);

    // 计算圆心角增量
    float dtheta = theta2 - theta1;
    if(dtheta >= -PI && dtheta <= PI) dtheta = dtheta; 
    else if(dtheta > PI) dtheta = dtheta - 2*PI;
    else if (dtheta < -PI) dtheta = dtheta + 2*PI;

    //计算角速度
    float theta_v = dtheta / T;
    float theta_t = theta1 + theta_v * dt;
    
    // 计算关系式
    float x_t = L2*cosf(theta_t) + cx;
    float y_t = L2*sinf(theta_t) + cy;
}*/

/**
 * @brief 更新足端轨迹坐标
 *
 * 根据当前轨迹运行时间，计算足端当前应该到达的坐标。
 * 当前采用三维空间中的线性插值。
 *
 * @param traj      轨迹参数
 * @param leg       需要更新的腿
 */
static void gait_traj_update(traj_state_t *traj, leg_t *leg, const leg_pos_t *end_pos)
{
    if (traj == NULL || leg == NULL)
        return;

    // 轨迹未运行，直接返回
    if (!traj->running)
        return;

    // 防止除零
    if (traj->duration <= 0.0f)
        return;
    
    TickType_t now = xTaskGetTickCount();

    float dt = (now - traj->last_tick) * (portTICK_PERIOD_MS); //转为ms

    traj->elapsed += dt;
    traj->last_tick = now;

    // 计算当前轨迹进度 t：0.0 ~ 1.0
    float t = traj->elapsed / traj->duration;

    // 限制 t 的范围
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f)  t = 1.0f;
    
    // 以下所有关系式/结果都是根据实际预先算好的，不建议复用，根据实际修改
    // 关系式计算可以用以上函数(例子)
    if(t <= LIFT_TIME_RATIO)
    {
        //阶段1:原地抬腿 start → lift_start
        float ratio = t / LIFT_TIME_RATIO;

        float alpha = -M_PI / 2.0f + (M_PI / 4.0f) * ratio;
        float R = 40.0f + 53.0f * cosf(alpha);
        leg->target_pos.z = 53.0f * sinf(alpha);

        leg->target_pos.x = R;
        leg->target_pos.y = 0.0f;
    }
    else if(t <= LIFT_TIME_RATIO + MOVE_TIME_RATIO)
    {
        //阶段2:保持高度，水平移动 lift_start → lift_end
        float ratio = (t - LIFT_TIME_RATIO) / MOVE_TIME_RATIO;

        // t ∈ [0, T]
        float theta = (M_PI / 4.0f) * ratio;
        leg->target_pos.x = 77.48f * cosf(theta);
        leg->target_pos.y = 77.48f * sinf(theta);
        leg->target_pos.z = -37.48f;
    }
    else
    {
        //阶段3:下降落地 lift_end → end
        float ratio =
            (t - LIFT_TIME_RATIO - MOVE_TIME_RATIO)
            / DOWN_TIME_RATIO;

        float alpha = -M_PI/4.0f - (M_PI/4.0f)* ratio;
        float R = 40.0f + 53.0f * cosf(alpha);

        leg->target_pos.z = 53.0f * sinf(alpha);
        leg->target_pos.x = leg->target_pos.y = R / sqrtf(2);
    }
//===================== 打印坐标进行验证 =====================
/*    ESP_LOGI("GAIT",
         "running=%d elapsed=%.1f dt=%.1f t=%.2f",
         traj->running,
         traj->elapsed,
         dt,
         t);
    
    ESP_LOGI("GAIT",
                    "target(%.1f, %.1f, %.1f)",
                    leg->target_pos.x,
                    leg->target_pos.y,
                    leg->target_pos.z);
*/
    
    //检验坐标合理性
    if(!check_pos_validity(leg->target_pos.x, 
                           leg->target_pos.y, 
                           leg->target_pos.z))
    {
        ESP_LOGW("TASK", "目标点不合理，拒绝执行！");
        
    }

    // 到达终点
    if (traj->elapsed >= traj->duration)
    {
        leg->target_pos = *end_pos;
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
        case CREEP:
            gait_pose_update(&g_gait_pose, leg_array);
            break;

        case WALK:
        case SPIN:
            gait_traj_update(&g_gait_traj.traj_state, &leg_array[0], &g_gait_traj.traj_pos.end);
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
    // 记录当前时间
    TickType_t last_log = xTaskGetTickCount();

    while (1)
    {
//        gait_update_target();

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

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
