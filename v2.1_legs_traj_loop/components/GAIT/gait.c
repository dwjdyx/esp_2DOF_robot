#include "gait.h"
#include "servo.h"
#include "math.h"
#include "esp_log.h"

gait_action_t gait_state = ACT_WALK;
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
    .point = {
        {40.0f, 0.0f, -53.0f},
        {77.48f, 0.0f, -37.48f},
        {54.78f, 54.78f, -37.48f},
        {28.28f, 28.28f, -53.0f}
    },
    .duration = 1500.0f,     // ms
    .elapsed = 0.0f,
    .last_tick = 0,
    .running = true
};
const gait_traj_t TRAJ_SPIN =
{
    .point = {
        {40.0f, 0.0f, -53.0f},
        {77.48f, 0.0f, -37.48f},
        {54.78f, 54.78f, -37.48f},
        {28.28f, 28.28f, -53.0f}
    },
    .duration = 1500.0f,     // ms
    .elapsed = 0.0f,
    .last_tick = 0,
    .running = true
};

gait_pose_t g_gait_pose = POSE_STAND;
gait_traj_t g_gait_traj[LEG_MAX_NUM] = 
{
    TRAJ_WALK, 
    TRAJ_WALK, 
    TRAJ_WALK, 
    TRAJ_WALK
};

// ===================== 坐标变化需要的参数 ====================
traj_param_t g_param[LEG_MAX_NUM] = {

    {.initialized = {true, true, true, true}},
    {.initialized = {true, true, true, true}},
    {.initialized = {true, true, true, true}},
    {.initialized = {true, true, true, true}}

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
 * @brief 计算两坐标极角
 *
 * @param start_pos       起始坐标
 * @param end_pos         终止坐标
 * @param theta           极角
 * @param circel          圆心坐标
 */
static void polar_angle_cal(const leg_pos_t *start_pos, const leg_pos_t *end_pos, 
                            float *theta1, float *theta2,
                            float circle_x, float circle_y, float n) // 圆心坐标
{
//============ 以下计算公式根据实际修改 ==============
    float start_u, start_v, end_u, end_v;
    if(n == 1)      // knee移动
    {
        start_u = sqrtf(start_pos->x * start_pos->x + start_pos->y * start_pos->y);
        start_v = start_pos->z;

        end_u = sqrtf(end_pos->x * end_pos->x + end_pos->y * end_pos->y);
        end_v = end_pos->z;
    }else if(n == 2)    // hip移动
    {
        start_u = start_pos->x;
        start_v= start_pos->y;

        end_u = end_pos->x;
        end_v = end_pos->y;
    }

    // 计算局部坐标
    float u0 = start_u - circle_x;
    float v0 = start_v - circle_y;

    float u1 = end_u - circle_x;
    float v1 = end_v - circle_y;

    // 计算起点终点局部极角
    *theta1 = atan2f(v0, u0);
    *theta2 = atan2f(v1, u1);
}

// 检查dtheta变化
/*static void check_traj_dtheta(float *dtheta)
{
    if(*dtheta > M_PI) *dtheta -= 2*M_PI;
    else if (*dtheta < -M_PI) *dtheta += 2*M_PI;
    else return;
}*/

static void hip_move(const leg_pos_t *start_pos, const leg_pos_t *end_pos, 
                            leg_t *leg, traj_param_t *traj_param, int init_index)
{
    if(traj_param->initialized[init_index])
    {
        // 计算起点xy平面角度
        traj_param->theta_xy = atan2f(start_pos->y, start_pos->x);
        // 计算极角
        polar_angle_cal(start_pos, end_pos, 
                        &traj_param->theta1, &traj_param->theta2, 
                        hip_circle_x, hip_circle_y, 1);
        // 计算极角变化量
        traj_param->dtheta = traj_param->theta2 - traj_param->theta1;

        traj_param->initialized[init_index] = false;  //标志位置false，以上仅初始化一次
    }
    
    traj_param->alpha = traj_param->theta1 + traj_param->dtheta * traj_param->ratio;  // 计算角度变化量

    float R = 40.0f + 53.0f * cosf(traj_param->alpha);

    leg->target_pos.z = 53.0f * sinf(traj_param->alpha);
    leg->target_pos.x = R * cosf(traj_param->theta_xy);
    leg->target_pos.y = R * sinf(traj_param->theta_xy);
}

static void knee_move(const leg_pos_t *start_pos, const leg_pos_t *end_pos, 
                            leg_t *leg, traj_param_t *traj_param, int init_index)
{
    if(traj_param->initialized[init_index])
    {
        traj_param->R_xy = sqrtf(start_pos->x * start_pos->x + start_pos->y * start_pos->y);
        polar_angle_cal(start_pos, end_pos, 
                        &traj_param->theta1, &traj_param->theta2, 
                        knee_circle_x, knee_circle_y, 2);
        traj_param->dtheta = traj_param->theta2 - traj_param->theta1;

        traj_param->initialized[init_index] = false;
    }

    traj_param->alpha = traj_param->theta1 + traj_param->dtheta * traj_param->ratio;
    leg->target_pos.x = traj_param->R_xy * cosf(traj_param->alpha);
    leg->target_pos.y = traj_param->R_xy * sinf(traj_param->alpha);
    leg->target_pos.z = end_pos->z;
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
static void gait_traj_update(gait_traj_t *traj, leg_t *leg, traj_param_t *traj_param)
{
    if (traj == NULL || leg == NULL || traj_param == NULL)
        return;

    // 轨迹未运行，直接返回
    if (!traj->running)
        return;

    // 防止除零
    if (traj->duration <= 0.0f)
        return;
//================== 计算时间差 ===================
    TickType_t now = xTaskGetTickCount();

    float dt = (now - traj->last_tick) * (portTICK_PERIOD_MS); //转为ms

    traj->elapsed += dt;
    traj->last_tick = now;

    // 计算当前轨迹进度 t：0.0 ~ 1.0
    float t = traj->elapsed / traj->duration;

    // 限制 t 的范围
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f)  t = 1.0f;
//==================== 分时间段计算坐标 =================
    // 以下关系式/结果基本是根据实际预先算好的，不建议直接复用，根据实际修改
    if(t <= LIFT_TIME_RATIO)
    {
        //阶段1:原地抬腿 start → lift_start
        traj_param->ratio = t / LIFT_TIME_RATIO;  // 根据比例计算时间
        hip_move(&traj->point[0], &traj->point[1], leg, traj_param, 0);
    }
    else if(t <= LIFT_TIME_RATIO + MOVE_TIME_RATIO)
    {
        //阶段2:保持高度，水平移动 lift_start → lift_end
        traj_param->ratio = (t - LIFT_TIME_RATIO) / MOVE_TIME_RATIO;
        knee_move(&traj->point[1], &traj->point[2], leg, traj_param, 1);
    }
    else if(t <= LIFT_TIME_RATIO + MOVE_TIME_RATIO + DOWN_TIME_RATIO)
    {
        //阶段3:下降落地 lift_end → end
        traj_param->ratio =
            (t - LIFT_TIME_RATIO - MOVE_TIME_RATIO)
            / DOWN_TIME_RATIO;

        hip_move(&traj->point[2], &traj->point[3], leg, traj_param, 2);
    }
    else
    {
        //阶段4:回到起始点 end → start
        traj_param->ratio =
            (t - LIFT_TIME_RATIO - MOVE_TIME_RATIO - DOWN_TIME_RATIO)
            / BACK_TIME_RATIO;
        knee_move(&traj->point[3], &traj->point[0], leg, traj_param, 3);

    }
//===================== 打印坐标验证坐标拆解合理性 =====================
    /*ESP_LOGI("GAIT",
                    "target(%.1f, %.1f, %.1f)",
                    leg->target_pos.x,
                    leg->target_pos.y,
                    leg->target_pos.z);
    
    //检验坐标合理性
    if(!check_pos_validity(leg->target_pos.x, 
                           leg->target_pos.y, 
                           leg->target_pos.z))
    {
        ESP_LOGW("TASK", "目标点不合理，拒绝执行！");
        
    }*/

    // 到达终点
    if (traj->elapsed >= traj->duration)
    {
        leg->target_pos = traj->point[0];
        traj->elapsed = 0;
        traj->last_tick = now;
        // 进行循环
        for(int i = 0; i < MAX_WAYPOINT; i++)
        {
            traj_param->initialized[i] = true;
        }
    }
}

// ========================== 以下为gait_task调用函数，层级由上至下 ============================
// 1.根据不同姿态选择坐标更新函数
static void gait_update_target(void)
{
    switch (gait_state)
    {
        case ACT_STAND:
        case ACT_CREEP:
        {
            gait_pose_update(&g_gait_pose, leg_array);
            break;
        }
        case ACT_WALK:
        case ACT_SPIN:
        {
            for (leg_id_t i = RF; i < LEG_MAX_NUM; i++)
            {
                gait_traj_update(&g_gait_traj[i], &leg_array[i], &g_param[i]);
            }
            break;
        }
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
