#include "servo.h"
#include "leg.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG_IK = "IK";
static const char *TAG_FK = "FK";

#define FK_EPSILON 0.5f // 允许最大 0.5mm 的位置偏差


//检验IK角度合理性
bool check_angle_validity(const leg_joint_angle_t *angle) 
{
    if (angle == NULL) return false;

    // ================== 检查是否有任何一个角度变成了 NaN 或 Inf ================
    if (isnan(angle->theta1) || isnan(angle->theta2) || isnan(angle->theta3) ||
        isinf(angle->theta1) || isinf(angle->theta2) || isinf(angle->theta3)) {
        ESP_LOGE(TAG_IK, "数学运算无效 (NaN 或 Inf)");
        return false;
    }

    // =============== 检查角度是否超出限制范围 ==================
    if (angle->theta1 < THETA1_MIN || angle->theta1 > THETA1_MAX) {
        ESP_LOGW(TAG_IK, "Theta1 超限: %.2f", angle->theta1);
        return false;
    }
    if (angle->theta2 < THETA2_MIN || angle->theta2 > THETA2_MAX) {
        ESP_LOGW(TAG_IK, "Theta2 超限: %.2f", angle->theta2);
        return false;
    }

    return true;
}
// =============== 检查单周期允许的最大角度变化量 ====================
bool check_angle_delta(const leg_joint_angle_t *angle,
                       const leg_joint_angle_t *last_angle)
{
    if (angle == NULL || last_angle == NULL)
        return false;

    if (fabsf(angle->theta1 - last_angle->theta1) > MAX_DELTA_ANGLE ||
        fabsf(angle->theta2 - last_angle->theta2) > MAX_DELTA_ANGLE) {

        ESP_LOGW(TAG_IK, "单周期角度突变过大！");
        return false;
    }

    return true;
}

/**
 * @brief  腿部正运动学 (FK) 验证
 * @param  actual_pos 计算得到的实际末端坐标
 * @param  leg_angle  当前输入的关节角度
 * @return
 */
void leg_FK(leg_pos_t *actual_pos, const leg_joint_angle_t *leg_angle)
{
    if (actual_pos == NULL || leg_angle == NULL) return;

    float t1 = leg_angle->theta1 * DEG_TO_RAD;
    float t2 = leg_angle->theta2 * DEG_TO_RAD;

    float c1 = cosf(t1);
    float s1 = sinf(t1);
    float c2 = cosf(t2);
    float s2 = sinf(t2);

    actual_pos->x = c1 * c2 * L2 + d3 * s1 + L1 * c1 + d2 * s1;
    actual_pos->y = s1 * c2 * L2 - c1 * d3 + L1 * s1 - d2 * c1;
    actual_pos->z = s2 * L2 + d1;
}

// FK 结果精度检验
bool check_fk_precision(const leg_pos_t *target_pos, const leg_joint_angle_t *angle) {
    if (target_pos == NULL || angle == NULL) return false;

    leg_pos_t calc_pos;
    
    // 用 IK 解出的角度反算 FK 得到位置
    leg_FK(&calc_pos, angle);
    
    // 计算欧式距离平方: (dx^2 + dy^2 + dz^2)
    float dx = target_pos->x - calc_pos.x;
    float dy = target_pos->y - calc_pos.y;
    float dz = target_pos->z - calc_pos.z;
    float error_sq = dx * dx + dy * dy + dz * dz;
    
    // 偏差超过阈值平方
    if (error_sq > (FK_EPSILON * FK_EPSILON)) {
        ESP_LOGE(TAG_FK, "FK 验证偏差超过阀值: error_sq=%.2f", error_sq);
        return false;
    }
    
    return true;
}

/**
 * @brief  腿部逆运动学 (IK)
 * @param  target_pos 目标脚尖坐标 (x, y, z)
 * @param  leg_angle    输出关节角度 (theta1, theta2, theta3)
 * @return bool         解算成功返回 true，超工作空间返回 false
 */
bool leg_IK(const leg_pos_t *target_pos, leg_joint_angle_t *leg_angle)
{
    if (target_pos == NULL || leg_angle == NULL) return false;

    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;
    float d23 = d2 + d3;
    float xy_sq = x * x + y * y;

    // 工作空间安全校验：根号内不能为负数
    if (xy_sq < (d23 * d23)) {
        ESP_LOGE(TAG_IK, "目标点过近，根号内为负数");
        return false; 
    }

    // 第一关节
    float t1 = atan2f(y, x) - atan2f(-d23, sqrtf(xy_sq - d23 * d23));
    leg_angle->theta1 = t1 * RAD_TO_DEG;

    // 第二关节
    float t2 = atan2f(z - d1, cosf(t1) * x + sinf(t1) * y - L1);
    leg_angle->theta2 = t2 * RAD_TO_DEG;

    return true;
}

//腿部控制任务
bool leg_control_task(leg_t *g_leg) 
{
    if (g_leg == NULL) return false;

    // 使用局部临时变量接收 IK 解算结果（防止脏数据写入）
    leg_joint_angle_t calc_angle;

    // 求解 IK
    if (!leg_IK(&g_leg->target_pos, &calc_angle)) {
        ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
        return false;
    }
    
    // 检查基础角度合法性
    if (!check_angle_validity(&calc_angle)) {
        ESP_LOGW("TASK", "IK 角度非法，阻断输出！");
        return false;
    }
    // 第一次不检查角度突变
    if (g_leg->initialized)
    {
        if (!check_angle_delta(&calc_angle,
                               &g_leg->last_joint_angle)) {
            ESP_LOGW("TASK", "IK 角度突变过大，阻断输出！");
            return false;
        }
    }
    // 检查FK精度
    if (!check_fk_precision(&g_leg->target_pos, &calc_angle)) {
        ESP_LOGW("TASK", "FK 精度校验不通过，阻断输出！");
        return false;
    }
    
    // 校验全部通过，才真正更新全局结构体的数据
    g_leg->joint_angle = calc_angle;

    // 更新 last_joint_angle 供下一周期比对，并输出 PWM
    g_leg->last_joint_angle = g_leg->joint_angle;
    g_leg->initialized = true;
    
    return true;
}
//关节角转换舵机角度
void theta_to_angle(float theta1, float theta2, leg_id_t leg_id)
{
    if (leg_id >= LEG_MAX_NUM)
        return;

    const leg_config_t *cfg = &leg_config[leg_id];

    float hip_angle =
        cfg->hip_offset + cfg->hip_dir * theta1;

    float knee_angle =
        cfg->knee_offset + cfg->knee_dir * theta2;

    ESP_LOGI("servo",
                    "Leg %d: angle(%.1f, %.1f)",
                    leg_id,
                    hip_angle,
                    knee_angle);

    servo_angle_set(cfg->hip, hip_angle);
    servo_angle_set(cfg->knee, knee_angle);
}
