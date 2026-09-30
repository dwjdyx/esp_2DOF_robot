#ifndef _LEG_H_
#define _LEG_H_

#include <stdbool.h>
#include "leg_structure.h"

// ============== 定义关节软限位（根据机器蜘蛛实际结构调整）=============
#define THETA1_MIN  -180.0f
#define THETA1_MAX  180.0f

#define THETA2_MIN  -180.0f
#define THETA2_MAX  180.0f

#define THETA3_MIN  -180.0f
#define THETA3_MAX  180.0f

#define MAX_DELTA_ANGLE 15.0f // 单周期允许的最大角度变化量 (度)

// ================ 数学参数 =====================
#define PI 3.14159265358979323846f
#define RAD_TO_DEG  (180.0f / PI)   //弧度转角度
#define DEG_TO_RAD  (PI / 180.0f)   //角度转弧度

//================ DH参数 ====================
#define L0 0.0f
#define L1 40.0f
#define L2 53.0f

#define d1 0.0f
#define d2 27.0f
#define d3 -27.0f

//======================== 函数 ==============================
bool leg_IK(const leg_pos_t *target_pos, leg_joint_angle_t *leg_angle);
void leg_FK(leg_pos_t *actual_pos, const leg_joint_angle_t *leg_angle);
bool leg_control_task(leg_t *g_leg);
void theta_to_angle(float theta1, float theta2, leg_id_t leg_id);

#endif
