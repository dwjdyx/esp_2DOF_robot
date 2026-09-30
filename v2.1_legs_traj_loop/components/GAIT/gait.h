#ifndef _GAIT_H_
#define _GAIT_H_

#include "leg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_WAYPOINT 4      // 最大轨迹点数量

//圆心坐标，根据实际修改
#define  hip_circle_x  40
#define  hip_circle_y  0
#define  knee_circle_x  0
#define  knee_circle_y  0

// 行走姿态 抬脚-平移-落地-回到起始点四个阶段时间比例
#define LIFT_TIME_RATIO   0.25f
#define MOVE_TIME_RATIO   0.25f
#define DOWN_TIME_RATIO   0.25f
#define BACK_TIME_RATIO   0.25f

typedef enum
{
    ACT_STAND,  //站立
    ACT_CREEP,  //匍匐
    ACT_WALK,   //行走
    ACT_SPIN    //旋转
}gait_action_t;      // 动作姿态

typedef struct
{
    leg_pos_t leg_pos[LEG_MAX_NUM]; 
}gait_pose_t;      // 静态动作坐标

typedef enum
{
    TRAJ_LIFT,      // 抬腿 hip舵机
    TRAJ_MOVE,      // 平移 knee舵机
    TRAJ_DOWN,      // 落下
    TRAJ_BACK
} traj_phase_t;     //不同阶段

typedef struct 
{
    const leg_pos_t point[MAX_WAYPOINT];    // 过程点坐标集

    float duration;         // 整个轨迹持续时间，单位：ms
    float elapsed;          // 当前已经经过的时间，单位：ms
    TickType_t  last_tick;  // 上一次更新时间
    
    bool running;           // 是否正在运行

}gait_traj_t;              //动态轨迹

typedef struct
{
    float theta1;       // 极角1
    float theta2;       // 极角2
    float dtheta;       // 极角变化量

    float theta_xy;     // knee舵机移动xy形成的角度
    float R_xy;         // hip舵机移动xy形成的半径
 
    float ratio;        // 时间变化量
    float alpha;        // 角度变化量
    bool initialized[MAX_WAYPOINT];   // 是否执行标志位 对应每个过程点切换阶段

}traj_param_t;

extern const gait_pose_t POSE_STAND;      //站立
extern const gait_pose_t POSE_CREEP;      //匍匐
extern const gait_traj_t TRAJ_WALK;       //行走
extern const gait_traj_t TRAJ_SPIN;       //旋转

extern gait_action_t gait_state;           // 动作
extern gait_pose_t g_gait_pose;
extern gait_traj_t g_gait_traj[LEG_MAX_NUM];
extern traj_param_t g_param[LEG_MAX_NUM];

void gait_task(void *param);
#endif
