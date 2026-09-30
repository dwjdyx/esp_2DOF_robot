#ifndef _GAIT_H_
#define _GAIT_H_

#include "leg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 行走姿态 抬脚-平移-落地三个阶段时间比例
#define LIFT_TIME_RATIO   0.3f
#define MOVE_TIME_RATIO   0.4f
#define DOWN_TIME_RATIO   0.3f

typedef enum
{
    STAND,  //站立
    WALK,   //匍匐
    CREEP,  //行走
    SPIN    //旋转
}gait_state_t;      // 姿态

typedef struct
{
    leg_pos_t leg_pos[LEG_MAX_NUM]; 
} gait_pose_t;      // 静态动作坐标

typedef struct
{
    leg_pos_t start;        // 起始落脚点
    leg_pos_t lift_start;   // 抬脚最高点起始坐标
    leg_pos_t lift_end;     // 落脚起始坐标
    leg_pos_t end;          // 最终落脚点

} traj_pos_t;              // 动态轨迹坐标参数

typedef struct 
{
    float lift_height;      //抬脚高度

    float duration;         // 整个轨迹持续时间，单位：ms
    float elapsed;          // 当前已经经过的时间，单位：ms
    TickType_t  last_tick;  // 上一次更新时间
    
    bool running;           // 是否正在运行

}traj_state_t;              //动态轨迹

typedef struct
{
    const traj_pos_t traj_pos;
    traj_state_t traj_state;
}gait_traj_t;

extern const gait_pose_t POSE_STAND;      //站立
extern const gait_pose_t POSE_CREEP;      //匍匐
extern const gait_traj_t TRAJ_WALK;       //行走
extern const gait_traj_t TRAJ_SPIN;       //旋转

extern gait_state_t gait_state;
extern gait_pose_t g_gait_pose;
extern gait_traj_t g_gait_traj;

void gait_task(void *param);
#endif
