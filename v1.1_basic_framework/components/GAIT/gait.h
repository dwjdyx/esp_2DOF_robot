#ifndef _GAIT_H_
#define _GAIT_H_

#include "leg.h"

typedef enum
{
    STAND,
    WALK,
    DANCE,
    SPIN
}gait_state_t;

typedef struct
{
    leg_pos_t leg_pos[LEG_MAX_NUM];
} gait_pose_t;

extern gait_state_t g_gait_state;

extern const gait_pose_t POSE_STAND;       //站立
extern const gait_pose_t POSE_WALK;        //行走
extern const gait_pose_t POSE_DANCE;       //跳舞
extern const gait_pose_t POSE_SPIN;        //旋转

void gait_pos_update(const gait_pose_t *pose, leg_t *leg_array);

#endif
