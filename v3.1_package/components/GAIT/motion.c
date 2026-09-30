#include "motion.h"

gait_action_t gait_state = ACT_WALK;
// ====================== 预设姿态坐标，根据实际修改 =====================
const gait_pose_t POSE_STAND =
{
    .leg_pos =
    {
        [RF] = {40.0f, 0.0f, -53.0f},
        [LF] = {40.0f, 0.0f, -53.0f},
        [LB] = {28.28f, -28.28f, -53.0f},
        [RB] = {28.28f, 28.28f, -53.0f}
    }
};
const gait_pose_t POSE_CREEP =
{
    .leg_pos =
    {
        [RF] = {93.0f, 0.0f, 0.0f},
        [LF] = {93.0f, 0.0f, 0.0f},
        [LB] = {28.28f, -28.28f, -53.0f},
        [RB] = {28.28f, 28.28f, -53.0f}
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
    .time = {
        .duration = 1500.0f,     // ms
        .elapsed = 0.0f,
        .last_tick = 0,
    },
    .phase_param = {
        .calc = {{0},{0},{0},{0}},
        .cur_phase = TRAJ_PHASE_1,
        .initialized = {true, true, true, true}
    },
    .running = false
};
const gait_traj_t TRAJ_SPIN =
{
    .point = {
        {40.0f, 0.0f, -53.0f},
        {77.48f, 0.0f, -37.48f},
        {54.78f, 54.78f, -37.48f},
        {28.28f, 28.28f, -53.0f}
    },
    .time = {
        .duration = 1500.0f,     // ms
        .elapsed = 0.0f,
        .last_tick = 0,
    },
    .phase_param = {
        .calc = {{0},{0},{0},{0}},
        .cur_phase = TRAJ_PHASE_1,
        .initialized = {true, true, true, true}
    },
    .running = false
};


gait_pose_t g_gait_pose = POSE_STAND;
gait_traj_t g_gait_traj[LEG_MAX_NUM] = 
{
    TRAJ_WALK,
    TRAJ_WALK, 
    TRAJ_WALK, 
    TRAJ_WALK
};
