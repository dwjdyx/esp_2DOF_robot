#include "gait.h"

gait_state_t g_gait_state = STAND;

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
const gait_pose_t POSE_WALK =
{
    .leg_pos =
    {
        [RF] = {80, -40, -60},
        [LF] = {80,  40, -60},
        [LB] = {80,  40, -60},
        [RB] = {80, -40, -60}
    }
};
const gait_pose_t POSE_DANCE =
{
    .leg_pos =
    {
        [RF] = {80, -40, -60},
        [LF] = {80,  40, -60},
        [LB] = {80,  40, -60},
        [RB] = {80, -40, -60}
    }
};
const gait_pose_t POSE_SPIN =
{
    .leg_pos =
    {
        [RF] = {80, -40, -60},
        [LF] = {80,  40, -60},
        [LB] = {80,  40, -60},
        [RB] = {80, -40, -60}
    }
};

//坐标更新
void gait_pos_update(const gait_pose_t *pose, leg_t *leg_array)
{
    for(leg_id_t i = RF; i < LEG_MAX_NUM; i++)
    {
        leg_array[i].target_pos = pose->leg_pos[i];
    }
}
