#include "leg_structure.h"

leg_t leg_array[LEG_MAX_NUM] = {
    { .initialized = false },
    { .initialized = false },
    { .initialized = false },
    { .initialized = false }
};

//映射表，根据实际情况修改offset和dir
const leg_config_t leg_config[LEG_MAX_NUM] =
{
    [RF] = {
        .hip = RF_HIP,
        .knee = RF_KNEE,
        .hip_offset = 90.0f,
        .knee_offset = 180.0f,
        .hip_dir = 1.0f,
        .knee_dir = 1.0f
    },

    [LF] = {
        .hip = LF_HIP,
        .knee = LF_KNEE,
        .hip_offset = 0.0f,
        .knee_offset = 90.0f,
        .hip_dir = 1.0f,
        .knee_dir = 1.0f
    },

    [RB] = {
        .hip = RB_HIP,
        .knee = RB_KNEE,
        .hip_offset = 0.0f,
        .knee_offset = 90.0f,
        .hip_dir = 1.0f,
        .knee_dir = 1.0f
    },

    [LB] = {
        .hip = LB_HIP,
        .knee = LB_KNEE,
        .hip_offset = 90.0f,
        .knee_offset = 180.0f,
        .hip_dir = 1.0f,
        .knee_dir = 1.0f
    }
};
