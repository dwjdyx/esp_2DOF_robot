#ifndef _LEG_STRUCTURE_H_
#define _LEG_STRUCTURE_H_

#include "servo.h"

// ================ 腿部数据结构体 ==================
typedef enum
{
    RF,                //右前腿
    LF,                //左前腿
    RB,                //右后腿
    LB,                //左后腿
    LEG_MAX_NUM
}leg_id_t;

typedef struct
{
    float x;
    float y;
    float z;
}leg_pos_t;             //腿部坐标

typedef struct
{
    float theta1;       //臀部舵机
    float theta2;       //膝盖舵机
    float theta3;       //末端固定常量值
}leg_joint_angle_t;     //关节角度

typedef struct 
{
    leg_pos_t target_pos;                   //目标坐标
    leg_joint_angle_t last_joint_angle;     //上次关节角度
    leg_joint_angle_t joint_angle;          //关节角度
    bool initialized;
}leg_t;             //腿部结构体

typedef struct
{
    servo_id_t hip;
    servo_id_t knee;

    float hip_offset;
    float knee_offset;

    float hip_dir;
    float knee_dir;

} leg_config_t;         //腿和舵机映射

extern const leg_config_t leg_config[LEG_MAX_NUM];
extern leg_t leg_array[LEG_MAX_NUM];        //腿部数组

#endif
