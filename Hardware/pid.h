/**
 * @file pid.h
 * @brief 增量式PID控制器头文件
 * 
 * 结构体成员说明：
 * - Kp/Ki/Kd: PID比例、积分、微分系数
 * - target: 设定目标值（期望速度）
 * - feedback: 反馈值（实际测量速度）
 * - err: 当前误差 = target - feedback
 * - err_last: 上一次误差
 * - err_prev: 上上次误差（用于微分项）
 * - output: PID计算输出值
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

#pragma pack(4)
// 增量式 PID 结构体
typedef struct
{
    float Kp;        // 比例系数
    float Ki;        // 积分系数
    float Kd;        // 微分系数

    float target;    // 设定目标值
    float feedback;  // 反馈值（实测）
    float err;       // 当前误差
    float err_last;  // 上一次误差
    float err_prev;  // 上上次误差

    volatile float output;  // PID输出
} PID_HandleTypeDef;
#pragma pack()

// 函数声明
void PID_Init(volatile PID_HandleTypeDef *pid, float kp, float ki, float kd, float target);
float PID_Calc(volatile PID_HandleTypeDef *pid, float feed_val);

#endif