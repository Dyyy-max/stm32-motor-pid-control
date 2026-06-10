/**
 * @file pid.c
 * @brief 增量式PID控制算法实现
 * 
 * 算法说明：
 * 增量式PID相较于位置式PID的优势：
 * 1. 计算量小，无需累积误差
 * 2. 输出为增量，误动作时影响小
 * 3. 手动/自动切换冲击小
 * 
 * 增量式PID公式：
 * Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
 * u(k) = u(k-1) + Δu(k)
 * 
 * 参数调节建议：
 * - Kp（比例系数）：响应速度，过大会导致振荡
 * - Ki（积分系数）：消除稳态误差，过大会导致超调
 * - Kd（微分系数）：抑制振荡，提高稳定性
 */

#include "pid.h"

/**
 * @brief  PID 参数初始化
 * @param  pid: PID 句柄指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @param  target: 控制目标值（设定速度）
 */
void PID_Init(volatile PID_HandleTypeDef *pid, float kp, float ki, float kd, float target)
{
    pid->Kp        = kp;
    pid->Ki        = ki;
    pid->Kd        = kd;
    pid->target    = target;
    pid->feedback  = 0.0f;
    pid->err       = 0.0f;
    pid->err_last  = 0.0f;
    pid->err_prev  = 0.0f;
    pid->output    = 0.0f;
}

/**
 * @brief  增量式 PID 计算函数
 * @param  pid: PID 句柄指针
 * @param  feed_val: 外部反馈值（实测速度）
 * @retval PID 输出值（PWM占空比）
 * 
 * 计算流程：
 * 1. 更新反馈值
 * 2. 计算当前误差
 * 3. 应用增量式PID公式计算输出增量
 * 4. 限幅处理（0~999）
 * 5. 更新误差历史记录
 */
float PID_Calc(volatile PID_HandleTypeDef *pid, float feed_val)
{
    pid->feedback = feed_val;
    pid->err = pid->target - pid->feedback;

    // 增量式 PID 公式
    pid->output += pid->Kp * (pid->err - pid->err_last)
               + pid->Ki * pid->err
               + pid->Kd * (pid->err - 2 * pid->err_last + pid->err_prev);

    // 输出限幅 0~999
    if(pid->output > 999)
        pid->output = 999;
    if(pid->output < 0)
        pid->output = 0;

    // 更新历史误差
    pid->err_prev  = pid->err_last;
    pid->err_last  = pid->err;

    return pid->output;
}
