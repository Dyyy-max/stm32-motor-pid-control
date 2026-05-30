#include "pid.h"

/**
 * @brief  PID 参数初始化
 * @param  pid: PID 句柄指针
 * @param  kp/ki/kd: PID 三参数
 * @param  target: 控制目标值
 */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float target)
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
 * @param  feed_val: 外部反馈值
 * @retval PID 输出值
 */
float PID_Calc(PID_HandleTypeDef *pid, float feed_val)
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








