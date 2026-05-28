#include "stm32f10x.h"

// PID结构体定义
typedef struct
{
    float Kp;    // 比例系数
    float Ki;    // 积分系数
    float Kd;    // 微分系数
    
    float SetPoint;  // 设定目标值
    float ActualValue; // 实际值
    float Err;    // 当前误差
    float LastErr; // 上一次误差
    float PrevErr; // 上上次误差
    float Output; // 输出值
} PID_Typedef;

// 全局PID变量
PID_Typedef pid;

// PID初始化函数
void PID_Init(void)
{
    pid.Kp = 2.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.1f;
    pid.SetPoint = 100.0f; // 目标值设为100
    pid.ActualValue = 0.0f;
    pid.Err = 0.0f;
    pid.LastErr = 0.0f;
    pid.PrevErr = 0.0f;
    pid.Output = 0.0f;
}

// 增量式PID计算函数
float PID_Incremental_Calc(PID_Typedef *pid, float actual_val)
{
    pid->ActualValue = actual_val;
    pid->Err = pid->SetPoint - pid->ActualValue;
    
    // 增量式PID公式
    pid->Output += pid->Kp * (pid->Err - pid->LastErr) +
                   pid->Ki * pid->Err +
                   pid->Kd * (pid->Err - 2 * pid->LastErr + pid->PrevErr);
    
    // 更新误差历史值
    pid->PrevErr = pid->LastErr;
    pid->LastErr = pid->Err;
    
    // 输出限幅（防止溢出）
    if (pid->Output > 1000) pid->Output = 1000;
    if (pid->Output < 0) pid->Output = 0;
    
    return pid->Output;
}

int main(void)
{
    float sim_actual = 0.0f; // 模拟实际值，用来测试PID
    
    PID_Init(); // 初始化PID参数
    
    while(1)
    {
        // 模拟一个缓慢变化的实际值（用来测试PID）
        sim_actual += pid.Output * 0.01f;
        if (sim_actual > 120) sim_actual = 0;
        
        // 计算PID输出
        PID_Incremental_Calc(&pid, sim_actual);
        
        // 可以打个断点，看pid.Err、pid.Output这些变量的值
    }
}