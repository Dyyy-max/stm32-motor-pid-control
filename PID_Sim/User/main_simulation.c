/**
 * @file main_simulation.c
 * @brief PID控制算法仿真测试程序
 * 
 * 功能说明：
 * - 在无硬件环境下测试PID算法效果
 * - 模拟电机转速闭环控制过程
 * - 通过串口输出仿真数据，观察PID响应曲线
 * 
 * 仿真模型：
 * - 模拟电机响应：sim_actual += output * 0.01
 * - 模拟速度上限：120.0
 * - 控制周期：约1秒（由延时循环决定）
 */

#include <stdio.h>
#include <stdint.h>
#include "pid.h"

int main(void)
{
    volatile PID_HandleTypeDef motor_pid;
    volatile float sim_actual = 0.0f;
    volatile uint32_t cycle = 0;
    int i;

    PID_Init(&motor_pid, 2.0f, 0.5f, 0.1f, 100.0f);

    printf("===== 电机PID转速闭环仿真 =====\r\n");
    printf("目标转速：%.1f | 初始转速：0.0\r\n", (double)motor_pid.target);
    printf("周期\t目标值\t实际转速\tPID输出\r\n");
	
    while(1)
    {
        cycle++;

        sim_actual += motor_pid.output * 0.01f;
        
        if (sim_actual > 120.0f)
            sim_actual = 0.0f;

        PID_Calc(&motor_pid, sim_actual);

        if (cycle % 10 == 0)
        {
           printf("%u\t%.1f\t%.1f\t%.1f\r\n",
                   cycle,
                   (double)motor_pid.target,
                   (double)sim_actual,
                   (double)motor_pid.output);
        }

        for(i = 0; i < 1000000; i++);
    }
}