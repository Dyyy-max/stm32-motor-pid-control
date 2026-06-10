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
    /* ==================== 变量定义 ==================== */
    volatile PID_HandleTypeDef motor_pid;  // PID控制结构体
    volatile float sim_actual = 0.0f;       // 模拟的实际转速
    volatile uint32_t cycle = 0;            // 控制周期计数
    int i;                                  // 延时循环变量

    /* ==================== PID参数初始化 ==================== */
    // 参数：Kp=2.0, Ki=0.5, Kd=0.1, 目标转速=100.0
    PID_Init(&motor_pid, 2.0f, 0.5f, 0.1f, 100.0f);

    /* ==================== 打印仿真信息 ==================== */
    printf("===== 电机PID转速闭环仿真 =====\r\n");
    printf("目标转速：%.1f | 初始转速：0.0\r\n", (double)motor_pid.target);
    printf("周期\t目标值\t实际转速\tPID输出\r\n");
	
    /* ==================== 主控制循环 ==================== */
    while(1)
    {
        cycle++;

        /* 模拟电机响应特性
         * 假设电机转速与PWM输出成近似线性关系
         * output * 0.01 为每个周期的转速增量
         */
        sim_actual += motor_pid.output * 0.01f;
        
        // 模拟速度上限（防止仿真数据溢出）
        if (sim_actual > 120.0f)
            sim_actual = 0.0f;

        // PID计算，更新输出值
        PID_Calc(&motor_pid, sim_actual);

        // 每10个周期打印一次数据（减少输出频率）
        if (cycle % 10 == 0)
        {
           printf("%u\t%.1f\t%.1f\t%.1f\r\n",
                   cycle,
                   (double)motor_pid.target,
                   (double)sim_actual,
                   (double)motor_pid.output);
        }

        // 简单延时（模拟控制周期）
        for(i = 0; i < 1000000; i++);
    }
}