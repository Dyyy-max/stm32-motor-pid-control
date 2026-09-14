/**
 * @file main.c
 * @brief STM32F103C8T6 电机PID闭环控制主程序
 * 
 * 功能说明：
 * - 系统时钟配置（72MHz HSE主频）
 * - 串口USART1初始化（9600波特率，用于调试输出）
 * - 定时器TIM2 PWM输出（控制电机转速）
 * - 定时器TIM3 输入捕获（测量外部脉冲频率与个数，PA6）
 * - GPIO初始化（按键输入、指示灯、控制信号）
 * - 增量式PID算法实现电机速度闭环控制
 *
 * 硬件连接：
 * - PA0: 按键输入（KEY1，上拉）
 * - PA1: TIM2_CH2 PWM输出（电机PWM控制）
 * - PA6: TIM3_CH1 输入捕获（脉冲测量；测试时可用跳线与PA1短接做信号回环自测）
 * - PA9/PA10: USART1_TX/RX（串口调试）
 * - PB0: 电机方向控制
 * - PC13: 板载LED指示
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"   /* HAL_TIM_MspPostInit 函数原型 */
#include "pid.h"

/* ==================== 全局变量定义 ==================== */
UART_HandleTypeDef huart1;        // USART1句柄（串口调试）
TIM_HandleTypeDef htim2;          // TIM2句柄（PWM输出）
TIM_HandleTypeDef htim3;          // TIM3句柄（输入捕获测频）
PID_HandleTypeDef motor_pid;      // 电机PID控制结构体
uint8_t motor_run = 0;            // 电机运行标志位（0:停止, 1:运行）
uint16_t pwm_duty = 500;          // PWM占空比初值（0-999）

/* ===== 输入捕获测量结果（中断中更新，主循环读取） =====
 * 原理：TIM3计数时钟1MHz（1个计数=1us），每个上升沿锁存一次计数值，
 *       相邻两次锁存值之差=信号周期(us)，频率=1e6/周期；
 *       同时对脉冲个数累计计数。
 * 该方法与电能表校表脉冲测试原理相同：单位时间内统计标准脉冲个数，
 * 再换算电能误差（电表脉冲常数 imp/kWh）。
 */
volatile uint32_t ic_pulse_count = 0;   // 累计捕获到的脉冲个数
volatile uint32_t ic_last_cnt    = 0;   // 上一次捕获锁存值
volatile uint32_t ic_last_ms     = 0;   // 最近一次捕获的系统时间(ms)，用于掉线判断
volatile float    ic_freq_hz     = 0.0f;// 最近一次测得的频率(Hz)



/**
 * @brief 重定向printf到USART1
 * @param ch: 要输出的字符
 * @param f: 文件指针（未使用）
 * @retval 输出的字符
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

/**
 * @brief 系统时钟配置
 * @details 配置72MHz系统时钟，使用外部8MHz晶振
 * - HSE: 8MHz外部晶振
 * - PLL: 9倍频 = 72MHz
 * - AHB: 1分频 = 72MHz
 * - APB1: 2分频 = 36MHz（最高允许36MHz）
 * - APB2: 1分频 = 72MHz
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) while(1);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) while(1);
}

/**
 * @brief USART1串口初始化
 * @details 配置USART1用于调试信息输出
 * - 波特率: 9600
 * - 数据位: 8位
 * - 停止位: 1位
 * - 无校验位
 * - 收发模式
 */
static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

/**
 * @brief TIM2定时器初始化（PWM输出）
 * @details 配置TIM2用于产生PWM信号控制电机转速
 * - 预分频: 72 (分频后频率 = 72MHz / 72 = 1MHz)
 * - 自动重装载值: 999 (PWM频率 = 1MHz / 1000 = 1kHz)
 * - PWM模式: PWM1 (计数小于比较值时输出有效电平)
 * - 通道2: PA1输出（PA0保留给按键KEY1，避免引脚复用冲突）
 */
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_duty;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

    /* 将PA1切换为TIM2_CH2复用推挽输出（实现于stm32f1xx_hal_msp.c），
       不调用此函数则PWM只在定时器内部有效、引脚上测不到波形 */
    HAL_TIM_MspPostInit(&htim2);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

/**
 * @brief TIM3定时器初始化（输入捕获，PA6 / TIM3_CH1）
 * @details 用于测量外部脉冲信号的频率与个数
 * - 预分频: 71 → 计数时钟 72MHz/(71+1)=1MHz，1个计数 = 1us
 * - 自动重装载: 65535（16位自由计数，差值按 &0xFFFF 处理回绕）
 * - 捕获边沿: 上升沿，直连通道，不分频、无滤波
 * - 每捕获一个上升沿进入中断，在回调中计算周期并累计脉冲数
 */
static void MX_TIM3_Init(void) {
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim3);

    sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;       // 上升沿捕获
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;   // 通道直连
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;             // 每次边沿都捕获
    sConfigIC.ICFilter = 0;                             // 不滤波
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

    /* 启动输入捕获并开启捕获中断（NVIC在HAL_TIM_Base_MspInit中使能） */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief 输入捕获中断回调函数（HAL库弱定义函数的用户实现）
 * @note  PA6每来一个上升沿执行一次：
 *        周期 = 本次锁存值 - 上次锁存值（单位us），频率 = 1e6 / 周期
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        uint32_t now_cnt = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t period_us = (now_cnt - ic_last_cnt) & 0xFFFFu; /* 处理16位回绕 */

        if (period_us != 0u) {
            ic_freq_hz = 1000000.0f / (float)period_us;
        }
        ic_last_cnt = now_cnt;
        ic_pulse_count++;
        ic_last_ms = HAL_GetTick();
    }
}

/**
 * @brief GPIO初始化
 * @details 配置各GPIO引脚功能
 * - PA0:  KEY1按键输入（上拉）
 * - PC13: 板载LED（推挽输出）
 * - PB0:  电机方向控制（推挽输出）
 */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA0: 按键输入（KEY1）
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC13: 板载LED
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PB0: 电机方向控制
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}




/**
 * @brief 电机启动
 * @details 初始化PID参数，设置PWM初始值，置位方向和控制引脚
 * @note PID参数: Kp=2.0, Ki=0.1, Kd=0.05, 目标速度=800
 */
void Motor_Start(void) {
    PID_Init(&motor_pid, 2.0f, 0.1f, 0.05f, 800.0f);
    pwm_duty = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_duty);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // 方向引脚置位
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED亮起
    printf("Motor Started\r\n");
}

/**
 * @brief 电机停止
 * @details 关闭PWM输出，复位方向和控制引脚
 */
void Motor_Stop(void) {
    pwm_duty = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_duty);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // 方向引脚复位
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    // LED熄灭
    printf("Motor Stopped\r\n");
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    // 初始化随机数种子（用于模拟速度波动）
    srand((unsigned int)HAL_GetTick());

    // 初始状态：电机停止
    Motor_Stop();
    motor_run = 0;

    printf("System Initialized!\r\n");

    while (1) {
        uint32_t cap_freq;  // 输入捕获测得的频率（无信号时为0）

        // ===== 按键检测（KEY1 on PA0） =====
        // 按键按下时为低电平，进行消抖处理
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
            HAL_Delay(20);  // 消抖延时20ms
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
                motor_run = !motor_run;  // 切换运行状态

                if (motor_run) {
                    Motor_Start();
                } else {
                    Motor_Stop();
                }

                // 等待按键释放，防止连续触发
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
            }
        }

        // ===== PID闭环控制（100ms周期） =====
        if (motor_run) {
            // 模拟电机速度反馈（基于PWM和随机波动）
            float simulated_speed = pwm_duty * 1.8f + (rand() % 100 - 50);

            // 速度限幅
            if (simulated_speed < 0) simulated_speed = 0;
            if (simulated_speed > 1500) simulated_speed = 1500;

            // PID计算，得到新的PWM占空比
            pwm_duty = (uint16_t)PID_Calc(&motor_pid, simulated_speed);

            // 更新PWM输出
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_duty);

            // 串口打印调试信息
            printf("Set:%d | Measured:%d | PWM:%d\r\n",
                   (int)(motor_pid.target * 10), (int)(simulated_speed * 10), pwm_duty);
        }

        // ===== 输入捕获结果上报（与电机是否运行无关，只要PA6有脉冲即可测量） =====
        // 超过300ms没有捕获到上升沿，认为无输入信号，频率显示0
        cap_freq = ((HAL_GetTick() - ic_last_ms) < 300u) ? (uint32_t)ic_freq_hz : 0u;
        printf("CAP Freq:%lu Hz | Pulses:%lu\r\n",
               (unsigned long)cap_freq, (unsigned long)ic_pulse_count);

        HAL_Delay(100);  // 控制周期100ms
    }
}





