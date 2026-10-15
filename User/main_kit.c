/**
 * @file main_kit.c
 * @brief STM32F103C8T6 电机PID闭环控制 —— 江协科技 PID 入门套件实物版
 *
 * 与回环自测版 User/main.c 的区别（同一套算法、同一 Keil 工程，编译时二选一）：
 * - PWM 走 TIM2_CH1 / PA0（套件控制板丝印 PWMA，直连 TB6612FNG 的 PWMA）；
 * - 方向控制从单引脚改为 TB6612 双脚真值表：AIN1=PB12 / AIN2=PB13；
 *   10=正转、01=反转、11=刹车、00=滑行；STBY(U2针13) 硬件直连 3.3V 常使能，固件不管；
 * - 按键改到套件 K1=PB10（按下接地，上拉输入，逻辑与原版相同）；
 * - TIM3 从"单通道输入捕获"改为"编码器接口模式"：PA6(E1A)/PA7(E1B)，
 *   硬件对 AB 正交信号自动四倍频，16 位 CNT 直接就是正交计数；
 * - 反馈量是**真实霍尔编码器转速**（M 法测速：固定 100ms 周期读 CNT 差值），
 *   不再使用 main.c 里的软件模拟速度，实现真正的电机速度闭环。
 *
 * 【硬件参数（出自套件用户手册与编码电机手册）】
 * - 电机：25GA370，12V 空载 620RPM，霍尔 YS2402 ×2，AB 正交；
 * - 霍尔 11 脉冲/电机轴每圈 × 减速比 9.2767 ≈ 102 脉冲/输出轴每圈/相，
 *   再 ×4（正交四倍频）= 408 个计数/输出轴一圈；
 * - 测速公式：RPM = CNT100ms 差值 × (60s / 408 / 0.1s) = delta × 1.4706；
 * - 620RPM 时计数率约 4216/s，100ms 增量约 422，远在 int16 余量内。
 *
 * 【安全上电顺序】（编码电机为 PH2.0-6P 一体插头：M+/GND/EB/EA/3V3/M-，
 *  电机绕组与编码器同插头，"只插编码器不插电机"物理上做不到）
 * 1. 编码器验证阶段：USB 线接最小系统板 USB 口供 5V（板载 LDO 产出 3.3V 逻辑电），
 *    底板 VIN（DC 座/接线端子）悬空不接 → TB6612 的 VM 无动力电；插好 6P 插头，
 *    此时编码器得电而电机绝不可能转动；
 * 2. SWD 烧录后手转电机输出轴：每 100ms 帧 CNT 增量之和每圈应≈408，正转 CNT 为正；
 *    若正转为负，对调插头上 EA/EB 两相，或将 ENC_CH1_POLARITY 改为 FALLING（一处即可）；
 * 3. 极性验证通过后再接底板动力电（手册允许 5~12V，首次建议先接 5V），
 *    初始目标转速保守设 150RPM（全速 620 的 1/4），只有按 K1 后电机才会转；
 * 4. 停止时 TB6612 给 11=刹车（短接制动），比 00=滑行更安全、定位更快。
 */

#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"   /* Error_Handler / 公共声明 */
#include "pid.h"

/* ==================== 套件硬件引脚定义（原理图固定，不可改线） ==================== */
#define TB6612_AIN1_PORT   GPIOB
#define TB6612_AIN1_PIN    GPIO_PIN_12   /* 套件网络 AIN1 */
#define TB6612_AIN2_PORT   GPIOB
#define TB6612_AIN2_PIN    GPIO_PIN_13   /* 套件网络 AIN2 */
#define KEY_K1_PORT        GPIOB
#define KEY_K1_PIN         GPIO_PIN_10   /* 套件 K1，按下接 GND */
#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13   /* 最小系统板 LED，低电平点亮 */

/* ==================== 控制与测速参数 ==================== */
#define PWM_PERIOD          999u          /* ARR+1=1000，PWM 1kHz@PSC71，分辨率 0.1% */
#define ENCODER_CPR         408.0f        /* 输出轴每圈正交计数（已含四倍频与减速比） */
#define SPEED_SAMPLE_S      0.1f          /* M 法测速闸门时间 = 100ms */
#define RPM_PER_COUNT       (60.0f / (ENCODER_CPR * SPEED_SAMPLE_S))  /* ≈1.4706 RPM/计数 */
#define TARGET_RPM_DEFAULT  150.0f        /* 初始目标转速（保守，全速 620RPM 的约 1/4） */

/* PID 参数：RPM 单位下的保守初值，**上板后必须按实际响应重新整定**。
 * 选值依据：150RPM 稳态约需 PWM 240/999；首次计算 Kp*150+Ki*150=225，不顶限幅。 */
#define PID_KP              1.2f
#define PID_KI              0.3f
#define PID_KD              0.0f

/* 编码器方向：若正转时 CNT 为负，对调 EA/EB 或把 RISING 改为 FALLING */
#define ENC_CH1_POLARITY    TIM_ICPOLARITY_RISING
#define ENC_CH2_POLARITY    TIM_ICPOLARITY_RISING

/* ==================== 全局变量定义（it.c 通过 extern 引用） ==================== */
UART_HandleTypeDef huart1;        /* USART1句柄（串口调试） */
TIM_HandleTypeDef htim2;          /* TIM2句柄（PWM输出，CH1/PA0） */
TIM_HandleTypeDef htim3;          /* TIM3句柄（编码器接口，PA6/PA7） */
PID_HandleTypeDef motor_pid;      /* 电机PID控制结构体 */
uint8_t motor_run = 0;            /* 电机运行标志（0:刹车停止, 1:运行） */
uint16_t pwm_duty = 0;            /* PWM比较值（0-999） */

/* ===== 编码器测速结果（主循环内 100ms 刷新一次） ===== */
volatile int16_t enc_delta = 0;   /* 本闸门内正交计数差（带符号，反转为负） */
float motor_speed_rpm = 0.0f;     /* 实测转速 RPM（M 法换算） */


/**
 * @brief 重定向printf到USART1
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

/**
 * @brief 系统时钟配置：HSE 8MHz × PLL9 = 72MHz，APB1=36MHz（TIM 时钟 72MHz）
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
 * @brief USART1 初始化（9600 8N1）。
 * @note 原理图实锤（J3 UART接口）：针3=TX 网络直连 PA9(USART1_TX)，
 *       针2=RX 网络直连 PA10(USART1_RX)，针1=VCC/针4=GND；PA8 在板上悬空。
 *       即排针收发都可用：USB转TTL 的 RXD 接排针 TX 针(针3)接收 printf，
 *       需要下行指令时模块 TXD 接排针 RX 针(针2)。
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
 * @brief TIM2 PWM 初始化（CH1 / PA0 = 套件 PWMA）
 * @details PSC=71 → 计数时钟 1MHz；ARR=999 → PWM 1kHz；占空比 0~999/1000。
 *          PA0 在本文件 GPIO 初始化中直接配成复用推挽（不走共享 MSP 的
 *          HAL_TIM_MspPostInit——那个函数配置的是回环版的 PA1/CH2）。
 */
static void MX_TIM2_PWM_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                 /* 上电占空比 0，杜绝上电冲转 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
 * @brief TIM3 编码器接口初始化（PA6=E1A/TI1，PA7=E1B/TI2）
 * @details ENCODERMODE_TI12：AB 两相的所有边沿都计数 → 硬件四倍频；
 *          CNT 在 0~65535 间自由计数，反转时递减（回绕按 int16 解读）。
 *          输入加数字滤波（N=8，采样 fDTS/32≈2.25MHz），滤除霍尔边沿抖动；
 *          电机最高计数率仅约 4.2kHz，远低于滤波时延上限，不丢脉冲。
 *          纯轮询读 CNT，不需要中断/NVIC。
 * @note    STM32F1 HAL 中 HAL_TIM_Encoder_Init() 内部只调用
 *          HAL_TIM_Base_Init() → HAL_TIM_Base_MspInit()（Core/Src 下的共享 MSP），
 *          **不存在 HAL_TIM_Encoder_MspInit 钩子**；共享 MSP 的 TIM3 分支只配了
 *          PA6（PA7 漏配，靠复位默认浮空输入侥幸可用）并使能了 TIM3 NVIC。
 *          因此这里在 Encoder_Init 之前显式初始化时钟与 PA6/PA7（D-003 修复），
 *          不依赖 MSP 分支行为。
 * @warning 关闭 TIM3 NVIC 必须放在 HAL_TIM_Encoder_Init() **之后**：该函数内部经
 *          HAL_TIM_Base_Init() 回调共享 MSP，而 MSP 的 TIM3 分支会 EnableIRQ(TIM3_IRQn)，
 *          若在 Init 之前关闭会被立刻重新使能（编码器模式不开 DIER 中断位，实际不进
 *          ISR，但 NVIC 必须显式关死，与 it.c 中仍存在的 TIM3_IRQHandler 彻底解耦）。
 */
static void MX_TIM3_Encoder_Init(void) {
    TIM_Encoder_InitTypeDef sEnc = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 显式底层初始化：TIM3 时钟 + PA6(E1A)/PA7(E1B) 复用输入 */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;   /* 霍尔上拉在控制板硬件侧 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sEnc.EncoderMode = TIM_ENCODERMODE_TI12;       /* 双通道边沿计数 = 四倍频 */
    sEnc.IC1Polarity = ENC_CH1_POLARITY;
    sEnc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEnc.IC1Prescaler = TIM_ICPSC_DIV1;
    sEnc.IC1Filter = 0x0F;
    sEnc.IC2Polarity = ENC_CH2_POLARITY;
    sEnc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEnc.IC2Prescaler = TIM_ICPSC_DIV1;
    sEnc.IC2Filter = 0x0F;

    HAL_TIM_Encoder_Init(&htim3, &sEnc);
    /* MSP 在上面 Init 内部刚使能了 TIM3 NVIC；编码器纯轮询不用中断，此处关死 */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/**
 * @brief GPIO 初始化（套件引脚）
 * @details PA0=PWMA 复用推挽；PB12/PB13=TB6612 方向，上电先刹车（11）；
 *          PB10=K1 上拉输入；PC13=LED。
 *          安全要点：方向脚在切成输出前先写入安全电平，避免引脚切换瞬间的毛刺。
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PB12/PB13 先写安全电平（11=刹车），再切换为推挽输出，杜绝上电毛刺 */
    HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = TB6612_AIN1_PIN | TB6612_AIN2_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PC13 板载 LED，初始熄灭（高电平） */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* PB10 = K1 按键，上拉输入（按下为低） */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = KEY_K1_PIN;
    HAL_GPIO_Init(KEY_K1_PORT, &GPIO_InitStruct);

    /* PA0 = TIM2_CH1 PWMA，复用推挽（与 HAL_TIM_MspPostInit 对 PA1 的做法等价） */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ==================== 业务逻辑 ==================== */

/**
 * @brief 电机启动：AIN1/AIN2=10 正转，PID 以 RPM 为工程单位重新初始化
 */
static void Motor_Start(void) {
    PID_Init(&motor_pid, PID_KP, PID_KI, PID_KD, TARGET_RPM_DEFAULT);
    pwm_duty = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_SET);    /* 10 = 正转 */
    HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);                  /* LED 亮 */
    printf("Motor Started (TB6612 FWD, target %d RPM)\r\n", (int)TARGET_RPM_DEFAULT);
}

/**
 * @brief 电机停止：PWM=0 且 AIN1/AIN2=11，TB6612 短接刹车
 */
static void Motor_Stop(void) {
    pwm_duty = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty);
    HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_SET);    /* 11 = 刹车 */
    HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);                    /* LED 灭 */
    printf("Motor Stopped (TB6612 brake)\r\n");
}

/**
 * @brief M 法测速：读取并清零 TIM3->CNT，把 100ms 计数差换算成 RPM
 * @retval 带符号转速（RPM），反转时为负
 */
static float Encoder_ReadRPM(void)
{
    /* uint16→int16 的重解释同时处理了 16 位回绕（如 65535 即 -1） */
    int16_t cnt16 = (int16_t)(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    enc_delta = cnt16;
    return (float)cnt16 * RPM_PER_COUNT;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Encoder_Init();   /* 先起编码器：即使电机没使能也能手转验证 */
    MX_TIM2_PWM_Init();

    /* 初始安全状态：PWM=0、TB6612 刹车 */
    motor_run = 0;
    Motor_Stop();

    printf("\r\n=== STM32 PID Kit Firmware (TB6612 + 408CPR Encoder) ===\r\n");
    printf("System Initialized! K1(PB10)=start/stop, sample=100ms\r\n");

    while (1) {
        /* ===== K1 按键检测（PB10，20ms 消抖，等待释放防连发） ===== */
        if (HAL_GPIO_ReadPin(KEY_K1_PORT, KEY_K1_PIN) == GPIO_PIN_RESET) {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(KEY_K1_PORT, KEY_K1_PIN) == GPIO_PIN_RESET) {
                motor_run = !motor_run;
                if (motor_run) {
                    Motor_Start();
                } else {
                    Motor_Stop();
                }
                while (HAL_GPIO_ReadPin(KEY_K1_PORT, KEY_K1_PIN) == GPIO_PIN_RESET);
            }
        }

        /* ===== 100ms 测速闸门：无论运行与否都读数，停止时可手转验证编码器 ===== */
        motor_speed_rpm = Encoder_ReadRPM();

        /* ===== PID 真实转速闭环（仅运行时） ===== */
        if (motor_run) {
            pwm_duty = (uint16_t)PID_Calc(&motor_pid, motor_speed_rpm);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty);

            printf("[RUN ] Tgt:%4d RPM | Spd:%6d RPM | CNT:%5d | PWM:%3d/999\r\n",
                   (int)motor_pid.target, (int)motor_speed_rpm,
                   (int)enc_delta, pwm_duty);
        } else {
            /* 停止态：手转电机轴，CNT 应随手势增减，整圈累计约 ±408（TC-15） */
            printf("[STOP] Spd:%6d RPM | CNT:%5d | brake\r\n",
                   (int)motor_speed_rpm, (int)enc_delta);
        }

        HAL_Delay(100);  /* 控制周期 = 测速闸门 = 100ms */
    }
}
