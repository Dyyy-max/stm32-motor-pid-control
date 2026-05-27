# STM32F103 直流电机PWM调速 + PID闭环稳速
## 项目简介
基于STM32F103C8T6(HAL库)开发的嵌入式车载电机控制Demo，实现按键启停、LED状态指示、PWM调速、增量式PID转速闭环控制，通过串口实时上报运行参数。
适用于汽车电子、嵌入式控制学习与工程实践。

## 技术栈
- 主控芯片：STM32F103C8T6
- 开发工具：STM32CubeMX、Keil MDK5
- 外设：GPIO、TIM(PWM)、USART
- 算法：增量式PID控制算法
- 版本管理：Git / GitHub

## 硬件说明
本项目**无需实体硬件**，全程基于 Keil MDK 软件仿真运行。
仿真目标芯片：STM32F103C8T6
可仿真外设：GPIO、TIM(PWM)、USART 串口，完整验证代码逻辑与算法效果。

## 功能列表
1. 按键控制电机启停，LED同步状态指示
2. TIM2 输出1kHz PWM波形调节电机转速
3. 增量式PID算法实现转速闭环稳速
4. USART1(9600波特率)串口打印转速、PWM、PID参数

## 编译&运行步骤
1. 使用 STM32CubeMX 重新生成工程（可选，直接用现有工程也可）
2. Keil MDK5 打开 `motor-pid-control/Motor_PID_Control.uvprojx` 编译
3. 开启软件仿真模式运行程序
4. 串口助手/Keil内置串口窗口设置 9600 波特率，查看运行日志
5. 可通过Keil寄存器、逻辑分析仪观察引脚电平与PWM波形

## 目录结构
arm-bsp-learing/
├── motor-pid-control/         # Keil工程与核心代码
│   ├── Core/                  # 业务代码（main.c、pid.c等）
│   ├── Drivers/               # STM32 HAL库驱动文件
│   └── Motor_PID_Control/     # Keil工程配置文件
├── .gitignore                 # Git忽略文件配置（过滤编译临时文件）
└── README.md                  # 项目说明文档（本文件）
本项目旨在展示嵌入式C语言开发、外设驱动与控制算法的工程实践能力，
适配汽车电子、嵌入式控制岗位的技能要求。