;=============================================================================
; 文件名: startup_sim.s
; 功能:   纯软件仿真启动文件（用于Keil仿真环境测试PID算法）
; 说明:   此文件为简化版启动代码，仅包含仿真所需的最基本初始化
;         不依赖真实STM32硬件，适用于算法验证和调试
;=============================================================================

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     0x20000400
                DCD     Reset_Handler

                AREA    |.text|, CODE, READONLY

Reset_Handler   PROC
                EXPORT  Reset_Handler   [WEAK]
                IMPORT  main
                LDR     R0, =main
                BX      R0
                ENDP

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Size      EQU     0x00000400
Stack_Mem       SPACE   Stack_Size

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
Heap_Size       EQU     0x00000400
Heap_Mem        SPACE   Heap_Size

                END