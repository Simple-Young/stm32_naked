        .syntax unified
        .cpu cortex-m3

        .global     main

        .word _sidata
        .word _sdata
        .word _edata
        .word _sbss
        .word _ebss

        .section    .isr_vector,   "a"
        .word       _estack       
        .word       Reset_Handler
        .word       NMI_Handler            /* NMI 中断 */
        .word       HardFault_Handler      /* 硬错误 */
        .word       MemManage_Handler      /* （可选） */
        .word       BusFault_Handler
        .word       UsageFault_Handler
        .word       0                      /* 保留 */
        .word       0
        .word       0
        .word       0
        .word       SVC_Handler
        .word       DebugMon_Handler
        .word       0                      /* 保留 */
        .word       PendSV_Handler
        .word       SysTick_Handler        /* 系统滴答定时器中断 */


        .word       WWDG_IRQHandler
        .word       PVD_IRQHandler
        .word       TAMPER_IRQHandler
        .word       RTC_IRQHandler
        .word       FLASH_IRQHandler
        .word       RCC_IRQHandler
        .word       EXTI0_IRQHandler
        .word       EXTI1_IRQHandler
        .word       EXTI2_IRQHandler
        .word       EXTI3_IRQHandler
        .word       EXTI4_IRQHandler
        .word       DMA1_Channel1_IRQHandler
        .word       DMA1_Channel2_IRQHandler
        .word       DMA1_Channel3_IRQHandler
        .word       DMA1_Channel4_IRQHandler
        .word       DMA1_Channel5_IRQHandler
        .word       DMA1_Channel6_IRQHandler
        .word       DMA1_Channel7_IRQHandler
        .word       ADC1_2_IRQHandler
        .word       USB_HP_CAN1_TX_IRQHandler
        .word       USB_LP_CAN1_RX0_IRQHandler
        .word       CAN1_RX1_IRQHandler
        .word       CAN1_SCE_IRQHandler
        .word       EXTI9_5_IRQHandler
        .word       TIM1_BRK_IRQHandler
        .word       TIM1_UP_IRQHandler
        .word       TIM1_TRG_COM_IRQHandler
        .word       TIM1_CC_IRQHandler
        .word       TIM2_IRQHandler
        .word       TIM3_IRQHandler
        .word       TIM4_IRQHandler
        .word       I2C1_EV_IRQHandler
        .word       I2C1_ER_IRQHandler
        .word       I2C2_EV_IRQHandler
        .word       I2C2_ER_IRQHandler
        .word       SPI1_IRQHandler
        .word       SPI2_IRQHandler
        .word       USART1_IRQHandler
        .word       USART2_IRQHandler
        .word       USART3_IRQHandler
        .word       EXTI15_10_IRQHandler
        .word       RTCAlarm_IRQHandler
        .word       USBWakeUp_IRQHandler
        
        .section    .text
        .global Reset_Handler
        .type Reset_Handler, %function

Reset_Handler:
    cpsie i             // 允许中断

    ldr r0, =_estack
    mov sp, r0

    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata

    movs r3, #0

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit
    
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss
    
    bl main
    bx lr

.weak SysTick_Handler

