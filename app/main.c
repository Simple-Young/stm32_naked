#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define PERIPH_BASE             0x40000000U /*!< Peripheral base address in the alias region */
#define AHBPERIPH_BASE          (PERIPH_BASE + 0x00020000U)
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         (PERIPH_BASE + 0x00010000U)

#define GPIOA_BASE              (APB2PERIPH_BASE + 0x00000800U)
#define GPIOB_BASE              (APB2PERIPH_BASE + 0x00000C00U)
#define GPIOC_BASE              (APB2PERIPH_BASE + 0x00001000U)
#define GPIOD_BASE              (APB2PERIPH_BASE + 0x00001400U)
#define GPIOE_BASE              (APB2PERIPH_BASE + 0x00001800U)
#define GPIOF_BASE              (APB2PERIPH_BASE + 0x00001C00U)
#define GPIOG_BASE              (APB2PERIPH_BASE + 0x00002000U)


#define GPIOA_CRH               (*(volatile unsigned int*)(GPIOA_BASE + 0x04))
#define GPIOA_ODR               (*(volatile unsigned int*)(GPIOA_BASE + 0x0C))
#define GPIOB_CRH               (*(volatile unsigned int*)(GPIOB_BASE + 0x04))
#define GPIOB_ODR               (*(volatile unsigned int*)(GPIOB_BASE + 0x0C))


#define USART1_BASE             (APB2PERIPH_BASE + 0x3800)
#define USART1_SR               (*(volatile uint32_t*)(USART1_BASE + 0x00))
#define USART1_DR               (*(volatile uint32_t*)(USART1_BASE + 0x04))
#define USART1_BRR              (*(volatile uint32_t*)(USART1_BASE + 0x08))
#define USART1_CR1              (*(volatile uint32_t*)(USART1_BASE + 0x0C))
#define USART1_CR2              (*(volatile uint32_t*)(USART1_BASE + 0x10))
#define USART1_CR3              (*(volatile uint32_t*)(USART1_BASE + 0x14))

#define USART1_SR_TXE           (1 << 7)   // 发送缓冲区空标志位
#define USART1_SR_TC            (1 << 6)   // 发送完成标志位
#define USART1_SR_RXNE          (1 << 5)   // 接收缓冲区非空标志位
#define USART1_CR1_UE           (1 << 13)  // USART使能位
#define USART1_CR1_TE           (1 << 3)   // 发送使能位
#define USART1_CR1_RE           (1 << 2)   // 接收使能位
#define USART1_CR2_STOP_BITS_1  (0 << 12)  // 1个停止位


#define RCC_BASE                (AHBPERIPH_BASE + 0x00001000U)
#define RCC_CR                  (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR                (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_APB2ENR             (*(volatile unsigned int*)(RCC_BASE + 0x18))

#define RCC_IOPAEN              (1 << 2)   // RCC_APB2ENR 中 GPIOA 使能位
#define RCC_IOPBEN              (1 << 3)   // RCC_APB2ENR 中 GPIOB 使能位
#define RCC_CR_HSEON            (1 << 16)
#define RCC_CR_HSERDY           (1 << 17)
#define RCC_CR_HSION            (1 << 0)
#define RCC_CR_HSIRDY           (1 << 1)
#define RCC_CR_PLLON            (1 << 24)
#define RCC_CR_PLLRDY           (1 << 25)
#define RCC_CFGR_PLLMULL9       (0x07 << 18)
#define RCC_CFGR_PLLSRC_HSE     (1 << 16)
#define RCC_CFGR_SW_HSI         (0 << 0)
#define RCC_CFGR_SW_PLL         (2 << 0)
#define RCC_CFGR_SWS_PLL        (2 << 2)
#define RCC_CFGR_PLLMUL6        (0x0A << 18)  // PLL输入时钟 x 6
#define RCC_CFGR_PLLSRC         (1 << 16)     // PLL输入源选择HSI/2
#define RCC_CFGR_HPRE_DIV1      (0 << 4)
#define RCC_CFGR_PPRE1_DIV2     (4 << 8)
#define RCC_CFGR_PPRE2_DIV1     (0 << 11)


#define FLASH_BASE              (AHBPERIPH_BASE + 0x00002000U)  
#define FLASH_ACR               (*(volatile uint32_t *)(FLASH_BASE + 0x00))

#define FLASH_ACR_PRFTBE        (1 << 4)
#define FLASH_ACR_LATENCY       (0x07 << 0)
#define FLASH_ACR_LATENCY_2     (0x02 << 0)
#define FLASH_ACR_LATENCY_1     (1 << 0)

#define NVIC_ISER1              (*(volatile uint32_t *)0xE000E104)
#define USART1_IRQn             37

#define SYSTICK_BASE            0xE000E010
#define STK_CTRL                (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define STK_LOAD                (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define STK_VAL                 (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

#define SystemCoreClock         72UL*1000UL*1000UL // 系统时钟频率


#define BUAUD_RATE_9600         9600 
#define BUAUD_RATE_115200       115200
#define BUAUD_RATE              BUAUD_RATE_115200

// 提供默认的中断处理函数
void Default_Handler(void)
{
    while (1);  // 卡死
}

void NMI_Handler(void){
    while (1);  // 卡死
}
void HardFault_Handler(void) {
    GPIOB_ODR &= ~(1 << 12);
    while (1);  // 卡死
}
void MemManage_Handler(void) {
    while (1);  // 卡死
}
void BusFault_Handler(void) {
    while (1);  // 卡死
}
void UsageFault_Handler(void) {
    while (1);  // 卡死
}
void SVC_Handler(void) {
    while (1);  // 卡死
}
void DebugMon_Handler(void) {
    while (1);  // 卡死
}
void PendSV_Handler(void) {
    while (1);  // 卡死
}

// External Interrupts
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TAMPER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
// void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTCAlarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

volatile uint32_t systick_counter = 0;

void SysTick_Handler(void)
{
    // GPIOB_ODR &= ~(1 << 12);
    systick_counter++;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = systick_counter;
    while ((systick_counter - start) < ms);
}

void clock_init(void){
    // 1. 设置 Flash 等待周期（2 个 wait state）
    FLASH_ACR |= (2 << 0); // LATENCY = 2

    // 2. 启动 HSE（外部晶振）
    RCC_CR |= (1 << 16);  // HSEON = 1

    // 3. 等待 HSE 稳定
    while (!(RCC_CR & (1 << 17)));  // HSERDY

    // 4. 设置 PLL：HSE 作为输入 ×9
    RCC_CFGR |= (1 << 16);   // PLLSRC = HSE
    RCC_CFGR &= ~(0xF << 18); // 清空 PLLMUL
    RCC_CFGR |= (7 << 18);   // PLLMUL = 7 + 2 = 9

    // 5. 启动 PLL
    RCC_CR |= (1 << 24);     // PLLON

    // 6. 等待 PLL 就绪
    while (!(RCC_CR & (1 << 25)));  // PLLRDY

    // 7. 切换系统时钟源为 PLL
    RCC_CFGR &= ~(0x3 << 0);  // 清空 SW
    RCC_CFGR |= (0x2 << 0);   // SW = 10 => PLL selected

    // 8. 等待切换完成
    while ((RCC_CFGR & (0x3 << 2)) != (0x2 << 2));  // SWS = 10
}

void SystemClock_Config(void) {
    // 1. 启用HSE（外部高速时钟，通常8MHz晶振）
    RCC_CR |= RCC_CR_HSEON;
    while(!(RCC_CR & RCC_CR_HSERDY));  // 等待HSE就绪
    
    // 2. 配置FLASH等待状态（重要！否则在高频率下会失败）
    FLASH_ACR |= FLASH_ACR_PRFTBE;     // 启用预取缓冲区
    FLASH_ACR &= ~FLASH_ACR_LATENCY;   // 清除等待状态位
    FLASH_ACR |= FLASH_ACR_LATENCY_2;  // 等待状态（48MHz < SYSCLK ≤ 72MHz）
    
    // 3. 配置AHB/APB分频器
    RCC_CFGR &= ~(0xF << 4);   // AHB不分频（清除HPRE[3:0]）
    RCC_CFGR &= ~(0x7 << 8);   // APB1分频/2（清除PPRE1[2:0]）
    RCC_CFGR |= (0x04 << 8);   // APB1 = HCLK/2
    RCC_CFGR &= ~(0x7 << 11);  // APB2不分频（清除PPRE2[2:0]）
    
    // 4. 配置PLL（假设HSE=8MHz，目标72MHz系统时钟）
    RCC_CFGR &= ~(0x3 << 16);  // 清除PLLSRC[1:0]
    RCC_CFGR |= RCC_CFGR_PLLSRC_HSE;  // HSE作为PLL输入
    RCC_CFGR &= ~(0xF << 18);  // 清除PLLMUL[3:0]
    RCC_CFGR |= RCC_CFGR_PLLMULL9;     // PLL倍频x9（8MHz * 9 = 72MHz）
    
    // 5. 启用PLL
    RCC_CR |= RCC_CR_PLLON;
    while(!(RCC_CR & RCC_CR_PLLRDY));  // 等待PLL锁定
    
    // 6. 切换到PLL作为系统时钟源
    RCC_CFGR &= ~0x03;         // 清除SW[1:0]
    RCC_CFGR |= RCC_CFGR_SW_PLL;  // 选择PLL作为系统时钟
    while((RCC_CFGR & 0x0C) != RCC_CFGR_SWS_PLL);  // 等待切换完成
}

void SystemClock_HSI_Config(void) {
    // 1. 确保HSI已启用（默认已启用）
    RCC_CR |= RCC_CR_HSION;
    while(!(RCC_CR & RCC_CR_HSIRDY));  // 等待HSI就绪
    
    // 2. 配置FLASH（HSI频率低，通常0等待状态）
    FLASH_ACR |= FLASH_ACR_PRFTBE;    // 启用预取缓冲区
    FLASH_ACR &= ~FLASH_ACR_LATENCY;  // 0等待状态
    
    // 3. 配置总线分频
    RCC_CFGR &= ~(0xF << 4);   // AHB不分频
    RCC_CFGR |= (0x04 << 8);   // APB1 = HCLK/2
    RCC_CFGR &= ~(0x7 << 11);  // APB2不分频
    
    // 4. 确保选择HSI作为系统时钟（默认就是HSI）
    RCC_CFGR &= ~0x03;
    
    // 系统时钟现在为HSI的8MHz
}

void SystemClock_HSI_PLL_Config(void) {
    // 1. 启用HSI（如果尚未启用）
    RCC_CR |= RCC_CR_HSION;
    while(!(RCC_CR & RCC_CR_HSIRDY));
    
    // 2. 配置FLASH（48MHz需要1等待状态）
    FLASH_ACR |= FLASH_ACR_PRFTBE;
    FLASH_ACR &= ~FLASH_ACR_LATENCY;
    FLASH_ACR |= FLASH_ACR_LATENCY_1;
    
    // 3. 配置PLL（HSI=8MHz，/2=4MHz输入，x12=48MHz输出）
    RCC_CFGR &= ~(0xF << 18);  // 清除PLL倍频设置
    RCC_CFGR |= RCC_CFGR_PLLMUL6;  // 实际是x12（见参考手册）
    RCC_CFGR |= RCC_CFGR_PLLSRC;   // HSI/2作为PLL输入
    
    // 4. 启用PLL
    RCC_CR |= RCC_CR_PLLON;
    while(!(RCC_CR & RCC_CR_PLLRDY));
    
    // 5. 配置总线分频
    RCC_CFGR &= ~(0xF << 4);   // AHB不分频
    RCC_CFGR |= (0x04 << 8);   // APB1 = HCLK/2 (24MHz)
    RCC_CFGR &= ~(0x7 << 11);  // APB2不分频 (48MHz)
    
    // 6. 切换到PLL作为系统时钟源
    RCC_CFGR &= ~0x03;
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while((RCC_CFGR & 0x0C) != RCC_CFGR_SWS_PLL);
}

void systick_init(void)
{   
    STK_CTRL = 0; 
    STK_LOAD = (SystemCoreClock / 1000) - 1;  // 1ms 中断
    STK_VAL = 0;  // 清零当前值
    // STK_CTRL = (1 << 2) |  // CLKSOURCE = 1, 使用 HCLK
    //            (1 << 1) |  // TICKINT = 1, 使能中断
    //            (1 << 0);   // ENABLE = 1, 启动
    STK_CTRL = 0x07;
}

void uart1_init(void) {
    // 1. 打开 GPIOA 和 USART1 时钟
    RCC_APB2ENR |= RCC_IOPAEN;   // IOPAEN
    RCC_APB2ENR |= (1 << 14);    // USART1EN

    // 2. 设置 PA9 为 复用推挽输出（TX）
    GPIOA_CRH &= ~(0xF << 4);  // 清 PA9 配置
    GPIOA_CRH |=  (0xB << 4);  // PA9: 1011 => AFIO 推挽输出, 50MHz

    GPIOA_CRH &= ~(0xF << 8);  // 清除 PA10 配置
    GPIOA_CRH |=  (0x4 << 8);  // 输入浮空 (0100)，RX 默认就是输入

    // 3. 设置波特率 9600（72MHz / 9600 = 7500）
    // USARTDIV = 72MHz / (16 * 9600) = 468.75
    // 468 = 0x1D4 => Mantissa = 468, Fraction = 0.75 * 16 ≈ 12
    USART1_BRR = SystemCoreClock / BUAUD_RATE;

    // 4. 使能 USART1，发送功能
    USART1_CR1 &= ~(1 << 10); // UE: 0 => 关闭 USART
    USART1_CR1 &= ~(1 << 12); // M: 0 => 8-bit data
    USART1_CR2 &= ~(0b11 << 12); // STOP: 00 => 1 stop bit
    USART1_CR3 &= ~((1 << 8) | (1 << 9)); // 无流控

    USART1_CR1 |= (1 << 13); // UE: USART enable
    USART1_CR1 |= (1 << 3);  // TE: Transmitter enable
    USART1_CR1 |= (1 << 2);  // RE: Receiver enable
    USART1_CR1 |= (1 << 5);    // RXNEIE: 接收非空中断

    // NVIC_ISER1 |= (1 << (USART1_IRQn - 32));  // USART1_IRQn = 37
}

void uart1_send_char(char c) {
    while (!(USART1_SR & (1 << 7))); // 等待 TXE 发送缓冲区空
    USART1_DR = c;
}

void uart1_send_string(const char* str) {
    while (*str) {
        uart1_send_char(*str++);
    }
}

char usart1_recv_char(void)
{
    while (!(USART1_SR & (1 << 5)));  // 等待 RXNE=1（接收到数据）
    return (char)(USART1_DR & 0xFF);  // 读取数据
}

volatile char rx_data = 0;

void USART1_IRQHandler(void)
{
    if (USART1_SR & (1 << 5))  // 检查 RXNE 是否置位
    {
        rx_data = USART1_DR & 0xFF;  // 读取数据（自动清 RXNE）
    }
}

extern const uint8_t _sheap;
extern const uint8_t _eheap;
extern const uint8_t _estack;
extern const uint8_t _sdata;
extern const uint8_t _edata;
extern const uint8_t _sbss;
extern const uint8_t _ebss;


// #define HEAP_TOTAL_SIZE         (5 * (1<<13))   // 40KB 堆大小
// #define HEAP_START_ADDR         (&_sheap)       // 堆起始地址
// #define HEAP_END_ADDR           (&_eheap)       // 堆结束地址
// #define HEAP_GUARD_ADDR         (&_eheap)       // 堆保护地址
// #define HEAP_GUARD_SIZE         (1<<12)         // 堆保护区大小
// #define HEAP_BLOCK_SIZE         (1<<4)          // 最小堆块大小
// #define HEAP_BLOCK_COUNT        (HEAP_TOTAL_SIZE / HEAP_BLOCK_SIZE) 
// #define HEAP_BIT_MAP_ADDR       ((uint8_t*)HEAP_GUARD_ADDR)
// #define HEAP_BIT_MAP_SIZE       (HEAP_BLOCK_COUNT) // 位图大小
// #define HEAP_USED_BLOCK_COUNT   (*(uint16_t*)(HEAP_BIT_MAP_ADDR + HEAP_BLOCK_COUNT)) // 已用块数量
// #define HEAP_FREE_BLOCK_COUNT   (*(uint16_t*)(HEAP_BIT_MAP_ADDR + HEAP_BLOCK_COUNT + 2)) // 空闲块起始地址

// uint16_t get_used_heap_block_count(void) {
//     return HEAP_USED_BLOCK_COUNT; // 返回已用块数量
// }

// uint16_t get_free_heap_block_count(void) {
//     return HEAP_FREE_BLOCK_COUNT; // 返回空闲块数量
// }

// void* malloc(size_t size) {
//    if(size % HEAP_BLOCK_SIZE != 0) {
//         size += (HEAP_BLOCK_SIZE - (size % HEAP_BLOCK_SIZE)); // 向上对齐到最小块大小
//     }
//     // 超过可用堆大小
//     if(size > HEAP_BLOCK_SIZE * HEAP_FREE_BLOCK_COUNT) {
//         return NULL;
//     }
//     uint16_t block_count = size / HEAP_BLOCK_SIZE; // 计算需要的块数
//     // 位图查找最小空闲块, 需要加锁
//     uint16_t start_block = HEAP_BLOCK_COUNT;
//     uint16_t free_block_count = 0; // 连续空闲块计数
//     for(uint16_t i=0; i < HEAP_BLOCK_COUNT; i++){
//         if(free_block_count + (HEAP_BLOCK_COUNT - i) < block_count) {
//             break; // 剩余空闲块不足
//         }
//         if((HEAP_BIT_MAP_ADDR[i] & (1<<0)) || (HEAP_BIT_MAP_ADDR[i] & (1<<1))) { // 检查第 i 个块是否空闲或被标记为free
//             if(start_block == HEAP_BLOCK_COUNT) {
//                 start_block = i; // 记录第一个空闲块
//             }
//             free_block_count++; // 连续空闲块计数
//         }else{
//             if(free_block_count >= block_count) {
//                 // 找到足够的连续空闲块
//                 for(uint16_t j = start_block; j < start_block + block_count; j++) {
//                     HEAP_BIT_MAP_ADDR[j] |= 0x01;  // 标记为已用
//                     HEAP_BIT_MAP_ADDR[j] &= ~0x02;  // 清除 free 标志
//                     HEAP_BIT_MAP_ADDR[j] |= 0x04;   // 设置为连续分配标志
//                 }
//                 HEAP_BIT_MAP_ADDR[start_block + block_count - 1] &= ~0x04; // 设置最后一个块标志为0
//                 HEAP_USED_BLOCK_COUNT += block_count; // 更新已用块数量
//                 return (void*)(HEAP_START_ADDR + start_block * HEAP_BLOCK_SIZE); // 返回分配的内存地址
//             }
//             start_block = HEAP_BLOCK_COUNT;      // 重置起始块
//             free_block_count = 0;   // 重置连续空闲块计数
//         }
//     }
//     return NULL; // 没有足够的连续空闲块
// }

// void free(void* ptr) {
//     uint16_t block_start = ((uint8_t*)ptr - HEAP_START_ADDR) / HEAP_BLOCK_SIZE; // 计算块起始位置
//     if(block_start >= HEAP_BLOCK_COUNT) {
//         return; // 无效的块起始位置
//     }
//     if((HEAP_BIT_MAP_ADDR[block_start] ^ 0x01) || (HEAP_BIT_MAP_ADDR[block_start] & 0x02)) {
//         return; // 已经是空闲块
//     }
//     for(uint16_t i = block_start; i < HEAP_BLOCK_COUNT; i++) {
//         if((HEAP_BIT_MAP_ADDR[i] & 0x01) && (HEAP_BIT_MAP_ADDR[i] && 0x04)) { // 检查是否是已用块
//             HEAP_BIT_MAP_ADDR[i] &= ~0x01; // 清除已用标志
//             HEAP_BIT_MAP_ADDR[i] |= 0x02;  // 设置为 free 标志
//             HEAP_BIT_MAP_ADDR[i] &= ~0x04; // 清除连续分配标志
//         } else if((HEAP_BIT_MAP_ADDR[i] & 0x01) && (HEAP_BIT_MAP_ADDR[i] ^ 0x04)) {
//             HEAP_BIT_MAP_ADDR[i] |= 0x02;  // 设置为 free 标志
//             break;
//         }else if(HEAP_BIT_MAP_ADDR[i] ^ 0x01){
//             break; // 理论上不可能出现这种情况
//         }
//     }
// }

/*================================================================================*/


int main(){

    clock_init();
    // SystemClock_Config();
    systick_init();
    uart1_init();
    
    RCC_APB2ENR |= RCC_IOPBEN;      // 使能 GPIOC 时钟
    GPIOB_CRH &= ~(0xF << 16);      // 清除 PB12 的配置位
    GPIOB_CRH |=  (0x3 << 16);      // 设置 PB12 为推挽输出
    GPIOB_ODR |= (1 << 12); 
    
    uint8_t *buff = malloc(100);
    if (buff != NULL) {
        memset(buff, 0, 100); // 初始化分配的内存
    }

    memcpy(buff, "Hello, STM32!", 13); // 拷贝字符串到分配的内存
    printf("Buffer content: %s\r\n", buff);
    printf("Hello, %s! Number: %d, Char: %c\r\n", "STM32", 123, 'A');
    
    size_t size = malloc_usable_size(buff); // 获取分配内存的大小
    printf("Allocated buffer size: %lu bytes\r\n", size);
    printf("buff ptr: %p\r\n", buff);

    while (1){
        
        delay_ms(100);
        
        // GPIOB_ODR &= ~(1 << 12);
        // GPIOB_ODR &= ~(1 << 12);  // 切换 PB12 的状态
        // delay_ms(1000);
        // GPIOB_ODR |= (1 << 12);   // 切换 PB12 的状态
        // uart1_send_string("Hello, STM32!\r\n");
    }
    return 0;
}