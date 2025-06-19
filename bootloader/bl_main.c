#include <stdint.h>


#define APP_BASE        0x08004000U
#define APP_STACK_PTR   (*(volatile uint32_t*)APP_BASE)
#define APP_RESET_ADDR  (*(volatile uint32_t*)(APP_BASE + 4))

#define SCB_VTOR_ADDRESS  ((volatile uint32_t*)0xE000ED08)

typedef void (*pFunction)(void);

static inline void set_msp(uint32_t topOfMainStack) {
    __asm volatile ("msr msp, %0" :: "r" (topOfMainStack) : );
}


void jump_to_app(void) {
    // 1. 关闭中断
    __asm volatile ("cpsid i"); // Disable interrupts

    // 2. 设置主栈指针
    set_msp(APP_STACK_PTR);

    // 3. 设置向量表地址
    *SCB_VTOR_ADDRESS = APP_BASE;

    // 4. 跳转到复位处理函数（即 app 的 startup.s 入口）
    pFunction app_entry = (pFunction)APP_RESET_ADDR;
    app_entry();
}


void boot_main(void) {
    jump_to_app();
    while (1);
}