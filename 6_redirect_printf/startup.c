// Startup code
__attribute__((naked, noreturn)) void _reset(void) 
{
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;                     // Zeros out .bss
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++; // Copies the data section from flash to RAM

    extern void main(void);
    main(); 

    for (;;) (void) 0;  // Infinite loop
}

extern void SysTick_Handler(void);  // Defined in IRQs.c
extern void _estack(void);          // Defined in link.ld

// 16 standard and 32 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = 
{
    _estack, _reset,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    SysTick_Handler
};