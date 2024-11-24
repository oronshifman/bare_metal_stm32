#include "hal.h"

int main(void)
{
    uint8_t blue_led = PIN('A', 0);
    uint8_t yellow_led = PIN('A', 1);
    uint8_t red_led = PIN('A', 4);

    RCC->IOPENR |= BIT(PINBANK(blue_led));

    gpio_set_mode(blue_led, OUTPUT);
    gpio_set_mode(yellow_led, OUTPUT);
    gpio_set_mode(red_led, OUTPUT);

    while (1)
    {
        gpio_set_reset_output(blue_led, true);
        spin(999999);
        gpio_set_reset_output(blue_led, false);
        spin(999999);

        gpio_set_reset_output(yellow_led, true);
        spin(999999);
        gpio_set_reset_output(yellow_led, false);
        spin(999999);

        gpio_set_reset_output(red_led, true);
        spin(999999);
        gpio_set_reset_output(red_led, false);
        spin(999999);
    }

    return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) 
{
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;                     // Zeros out .bss
    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++; // Copies the data section from flash to RAM

    main(); 

    for (;;) (void) 0;  // Infinite loop
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 32 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = {_estack, _reset};