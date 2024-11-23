#include "hal.h"
#include "IRQs.h"

volatile uint32_t milliseconds_since_reset = 0;

bool is_timer_expired(uint32_t *timer, uint32_t period, uint32_t now) 
{
    if (now + period < *timer) *timer = 0;               // Time wrapped? Reset timer
    if (*timer == 0) *timer = now + period;              // First poll? Set expiration
    if (*timer > now) return false;                      // Not expired yet, return
    *timer = (now - *timer) > period ? now + period : 
                                       *timer + period;  // Next expiration time
    return true;                                         // Expired, return true
}

int main(void)
{
    uint8_t blue_led = PIN('A', 0);
    uint8_t yellow_led = PIN('A', 1);
    uint8_t red_led = PIN('A', 4);

    RCC->IOPENR |= BIT(PINBANK(blue_led));

    gpio_set_mode(blue_led, OUTPUT);
    gpio_set_mode(yellow_led, OUTPUT);
    gpio_set_mode(red_led, OUTPUT);

    systick_init(systick_get_freq() / 1000);

    uint32_t timer, period = 500;
    while (1)
    {
        if (is_timer_expired(&timer, period, milliseconds_since_reset))
        {
            static bool on = true;

            gpio_set_reset_output(blue_led, on);
            gpio_set_reset_output(yellow_led, on);
            gpio_set_reset_output(red_led, on);
            
            on = !on;
        }
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
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = 
{
    _estack, _reset,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    SysTick_Handler
};