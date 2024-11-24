#include "hal.h"

static volatile uint32_t milliseconds_since_reset;
void SysTick_Handler(void)
{
    milliseconds_since_reset++;
}

int main(void)
{
    uint8_t blue_led = PIN('A', 0);
    uint8_t yellow_led = PIN('A', 1);
    uint8_t red_led = PIN('A', 4);

    gpio_set_mode(blue_led, GPIO_MODE_OUTPUT);
    gpio_set_mode(yellow_led, GPIO_MODE_OUTPUT);
    gpio_set_mode(red_led, GPIO_MODE_OUTPUT);

    systick_init(systick_get_freq() / 1000);

    usart_init(USART2, 115200);

    uint32_t timer, period = 1000;
    while (1)
    {
        if (is_timer_expired(&timer, period, milliseconds_since_reset))
        {
            static bool on = true;

            gpio_set_reset_output(blue_led, on);
            gpio_set_reset_output(yellow_led, on);
            gpio_set_reset_output(red_led, on);

            printf("LED: %-6s ms since reset: %ld\r\n", on ? "on" : "off", milliseconds_since_reset);
            
            on = !on;
        }
    }

    return 0;
}