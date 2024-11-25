#include "hal.h"

static volatile uint32_t milliseconds_since_reset;
void SysTick_Handler(void)
{
    milliseconds_since_reset++;
}

void SystemInit(void)
{
    SysTick_Config(systick_get_freq() / 1000);
}

int main(void)
{
    uint8_t on_board_LED = PIN('A', 5);

    gpio_set_mode(on_board_LED, GPIO_MODE_OUTPUT);

    uart_init(USART2, 115200);

    uint32_t timer, period = 1000;
    while (1)
    {
        if (is_timer_expired(&timer, period, milliseconds_since_reset))
        {
            static bool on = true;

            gpio_set_reset_output(on_board_LED, on);

            printf("LED: %-6s ms since reset: %ld\r\n", on ? "on" : "off", milliseconds_since_reset);
            
            on = !on;
        }
    }

    return 0;
}