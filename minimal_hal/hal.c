#include "hal.h"

static inline void gpio_set_pin(uint8_t pin, enum gpio_mode mode)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    uint8_t pin_num = PINNO(pin);
    gpio->MODER &= ~(0b11 << (pin_num * BITS_PER_PIN_MODER));
    gpio->MODER |= mode << (pin_num * BITS_PER_PIN_MODER);
}
