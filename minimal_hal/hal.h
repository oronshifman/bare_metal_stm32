#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>

struct gpio
{
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH, BRR;
};

#define BITS_PER_PIN_MODER (2)

/**
 *  @brief creates a pointer to struct gpio for the requested bank (A, B, C, D or F)
 *
 *  Example use:
 *       GPIO(PINBANK(pin))
 *  where pin is any uint8_t value created by the PIN() macro
 */
#define GPIO(bank) ((struct gpio *)(0x50000000 + 0x400 * (bank)))

/**
 *  @brief creates a uint8_t value where its high 4 bits represent the bank and 
 *         its low 4 bits represent the pin number 
 * 
 *  Example use:
 *      PIN('A', 5) 
 *  Returns: 
 *      bank pin
 *      0000 0101
 */
#define PIN(bank, pin_num) ((uint8_t)((((bank) - 'A') << 4) | (pin_num & 0xf)))

/**
 * @brief extracts pin number from pin
 * @param pin a uint8_t created by PIN() macro
 */
#define PINNO(pin) (pin & 0xf)

/**
 * @brief extracts bank value (A, B, C, D or F) from pin
 * @param pin a uint8_t created by PIN() macro
 */
#define PINBANK(pin) (pin >> 4)

enum gpio_mode
{
    INPUT, OUTPUT, AF, ANALOG
};

/**
 * @brief sets gpio mode to <mode> at pin <pin>
 * @param pin a uint8_t created by PIN() macro
 * @param mode a enum gpio_mode value to set the pin to 
 *        this value can be:
 *            INPUT, OUTPUT, AF (alternate function), ANALOG
 */
static inline void gpio_set_pin(uint8_t pin, enum gpio_mode mode);

#endif /* HAL_H */
