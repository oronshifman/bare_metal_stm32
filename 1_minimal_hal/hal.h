#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>
#include <stdbool.h>

/********************************************************************************************
 * 
 * PIN ACCESS
 * 
 ********************************************************************************************/
/**
 * @brief creates bit in the x'th position
 */
#define BIT(x) (1UL << (x))

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

/********************************************************************************************
 * 
 * GPIO
 * 
 ********************************************************************************************/
struct gpio
{
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH, BRR;
};

#define GPIO_BITS_PER_PIN_MODER (2)

/**
 *  @brief creates a pointer to struct gpio for the requested bank (A, B, C, D or F)
 *
 *  Example use:
 *       GPIO(PINBANK(pin))
 *  where pin is any uint8_t value created by the PIN() macro
 */
#define GPIO(bank) ((struct gpio *)(0x50000000 + 0x400 * (bank)))

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
static inline void gpio_set_mode(uint8_t pin, enum gpio_mode mode)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    uint8_t pin_num = PINNO(pin);
    gpio->MODER &= ~(0b11U << (pin_num * GPIO_BITS_PER_PIN_MODER));
    gpio->MODER |= mode << (pin_num * GPIO_BITS_PER_PIN_MODER);
}

/**
 * @brief set/reset the ODR for a given pin
 * @param pin a uint8_t created by PIN() macro
 * @param value eather true to set pin's ODR or false to reset it
 */
static inline void gpio_set_reset_output(uint8_t pin, bool value)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    gpio->BSRR = BIT(PINNO(pin)) << (value ? 0 : 16);
}

/********************************************************************************************
 * 
 * RCC
 * 
 ********************************************************************************************/

struct rcc
{
    volatile uint32_t CR, ICRSCR, CFGR, CRRCR, CIER, CIFR, CICR, IOPRSTR, AHBRSTR, APBRSTR1, 
    APBRSTR2, IOPENR, AHBENR, ABPENR1, APBENR2, IOPSMENR, AHPSMENR, APBSMENR1, APBSMENR2, 
    CCIPR, CCIPR2, CSR1, CSR2;
};
#define RCC ((struct rcc *)0x40021000)

/********************************************************************************************
 * 
 * GENERAL FUNCTION 
 * 
 ********************************************************************************************/

/**
 * @brief an inaccurate delay function
 * @param count number of NOP to execute
 */
static inline void spin(volatile uint32_t count)
{
    while (count--)
    {
        (void) 0;
    }
}

#endif /* HAL_H */
