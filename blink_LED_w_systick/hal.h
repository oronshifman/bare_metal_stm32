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
    volatile uint32_t MODER,    // mode register 
                      OTYPER,   // output type register 
                      OSPEEDR,  // output speed register 
                      PUPDR,    // pull-up/pull-down register 
                      IDR,      // input data register 
                      ODR,      // output data register 
                      BSRR,     // bit set/reset register 
                      LCKR,     // lock register 
                      AFRL,     // alternate function low register 
                      AFRH,     // alternate function high register 
                      BRR;      // bit reset register 
};

/**
 *  @brief creates a pointer to struct gpio for the requested bank (A, B, C, D or F)
 *
 *  Example use:
 *       GPIO(PINBANK(pin))
 *  where pin is any uint8_t value created by the PIN() macro
 */
#define GPIO(bank) ((struct gpio *)(0x50000000 + 0x400 * (bank)))

#define GPIO_BITS_PER_PIN_MODER (2)

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
    gpio->MODER |= (mode & 0b11U) << (pin_num * GPIO_BITS_PER_PIN_MODER);
}

/**
 * @brief set/reset the ODR for a given pin
 * @param pin a uint8_t created by PIN() macro
 * @param value eather true to set pin's ODR or false to reset it
 */
static inline void gpio_set_reset_output(uint8_t pin, bool value)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    gpio->BSRR = (1U << PINNO(pin)) << (value ? 0 : 16);
}

/********************************************************************************************
 * 
 * SisTick
 * 
 ********************************************************************************************/
#define SYSCLK_FREQ (48000000)

struct systick
{
    volatile uint32_t CSR,   // control and status register
                      RVR,   // reload value register
                      CVR,   // current value register
                      CALIB; // calibration value register
};
#define SYSTICK ((struct systick *)0xe000e010)

/**
 * This should be defined in main.c
 */
extern volatile uint32_t milliseconds_since_reset;

/**
 * @brief used for initiating systick
 * @param ticks The number of clock ticks to initialize systick with
 * 
 * Explanation:
 *      the following line will generate a SysTick interrupt every millisecond
 *          systick_init(SYSCLK_FREQ / 1000);
 */
static inline void systick_init(uint32_t ticks)
{
    if (ticks - 1 > 0xffffff)
    {
        return;
    }

    SYSTICK->RVR = ticks - 1; 
    SYSTICK->CVR = 0;
    SYSTICK->CSR = BIT(0) | BIT(1) | BIT(2);
    // RCC->APB2ENR |= BIT(14) // On the board used in the guide ther is a need to enable the 
                               // the clock for SysTick. On the STM32C031C6 board, the SysTick
                               // clock is enabled on MCU startup as the default value in 
                               // RCC_CR. This is, HSION is set to 1 on reset.
                                  
}

/********************************************************************************************
 * 
 * RCC
 * 
 ********************************************************************************************/
struct rcc
{
    volatile uint32_t CR,           // control register
                      ICRSCR,       // internal clock source calibration register 
                      CFGR,         // configuration register 
                      RESERVED[2], 
                      CRRCR,        // clock recovery RC register 
                      CIER,         // clock interrupt enable register 
                      CIFR,         // clock interrupt flag register 
                      CICR,         // clock interrupt clear register 
                      IOPRSTR,      // I/O port reset register 
                      AHBRSTR,      // AHB peripheral reset register 
                      APBRSTR1,     // APB peripheral reset register 1 
                      APBRSTR2,     // APB peripheral reset register 2 
                      IOPENR,       // I/O port clock enable register
                      AHBENR,       // AHB peripheral clock enable register
                      ABPENR1,      // APB peripheral clock enable register 1 
                      APBENR2,      // APB peripheral clock enable register 2
                      IOPSMENR,     // I/O port in Sleep mode clock enable register 
                      AHPSMENR,     // AHB peripheral clock enable in Sleep/Stop mode register 
                      APBSMENR1,    // APB peripheral clock enable in Sleep/Stop mode register 1 
                      APBSMENR2,    // APB peripheral clock enable in Sleep/Stop mode register 2
                      CCIPR,        // peripherals independent clock configuration register 1 
                      CCIPR2,       // peripherals independent clock configuration register 2
                      CSR1,         // control/status register 1 
                      CSR2;         // control/status register 2
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
