#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

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
                      APBENR1,      // APB peripheral clock enable register 1 
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

/**
 * Definitions for bit positions and masks
 */
#define RCC_CR_HSIDIV_POS (11)
#define RCC_CR_HSIDIV_MASK (0b111 << RCC_CR_HSIDIV_POS)

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
#define GPIO_BITS_PER_PIN_AFR (4)

enum gpio_mode
{
    GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG
};

/**
 * @brief sets gpio mode to <mode> at pin <pin> and enables the specified pins port clock
 * @param pin a uint8_t created by PIN() macro
 * @param mode a enum gpio_mode value to set the pin to 
 *        this value can be:
 *            INPUT, OUTPUT, AF (alternate function), ANALOG
 */
static inline void gpio_set_mode(uint8_t pin, enum gpio_mode mode)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    uint8_t pin_num = PINNO(pin);

    RCC->IOPENR |= BIT(PINBANK(pin));

    gpio->MODER &= ~(0b11UL << (pin_num * GPIO_BITS_PER_PIN_MODER));
    gpio->MODER |= (mode & 0b11UL) << (pin_num * GPIO_BITS_PER_PIN_MODER);
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

/**
 * @brief sets the af of a specified pin
 * @param pin to change it's af
 * @param af_num selected af to change to
 */
static inline void gpio_set_af(uint8_t pin, uint8_t af_num) 
{
  struct gpio *gpio = GPIO(PINBANK(pin));  
  uint8_t pin_num = PINNO(pin); 

  volatile uint32_t *AFR = pin_num > 7 ? &gpio->AFRH : &gpio->AFRL;
  *AFR &= ~(0xfUL << ((pin_num & 0b111) * GPIO_BITS_PER_PIN_AFR));
  *AFR |= ((uint32_t)af_num) << ((pin_num & 0b111) * GPIO_BITS_PER_PIN_AFR);
}

/********************************************************************************************
 * 
 * SisTick
 * 
 ********************************************************************************************/
#define HSI48_FREQ (48000000)

struct systick
{
    volatile uint32_t CSR,   // control and status register
                      RVR,   // reload value register
                      CVR,   // current value register
                      CALIB; // calibration value register
};
#define SYSTICK ((struct systick *)0xe000e010)

/**
 * @brief used for initiating systick
 * @param ticks The number of clock ticks to initialize systick with
 * 
 * Explanation:
 *      the following line will generate a SysTick interrupt every millisecond
 *          systick_init(systick_get_freq() / 1000);
 */
static inline void systick_init(uint32_t ticks)
{
    if (ticks - 1 > 0xffffff) return;

    SYSTICK->RVR = ticks - 1; 
    SYSTICK->CVR = 0;
    SYSTICK->CSR = BIT(0) | BIT(1) | BIT(2);
    // RCC->APB2ENR |= BIT(14) // On the board used in the guide ther is a need to enable the 
                               // the clock for SysTick. On the STM32C031C6 board, the SysTick
                               // clock is enabled on MCU startup as the default value in 
                               // RCC_CR. This is, HSION is set to 1 on reset.
                                  
}

/**
 * @brief
 * @param timer
 * @param period
 * @param now
 * @return
 */
static inline bool is_timer_expired(uint32_t *timer, uint32_t period, uint32_t now) 
{
    if (now + period < *timer) *timer = 0;               // Time wrapped? Reset timer
    if (*timer == 0) *timer = now + period;              // First poll? Set expiration
    if (*timer > now) return false;                      // Not expired yet, return
    *timer = (now - *timer) > period ? now + period : 
                                       *timer + period;  // Next expiration time
    return true;                                         // Expired, return true
}

/**
 * @brief calculates the systick frequency as a product of HSI48 / HSIDIV. This is vaguely stated in
 *        the reference manual, page 113 where it says that HSISYS is derived from HSI48 through 
 *        division in a factor programmable (meaning HSIDIV) from 1 to 128.
 * @return systick frequency in Hz
 */
static inline uint32_t systick_get_freq(void)
{
    return HSI48_FREQ / (BIT((RCC->CR & (RCC_CR_HSIDIV_MASK)) >> RCC_CR_HSIDIV_POS));
}

/********************************************************************************************
 * 
 * USART 
 * 
 ********************************************************************************************/
struct usart
{
    volatile uint32_t CR1,      // control register 1
                      CR2,      // control register 2
                      CR3,      // control register 3
                      BRR,      // baud rate register 
                      GTPR,     // guard time and prescaler register 
                      RTOR,     // receiver timeout register 
                      RQR,      // request register 
                      ISR,      // interrupt and status register 
                      ICR,      // interrupt flag clear register 
                      RDR,      // receive data register 
                      TDR,      // transmit data register
                      PRESC;    // prescaler register 
};
#define USART1 ((struct usart *)0x40013800)
#define USART2 ((struct usart *)0x40004400)

/**
 * General usart helper definitions
 */
#define USART_BITS_IN_BRR (16)

/**
 * Definitions for bit positions
 */
#define USART_ISR_TXE_POS (7)                     // Transmit data register empty
#define USART_ISR_RXNE_POS (5)                    // Read data register not empty

/**
 * @brief returns the status of the USARTx_RDR register.
 * @param usart the usart instance to check
 * @return 1 if USARTx_RDR register is ready to be read 0 if it is not
 */
static inline uint8_t usart_is_read_ready(struct usart *usart)
{
    return (usart->ISR & BIT(USART_ISR_RXNE_POS)) >> USART_ISR_RXNE_POS;
}

/**
 * @brief return the status of the USARTx_TDR register. Call this function befor sending data and 
 *        assert the TDR register is empty and ready.
 * @param usart the usart instance to check
 * @return 1 if USARTx_TDR is empty and 0 if it is not
 */
static inline uint8_t usart_is_tx_empty(struct usart *usart)
{
    return (usart->ISR & BIT(USART_ISR_TXE_POS)) >> USART_ISR_TXE_POS;
}

/**
 * @brief reads the data received in USARTx_RDR. It is recomended to check that the data is indeed
 *        available in the USARTx_RDR by calling usart_is_read_ready().
 * @param usart instance to read data from
 */
static inline uint8_t usart_read_byte(struct usart *usart)
{
    return (uint8_t)usart->RDR;
}

/**
 * @brief writes a byte to USARTx_TDR to be sent over the USART. NOTE: before sending new data
 *        USARTx_TDR must be asserted as empty and ready to be loaded with new data to be sent.
 *        This is done by calling usart_is_tx_empty().
 * @param usart instance to send over
 * @param data the data to be sent
 */
static inline void usart_write_byte(struct usart *usart, uint8_t byte)
{
    usart->TDR = byte;
}

/**
 * @brief send a buffer over usart
 * @param usart instance to send the data over
 * @param buf data to be sent
 * @param len length of the buffer to be sent
 */
static inline void usart_write_buffer(struct usart *usart, char *buf, size_t len)
{
    while (len--) 
    {
        while (!usart_is_tx_empty(usart));
        usart_write_byte(usart, *(uint8_t *)buf++);
    }
}

/**
 * @brief
 * @param baud
 * @return
 */
static inline uint16_t calculate_baud_rate(uint32_t baud)
{
    float divided_freq = (float)systick_get_freq() / (float)baud;

    return (uint16_t)((divided_freq * USART_BITS_IN_BRR) + 0.5f);
}

/**
 * @brief asums oversampling by 16, word length of 8 bits, and 1 stop bit (all default
 *        reset values)
 * @param usart usart instance to be initiated and enabled
 * @param baud baud rate to be set to usart
 */
static inline void usart_init(struct usart *usart, uint32_t baud)
{
    // enable clock
    if (usart == USART1) RCC->APBENR2 |= BIT(14);
    if (usart == USART2) RCC->APBENR1 |= BIT(17);

    // set gpio pins 
    uint8_t tx = 0, rx = 0;
    if (usart == USART1) tx = PIN('B', 6), rx = PIN('B', 7);
    if (usart == USART2) tx = PIN('A', 2), rx = PIN('A', 3);

    // set af
    gpio_set_mode(tx, GPIO_MODE_AF);
    gpio_set_af(tx, 1);
    gpio_set_mode(rx, GPIO_MODE_AF);
    gpio_set_af(rx, 1);
    
    // set baud rate
    usart->CR1 = 0;
    usart->BRR = (uint16_t)((float)systick_get_freq() / (float)baud + 0.5f);

    // enable usart (bit(0)), transmit (bit(2)) and receive (bit(3))
    usart->CR1 = BIT(0) | BIT(2) | BIT(3);
}

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
    while (count--) (void) 0;
}

#endif /* HAL_H */
