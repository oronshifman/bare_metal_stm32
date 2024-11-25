#ifndef __HAL_H__
#define __HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "stm32c031xx.h"

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

/**
 *  @brief creates a pointer to struct gpio for the requested bank (A, B, C, D or F)
 *
 *  Example use:
 *       GPIO(PINBANK(pin))
 *  where pin is any uint8_t value created by the PIN() macro
 */
#define GPIO(bank) ((GPIO_TypeDef *)(IOPORT_BASE + 0x400 * (bank)))

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
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
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
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
    gpio->BSRR = BIT(PINNO(pin)) << (value ? 0 : 16);
}

/**
 * @brief sets the af of a specified pin
 * @param pin to change it's af
 * @param af_num selected af to change to
 */
static inline void gpio_set_af(uint8_t pin, uint8_t af_num) 
{
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  
  uint8_t pin_num = PINNO(pin); 

  volatile uint32_t *AFR = pin_num > 7 ? &gpio->AFR[1] : &gpio->AFR[0];
  *AFR &= ~(0xfUL << ((pin_num & 0b111) * GPIO_BITS_PER_PIN_AFR));
  *AFR |= ((uint32_t)af_num) << ((pin_num & 0b111) * GPIO_BITS_PER_PIN_AFR);
}

/********************************************************************************************
 * 
 * SisTick
 * 
 ********************************************************************************************/
#define HSI48_FREQ (48000000)

// TODO(24.11.24): finish doc
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
 * UART 
 * 
 ********************************************************************************************/

#define UART1 USART1
#define UART2 USART2

/**
 * @brief returns the status of the UARTx_RDR register.
 * @param uart the uart instance to check
 * @return 1 if UARTx_RDR register is ready to be read 0 if it is not
 */
static inline uint8_t uart_is_read_ready(USART_TypeDef *uart)
{
    return (uart->ISR & USART_ISR_RXNE_RXFNE_Msk) >> USART_ISR_RXNE_RXFNE_Pos;
}

/**
 * @brief return the status of the UARTx_TDR register. Call this function befor sending data and 
 *        assert the TDR register is empty and ready.
 * @param uart the uart instance to check
 * @return 1 if UARTx_TDR is empty and 0 if it is not
 */
static inline uint8_t uart_is_tx_empty(USART_TypeDef *uart)
{
    return (uart->ISR & USART_ISR_TXE_TXFNF_Msk) >> USART_ISR_TXE_TXFNF_Pos;
}

/**
 * @brief reads the data received in UARTx_RDR. It is recomended to check that the data is indeed
 *        available in the UARTx_RDR by calling uart_is_read_ready().
 * @param uart instance to read data from
 */
static inline uint8_t uart_read_byte(USART_TypeDef *uart)
{
    return (uint8_t)uart->RDR;
}

/**
 * @brief writes a byte to UARTx_TDR to be sent over the uart. NOTE: before sending new data
 *        UARTx_TDR must be asserted as empty and ready to be loaded with new data to be sent.
 *        This is done by calling uart_is_tx_empty().
 * @param uart instance to send over
 * @param data the data to be sent
 */
static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte)
{
    uart->TDR = byte;
}

/**
 * @brief send a buffer over uart
 * @param uart instance to send the data over
 * @param buf data to be sent
 * @param len length of the buffer to be sent
 */
static inline void uart_write_buffer(USART_TypeDef *uart, char *buf, size_t len)
{
    while (len--) 
    {
        while (!uart_is_tx_empty(uart));
        uart_write_byte(uart, *(uint8_t *)buf++);
    }
}

// TODO(24.11.24): finish doc
/**
 * @brief
 * @param baud
 * @return
 */
static inline uint16_t calculate_baud_rate(uint32_t baud)
{
    return (uint16_t)((float)systick_get_freq() / (float)baud + 0.5f);
}

/**
 * @brief asums oversampling by 16, word length of 8 bits, and 1 stop bit (all default
 *        reset values)
 * @param uart uart instance to be initiated and enabled
 * @param baud baud rate to be set to uart
 */
static inline void uart_init(USART_TypeDef *uart, uint32_t baud)
{
    // enable clock
    if (uart == UART1) RCC->APBENR2 |= BIT(14);
    if (uart == UART2) RCC->APBENR1 |= BIT(17);

    // set gpio pins 
    uint8_t tx = 0, rx = 0;
    if (uart == UART1) tx = PIN('B', 6), rx = PIN('B', 7);
    if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);

    // set af
    gpio_set_mode(tx, GPIO_MODE_AF);
    gpio_set_af(tx, 1);
    gpio_set_mode(rx, GPIO_MODE_AF);
    gpio_set_af(rx, 1);
    
    // set baud rate
    uart->CR1 = 0;
    uart->BRR = calculate_baud_rate(baud);

    // enable uart (USART_CR1_UE), transmit (USART_CR1_TE) and receive (USART_CR1_RE)
    uart->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
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
