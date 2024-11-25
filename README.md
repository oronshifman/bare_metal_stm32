# STM32C031C6 Bare-Metal Learning Project

This repository is a step-by-step bare-metal learning project for the STM32C031C6-NUCLEO board. It is based on the [Bare-Metal Programming Guide](https://github.com/cpq/bare-metal-programming-guide) by cpq, adapted for the STM32C0xx family. Each directory represents a chapter in the guide, progressively enhancing functionality and showcasing the principles of bare-metal programming.

---

## Project Overview

The **Bare-Metal Programming Guide** is an excellent resource for developers who want to learn how to program microcontrollers using only a GCC compiler and a datasheet, without relying on frameworks. It teaches the fundamentals of embedded systems programming, helping you understand how frameworks like STM32Cube, Keil, and Arduino operate under the hood.

This project follows the guide and adapts its examples for the STM32C031C6 microcontroller. By the end of this project, you'll understand how to create bare-metal firmware from scratch, including implementing basic peripherals and building a minimal CMSIS project.

Each folder contains fully independen working code.

This code can be use as a reference for adapring the code in the original guide to the STM32C0xx family.

---

## Prerequisites

Before working with this project, ensure you have the following:

- An STM32C031C6-NUCLEO board.
- GCC-based ARM toolchain (e.g., `arm-none-eabi-gcc`).
- A basic understanding of C programming and microcontroller architecture.

---

## How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/oronshifman/stm32-bare-metal-learning.git
   cd stm32-bare-metal-learning
   ```

2. Navigate to a specific directory (e.g., `3_blink_LED`) and inspect the code.

3. Build and flash the project using your GCC toolchain:
   ```bash
   make
   make flash
   ```

4. Follow the guide to understand the functionality and learn the concepts behind the code.

---

## About the Bare-Metal Programming Guide

The original guide covers the following advanced projects in addition to the basics:

- **blinky**: Blink an LED and print debug messages periodically.
- **cli**: A UART-based command-line interface for interacting with the board.
- **lfs**: Implement file functions using `littlefs` in flash memory.
- **webui**: Build an embedded web server with a professional UI using the Mongoose library.

This project focuses on the introductory steps tailored for the STM32C031C6 but can serve as a foundation for more advanced development.

---

## Credits

- Original guide by cpq: [Bare-Metal Programming Guide](https://github.com/cpq/bare-metal-programming-guide).
- STM32 HAL and CMSIS libraries: [STMicroelectronics](https://www.st.com/).

---

## License

This project follows the same license as the Bare-Metal Programming Guide. Please refer to the [LICENSE](https://github.com/cpq/bare-metal-programming-guide/blob/master/LICENSE) file in the original repository.

---

Happy coding! 🚀
