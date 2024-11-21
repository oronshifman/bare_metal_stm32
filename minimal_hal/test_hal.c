#include <stdio.h>
#include "hal.h"

int main(void)
{
    uint8_t pin = PIN('A', 5);
    printf("0x%x\n0x%x\n0x%x\n\n", pin, PINBANK(pin), PINNO(pin));

    pin = PIN('B', 15);
    printf("0x%x\n0x%x\n0x%x\n\n", pin, PINBANK(pin), PINNO(pin));

    pin = PIN('C', 6);
    printf("0x%x\n0x%x\n0x%x\n\n", pin, PINBANK(pin), PINNO(pin));
    return 0;
}