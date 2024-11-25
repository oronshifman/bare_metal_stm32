BIN = firmware

CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
          -Wformat-truncation -fno-common -Wconversion \
          -g3 -O0 -ffunction-sections -fdata-sections \
		  -I. -Icmsis_core/CMSIS/Core/Include -Icmsis_c0/Include\
          -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -mfpu=fpv4-sp-d16 

LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$(BIN).map

SOURCES = main.c cmsis_c0/Source/Templates/gcc/startup_stm32c031xx.s syscalls.c

build: $(BIN).elf

firmware.elf: cmsis_core cmsis_c0 hal.h link.ld Makefile $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: $(BIN).elf
	arm-none-eabi-objcopy -O binary $< $@

flash: $(BIN).bin
	st-flash --reset write $< 0x8000000

cmsis_core:
	git clone --depth 1 -b 5.9.0 https://github.com/ARM-software/CMSIS_5 $@ 

cmsis_c0:
	git clone --depth 1 -b v1.3.0 https://github.com/STMicroelectronics/cmsis-device-c0 $@

clean:
	rm -f $(BIN).*