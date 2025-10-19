PROG?=uart_test

INCLUDE_DIR+=../../common
INCLUDE_DIR+=../../drivers

CC=arm-none-eabi-gcc
CPP=arm-none-eabi-g++

CFLAGS += -Wall -mcpu=cortex-m4 -march=armv7e-m -mthumb -std=c11 -Og -g
CPPFLAGS += -Wall -mcpu=cortex-m4 -march=armv7e-m -mthumb -std=c11
LDFLAGS = -nostdlib -mcpu=cortex-m4 -T ../../common/link.lds -Wl,-Map=final.map
VPATH += ../../common ../../drivers ../
OBJS+=main.o start_up.o

RULE_DIR=../../mk
include ${RULE_DIR}/Rules.mak


.PHONY: load


load: all
	openocd -f board/ti_msp432_launchpad.cfg -c "program ${TARGET_DIR}/${PROG}.elf verify reset exit"