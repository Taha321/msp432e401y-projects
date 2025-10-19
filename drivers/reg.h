#ifndef __REG_H__
#define __REG_H__

#include "gpio_reg.h"
#include "sysctl_reg.h"
#include "uart_reg.h"

#define HWREG(x)                                                              \
        (*((volatile uint32_t *)(x)))

#endif // __REG_H__