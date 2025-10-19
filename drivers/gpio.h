#ifndef __GPIO_H__
#define __GPIO_H__
#include "reg.h"

enum gpio_port {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_J,
    GPIO_PORT_K,
    GPIO_PORT_L,
    GPIO_PORT_M,
    GPIO_PORT_N,
    GPIO_PORT_P,
    GPIO_PORT_Q
};

void gpio_init(enum gpio_port port, uint8_t pin, uint8_t dir_mode);
void gpio_reset(enum gpio_port port, uint8_t pin);

void gpio_set_state(enum gpio_port port, uint8_t pins, int state);
void gpio_get_state(enum gpio_port port, uint8_t pins, int* state);




#endif // __GPIO_H__