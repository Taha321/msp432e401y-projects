#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "gpio.h"
#include "uart.h"

static uart_index_t uart_idx = UART_INDEX_0;

void wait() {
    volatile uint32_t ui32Loop;
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }
}

void led_init(void) {
    reg_sysctl->RCGCGPIO |= 1 << 12;

    __asm("nop");
    __asm("nop");
    __asm("nop"); 

    reg_gpio_pn->DIR |= 0xff;       // set port N as output
    reg_gpio_pn->AFSEL &= ~0xff;    // set afsel for port N
    reg_gpio_pn->PC &= ~0xff;       // gpiopc port N
    reg_gpio_pn->DR2R |= 0xff;      // 2ma drive on port N
    reg_gpio_pn->DR4R &= ~0xff;     // disable 4ma on port N
    reg_gpio_pn->DR8R &= ~0xff;     // disable 8ma on port N
    reg_gpio_pn->DEN |= 0xff;       // enable digital on port N
}

void uart_rx_callback(uint8_t c, void* args) {
    // echo back received character
    if(c == '\r' || c == '\n') {
        uart_print_char(uart_idx, '\n');
        uart_print_char(uart_idx, '\r');
        return;
    }
    uart_print_char(uart_idx, c);
}

int main(void)
{
    led_init();

    uart_init(uart_idx, UART_BAUD_115200);
    uart_set_rx_callback(uart_idx, uart_rx_callback, NULL);

    while(1)
    {   
        reg_gpio_pn->DATA1 = 1;
        wait();
        reg_gpio_pn->DATA1 = 0;
        wait();
    }

    uart_disable(uart_idx);

    return 0;   
}
