#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

void wait() {
    volatile uint32_t ui32Loop;
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }
}


int main(void)
{
    reg_sysctl->RCGCGPIO |= 0xffff;
    reg_sysctl->RCGCUART |= 0x1; // enable clock to UART0

    __asm("nop");
    __asm("nop");
    __asm("nop");    

//pin config
    reg_gpio_pa->AFSEL |= 0x3; // set PA0 and PA1 to afsel
    reg_gpio_pa->PCTL &= ~0xff; // clear PCTL bit field for PA0 and PA1
    reg_gpio_pa->PCTL |= 0x11; // set PA0 and PA
    reg_gpio_pa->DIR |= 0x3; // set output on PA0 and PA1
    reg_gpio_pa->DR2R |= 0x3; // 2ma drive on PA0 and PA1
    reg_gpio_pa->DR4R &= ~0xff;     // disable 4ma on port A
    reg_gpio_pa->DR8R &= ~0xff;     // disable 8ma on port A  
    reg_gpio_pa->DEN |= 0x3; // enable digital on PA0 and PA1

//uart config
    reg_uart0->CTL &= ~0x1; // disable uart0 while configuring
    reg_uart0->IBRD = 8; // uart baud rate 115200, int 8
    reg_uart0->FBRD = 44; // uart baud rate 115200, frac 44
    reg_uart0->LCRH = 0x70; // 8 bit, no parity, one stop bit, fifo enabled
    reg_uart0->CTL |= 0x01; // uart enable
    reg_uart0->CTL |= 0x300; // uart tx and rx enabl


    while(1)
    {   
        reg_uart0->TDR = 'A'; // send character 'A' over UART
        wait();
    }

    return 0;   
}
