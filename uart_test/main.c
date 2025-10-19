#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"

void wait() {
    volatile uint32_t ui32Loop;
    for(ui32Loop = 0; ui32Loop < 200000; ui32Loop++)
    {
    }
}

static int pn_led_state = 0;
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

void UART0_IRQHandler(void)
{
    uint32_t recieved_char = reg_uart0->TDR;

    if(recieved_char == '\r') {
        recieved_char = '\n';
        reg_uart0->TDR = recieved_char; // echo back received character
        while ((reg_uart0->FR >> 3) & 1) {} // wait until tx is completed
        reg_uart0->TDR = '\r'; // send carriage return after newline
        while ((reg_uart0->FR >> 3) & 1) {} // wait until tx is completed
        goto exit;
    }
    reg_uart0->TDR = recieved_char; // echo back received character
    while ((reg_uart0->FR >> 3) & 1) {} // wait until tx is completed

exit:
    reg_uart0->ICR = 0x10; // clear rx interrupt
}

int main(void)
{

    led_init();

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
    reg_uart0->LCRH = 0x60; // 8 bit, no parity, one stop bit, fifo enabled
    reg_uart0->IM |= 0x10; // enable rx interrupt

    HWREG(0xE000E100) |= 0x1 << 5; // enable interrupt number 5 (UART0)

    reg_uart0->CTL |= 0x01; // uart enable
    reg_uart0->CTL |= 0x300; // uart tx and rx enabl


    while(1)
    {   
        reg_gpio_pn->DATA1 = 1;
        wait();
        reg_gpio_pn->DATA1 = 0;
        wait();
    }

    return 0;   
}
