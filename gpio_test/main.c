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

    __asm("nop");
    __asm("nop");
    __asm("nop");    

    reg_gpio_pd->DIR |= 0xff;       // set port D as output 
    reg_gpio_pd->AFSEL &= ~0xff;    // set afsel for port D 
    reg_gpio_pd->PC &= ~0xff;       // gpiopc port D
    reg_gpio_pd->DR2R |= 0xff;      // 2ma drive on port D 
    reg_gpio_pd->DR4R &= ~0xff;     // disable 4ma on port D     
    reg_gpio_pd->DR8R &= ~0xff;     // disable 8ma on port D     
    reg_gpio_pd->DEN |= 0xff;       // enable digital on port D

    reg_gpio_pn->DIR |= 0xff;       // set port N as output
    reg_gpio_pn->AFSEL &= ~0xff;    // set afsel for port N
    reg_gpio_pn->PC &= ~0xff;       // gpiopc port N
    reg_gpio_pn->DR2R |= 0xff;      // 2ma drive on port N
    reg_gpio_pn->DR4R &= ~0xff;     // disable 4ma on port N
    reg_gpio_pn->DR8R &= ~0xff;     // disable 8ma on port N
    reg_gpio_pn->DEN |= 0xff;       // enable digital on port N

    while(1)
    {
        reg_gpio_pd->DATA4 = 1;
        reg_gpio_pd->DATA5 = 0;
        reg_gpio_pn->DATA1 = 1;
        wait();
        reg_gpio_pd->DATA4 = 0;
        reg_gpio_pd->DATA5 = 1;
        reg_gpio_pn->DATA1 = 0;
        wait();
    }

    return 0;   
}
