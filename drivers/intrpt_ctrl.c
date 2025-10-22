#include <stdint.h>
#include "intrpt_ctrl.h"

#define NVIC_ISER_BASE_ADDR 0xE000E100
#define NVIC_ICER_BASE_ADDR 0xE000E180

void intrpt_ctrl_enable_irq(unsigned irq_num) {
    unsigned offset = irq_num / 32;
    unsigned bit_pos = irq_num % 32;

    volatile uint32_t* iser_reg = (volatile uint32_t*)(NVIC_ISER_BASE_ADDR + (offset * 4));
    *iser_reg |= (1 << bit_pos);
}
void intrpt_ctrl_disable_irq(unsigned irq_num) {
    unsigned offset = irq_num / 32;
    unsigned bit_pos = irq_num % 32;

    volatile uint32_t* icer_reg = (volatile uint32_t*)(NVIC_ICER_BASE_ADDR + (offset * 4));
    *icer_reg |= (1 << bit_pos);
}