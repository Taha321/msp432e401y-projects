#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "uart.h"
#include "reg.h"
#include "intrpt_ctrl.h"

typedef struct {
    uart_register_map_t* uart_base;

    struct {
        unsigned port_idx;
        unsigned pin;
        uint8_t pin_ctl;
    } pin_config_rx;

    struct {
        unsigned port_idx;
        unsigned pin;
        uint8_t pin_ctl;
    } pin_config_tx;

    unsigned interrupt_num;

    uart_rx_callback_t rx_callback;
    void* rx_callback_args;

    bool enabled;

} uart_entry_t;

static gpio_register_map_t* gpio_port_list[] = {
    reg_gpio_pa,
    reg_gpio_pb,
    reg_gpio_pc,
    reg_gpio_pd,
    reg_gpio_pe,
    reg_gpio_pf,
    reg_gpio_pg,
    reg_gpio_ph,
    reg_gpio_pj,
    reg_gpio_pk,
    reg_gpio_pl,
    reg_gpio_pm,
    reg_gpio_pn,
    reg_gpio_pp,
    reg_gpio_pq,
    reg_gpio_pr,
    reg_gpio_ps,
    reg_gpio_pt
};

static uart_entry_t uart_list[] = {
    { reg_uart0, { 0, 0, 1 }, { 0, 1, 1 }, 5, NULL, NULL, false },
    { reg_uart1, { 14, 4, 1 }, { 1, 1, 1 }, 6, NULL, NULL, false },
    { reg_uart2, { 3, 4, 1 }, { 3, 5, 1 }, 33, NULL, NULL, false },
    { reg_uart3, { 0, 4, 1 }, { 0, 5, 1 }, 56, NULL, NULL, false },
    { reg_uart4, { 9, 0, 1 }, { 9, 1, 1 }, 57, NULL, NULL, false },
    { reg_uart5, { 2, 6, 1 }, { 2, 7, 1 }, 58, NULL, NULL, false },
    { reg_uart6, { 13, 0, 1 }, { 13, 1, 1 }, 59, NULL, NULL, false },
    { reg_uart7, { 2, 4, 1 }, { 2, 5, 1 }, 60, NULL, NULL, false },
};


int uart_init(uart_index_t index, uart_baud_t baudrate) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return -1; // invalid index
    }

    uart_register_map_t* uart = uart_list[idx].uart_base;
    gpio_register_map_t* port_tx = gpio_port_list[uart_list[idx].pin_config_tx.port_idx];
    gpio_register_map_t* port_rx = gpio_port_list[uart_list[idx].pin_config_rx.port_idx];

//enable clocks
    reg_sysctl->RCGCUART |= (1 << idx); // enable clock to uart
    reg_sysctl->RCGCGPIO |= (1 << uart_list[idx].pin_config_tx.port_idx); // enable clock to tx port
    reg_sysctl->RCGCGPIO |= (1 << uart_list[idx].pin_config_rx.port_idx); // enable clock to rx port

    __asm("nop");
    __asm("nop");
    __asm("nop"); 

//tx pin config
    port_tx->AFSEL |= (1 << uart_list[idx].pin_config_tx.pin); // set TX pin to afsel
    port_tx->PCTL &= ~(0xf << (uart_list[idx].pin_config_tx.pin * 4)); // clear PCTL bit field for TX pin
    port_tx->PCTL |= (uart_list[idx].pin_config_tx.pin_ctl << (uart_list[idx].pin_config_tx.pin * 4)); // set PCTL for TX pin
    port_tx->DIR |= (1 << uart_list[idx].pin_config_tx.pin); // set output on TX pin
    port_tx->DR2R |= (1 << uart_list[idx].pin_config_tx.pin); // 2ma drive on TX pin
    port_tx->DR4R &= ~(1 << uart_list[idx].pin_config_tx.pin);     // disable 4ma on TX pin
    port_tx->DR8R &= ~(1 << uart_list[idx].pin_config_tx.pin);     // disable 8ma on TX pin  
    port_tx->DEN |= (1 << uart_list[idx].pin_config_tx.pin); // enable digital on TX pin

//rx pin config
    port_rx->AFSEL |= (1 << uart_list[idx].pin_config_rx.pin); // set RX pin to afsel
    port_rx->PCTL &= ~(0xf << (uart_list[idx].pin_config_rx.pin * 4)); // clear PCTL bit field for RX pin
    port_rx->PCTL |= (uart_list[idx].pin_config_rx.pin_ctl << (uart_list[idx].pin_config_rx.pin * 4)); // set PCTL for RX pin
    port_rx->DIR &= ~(1 << uart_list[idx].pin_config_rx.pin); // set input on RX pin
    port_rx->DR2R |= (1 << uart_list[idx].pin_config_rx.pin); // 2ma drive on RX pin
    port_rx->DR4R &= ~(1 << uart_list[idx].pin_config_rx.pin);     // disable 4ma on RX pin
    port_rx->DR8R &= ~(1 << uart_list[idx].pin_config_rx.pin);     // disable 8ma on RX pin  
    port_rx->DEN |= (1 << uart_list[idx].pin_config_rx.pin); // enable digital on RX pin

//uart config
    uart->CTL &= ~0x1; // disable uart while configuring

    // Assuming system clock is 16MHz
    uint32_t sys_clock = 16000000;
    uint32_t baud_divisor = sys_clock / (16 * baudrate);
    uint32_t frac_part = ((sys_clock % (16 * baudrate)) * 64 + (baudrate / 2)) / baudrate;

    uart->IBRD = baud_divisor; // set integer baud rate divisor
    uart->FBRD = frac_part; // set fractional baud rate divisor
    uart->LCRH = 0x60; // 8 bit, no parity, one stop bit, fifo enabled
    uart->IM |= 0x10; // enable rx interrupt

    uart->CTL |= 0x01; // uart enable
    uart->CTL |= 0x300; // uart tx and rx enable

    intrpt_ctrl_enable_irq(uart_list[idx].interrupt_num);

    uart_list[idx].enabled = true;
    return 0;
}
int uart_disable(uart_index_t index) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return -1; // invalid index
    }

    uart_register_map_t* uart = uart_list[idx].uart_base;

    uart->CTL &= ~0x1; // disable uart

    intrpt_ctrl_disable_irq(uart_list[idx].interrupt_num);

    uart_list[idx].enabled = false;
    return 0;
}

int uart_set_rx_callback(uart_index_t index, uart_rx_callback_t callback, void* args) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return -1; // invalid index
    }

    uart_list[idx].rx_callback = callback;
    uart_list[idx].rx_callback_args = args;

    return 0;   
}

int uart_print_char(uart_index_t index, char c) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return -1; // invalid index
    }

    if(!uart_list[idx].enabled) {
        return -1; // uart not enabled
    }

    uart_register_map_t* uart = uart_list[idx].uart_base;

    while ((uart->FR >> 3) & 1) {} // wait until tx is completed
    uart->TDR = c; // send character

    return 0;   
}
int uart_print_string(uart_index_t index, const char* str) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return -1; // invalid index
    }

    if(!uart_list[idx].enabled) {
        return -1; // uart not enabled
    }

    uart_register_map_t* uart = uart_list[idx].uart_base;

    char* ptr = (char*)str;

    while(*ptr != '\0') {
        while ((uart->FR >> 3) & 1) {} // wait until tx is completed
        uart->TDR = *ptr; // send character
        ptr++;
    }

    return 0;
}

void uart_handler(uart_index_t index) {
    unsigned idx = (unsigned)index;

    if(idx >= sizeof(uart_list) / sizeof(uart_list[0])) {
        return; // invalid index
    }

    uart_register_map_t* uart = uart_list[idx].uart_base;

    uint8_t recieved_char = (uint8_t)(uart->TDR & 0xFF);

    // call user rx callback if set
    if(uart_list[idx].rx_callback != NULL) {
        uart_list[idx].rx_callback((char)recieved_char, uart_list[idx].rx_callback_args);
    }

    uart->ICR = 0x10; // clear rx interrupt
}

void uart0_irq_handler(void)
{
    uart_handler(UART_INDEX_0);
}

void uart1_irq_handler(void)
{
    uart_handler(UART_INDEX_1);
}

void uart2_irq_handler(void)
{
    uart_handler(UART_INDEX_2);
}

void uart3_irq_handler(void)
{
    uart_handler(UART_INDEX_3);
}

void uart4_irq_handler(void)
{
    uart_handler(UART_INDEX_4);
}

void uart5_irq_handler(void)
{
    uart_handler(UART_INDEX_5);
}

void uart6_irq_handler(void)
{
    uart_handler(UART_INDEX_6);
}

void uart7_irq_handler(void)
{
    uart_handler(UART_INDEX_7);
}
