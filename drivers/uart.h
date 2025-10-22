#ifndef __UART_H__
#define __UART_H__


typedef enum {
    UART_INDEX_0 = 0,
    UART_INDEX_1,
    UART_INDEX_2,
    UART_INDEX_3,
    UART_INDEX_4,
    UART_INDEX_5,
    UART_INDEX_6,
    UART_INDEX_7
} uart_index_t;

typedef enum {
    UART_BAUD_9600 = 9600,
    UART_BAUD_19200 = 19200,
    UART_BAUD_38400 = 38400,
    UART_BAUD_57600 = 57600,
    UART_BAUD_115200 = 115200
} uart_baud_t;

typedef void (*uart_rx_callback_t)(char c, void* args);

int uart_init(uart_index_t index, uart_baud_t baudrate);
int uart_disable(uart_index_t index);

int uart_set_rx_callback(uart_index_t index, uart_rx_callback_t callback, void* args);

int uart_print_char(uart_index_t index, char c);
int uart_print_string(uart_index_t index, const char* str);








#endif // __UART_H__