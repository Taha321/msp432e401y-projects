#ifndef __UART_REG_H__
#define __UART_REG_H__


typedef __attribute__((packed)) struct {
    volatile uint32_t TDR;
    volatile uint32_t RSR;
    volatile uint8_t RESERVED0[0x10];
    volatile uint32_t FR;
    volatile uint8_t RESERVED1[0x4];
    volatile uint32_t ILPR;
    volatile uint32_t IBRD;
    volatile uint32_t FBRD;
    volatile uint32_t LCRH;
    volatile uint32_t CTL;
    volatile uint32_t IFLS;
    volatile uint32_t IM;
    volatile uint32_t RIS;
    volatile uint32_t MIS;
    volatile uint32_t ICR;
    volatile uint32_t DMACTL;
    volatile uint8_t RESERVED2[0x58];
    volatile uint32_t ADDR9BIT;
    volatile uint32_t AMASK9BIT;
    volatile uint8_t RESERVED3[0xf14];
    volatile uint32_t PP;
    volatile uint8_t RESERVED4[0x4];
    volatile uint32_t CC;
    volatile uint8_t RESERVED5[0x4];
    volatile uint32_t PeriphID4;
    volatile uint32_t PeriphID5;
    volatile uint32_t PeriphID6;
    volatile uint32_t PeriphID7;
    volatile uint32_t PeriphID0;
    volatile uint32_t PeriphID1;
    volatile uint32_t PeriphID2;
    volatile uint32_t PeriphID3;
    volatile uint32_t PCellID0;
    volatile uint32_t PCellID1;
    volatile uint32_t PCellID2;
    volatile uint32_t PCellID3;

} uart_register_map_t;

#define reg_uart0 ((uart_register_map_t*)0x4000C000)
#define reg_uart1 ((uart_register_map_t*)0x4000D000)
#define reg_uart2 ((uart_register_map_t*)0x4000E000)
#define reg_uart3 ((uart_register_map_t*)0x4000F000)
#define reg_uart4 ((uart_register_map_t*)0x40010000)
#define reg_uart5 ((uart_register_map_t*)0x40011000)
#define reg_uart6 ((uart_register_map_t*)0x40012000)
#define reg_uart7 ((uart_register_map_t*)0x40013000)




#endif // __UART_REG_H__