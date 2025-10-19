#ifndef __SYSCTL_REG_H__
#define __SYSCTL_REG_H__

#include <stdint.h>

typedef __attribute__((packed)) struct {
    volatile uint8_t RESERVED0[0x600];
    volatile uint32_t RCGCWD;
    volatile uint32_t RCGCTIMER;
    volatile uint32_t RCGCGPIO;
    volatile uint16_t RCGCDMA;
    volatile uint32_t RCGCEPI;
    volatile uint32_t RCGCHIB;
    volatile uint32_t RCGCUART;
    volatile uint32_t RCGCSSI;
    volatile uint32_t RCGCI2C;
    volatile uint8_t RESERVED1[0x4];
    volatile uint32_t RCGCUSB;
    volatile uint8_t RESERVED2[0x4];
    volatile uint32_t RCGCEPHY;
    volatile uint32_t RCGCCAN;
    volatile uint32_t RCGCADC;
    volatile uint32_t RCGCACMP;
    volatile uint32_t RCGCPWM;
    volatile uint32_t RCGCQEI;
    volatile uint8_t RESERVED3[0x10];
    volatile uint32_t RCGCEEPROM;
    volatile uint8_t RESERVED4[0x18];
    volatile uint32_t RCGCCCM;
    volatile uint8_t RESERVED5[0x18];
    volatile uint32_t RCGCLCD;
    volatile uint8_t RESERVED6[0x4];
    volatile uint32_t RCGCOWIRE;
    volatile uint32_t RCGCEMAC;

} sysctl_register_map_t;

static sysctl_register_map_t* reg_sysctl = (sysctl_register_map_t*)0x400fe000;


#endif // __SYSCTL_REG_H__