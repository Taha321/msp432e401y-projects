#ifndef __GPIO_REG_H__
#define __GPIO_REG_H__

#include <stdint.h>

typedef __attribute__((packed)) struct {
    volatile uint8_t RESERVED0[0x4];
    volatile uint8_t DATA0 : 1;
    volatile uint8_t RESERVED1 : 7;
    volatile uint8_t RESERVED2[0x3];
    volatile uint8_t RESERVED3 : 1;
    volatile uint8_t DATA1 : 1;
    volatile uint8_t RESERVED4 : 6;
    volatile uint8_t RESERVED5[0x7];
    volatile uint8_t RESERVED6 : 2;
    volatile uint8_t DATA2 : 1;
    volatile uint8_t RESERVED7 : 5;
    volatile uint8_t RESERVED8[0xf];
    volatile uint8_t RESERVED9 : 3;
    volatile uint8_t DATA3 : 1;
    volatile uint8_t RESERVED10 : 4;
    volatile uint8_t RESERVED11[0x1f];
    volatile uint8_t RESERVED12 : 4;
    volatile uint8_t DATA4 : 1;
    volatile uint8_t RESERVED13 : 3;
    volatile uint8_t RESERVED14[0x3f];
    volatile uint8_t RESERVED15 : 5;
    volatile uint8_t DATA5 : 1;
    volatile uint8_t RESERVED16 : 2;
    volatile uint8_t RESERVED17[0x7f];
    volatile uint8_t RESERVED18 : 6;
    volatile uint8_t DATA6 : 1;
    volatile uint8_t RESERVED19 : 1;
    volatile uint8_t RESERVED20[0xff];
    volatile uint8_t RESERVED21 : 7;
    volatile uint8_t DATA7 : 1;
    volatile uint8_t RESERVED22[0x1fc];
    volatile uint32_t DIR;
    volatile uint32_t IS;
    volatile uint32_t IBE;
    volatile uint32_t IEV;
    volatile uint32_t IM;
    volatile uint32_t RIS;
    volatile uint32_t MIS;
    volatile uint32_t ICR;
    volatile uint32_t AFSEL;
    volatile uint8_t  RESERVED23[0xdc];
    volatile uint32_t DR2R;
    volatile uint32_t DR4R;
    volatile uint32_t DR8R;
    volatile uint32_t ODR;
    volatile uint32_t PUR;
    volatile uint32_t PDR;
    volatile uint32_t SLR;
    volatile uint32_t DEN;
    volatile uint32_t LOCK;
    volatile uint32_t CR;
    volatile uint32_t AMSEL;
    volatile uint32_t PCTL;
    volatile uint32_t ADCCTL;
    volatile uint32_t DMACTL;
    volatile uint32_t OSI;
    volatile uint32_t DR12R;
    volatile uint32_t WAKEPEN;
    volatile uint32_t WAKELVL;
    volatile uint32_t WAKESTAT;
    volatile uint8_t RESERVED24[0xA74];
    volatile uint32_t PP;
    volatile uint32_t PC;
    volatile uint8_t RESERVED25[0x8];
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

} gpio_register_map_t;

#define reg_gpio_pa ((gpio_register_map_t*)0x40058000)
#define reg_gpio_pb ((gpio_register_map_t*)0x40059000)
#define reg_gpio_pc ((gpio_register_map_t*)0x4005A000)
#define reg_gpio_pd ((gpio_register_map_t*)0x4005B000)
#define reg_gpio_pe ((gpio_register_map_t*)0x4005C000)
#define reg_gpio_pf ((gpio_register_map_t*)0x4005D000)
#define reg_gpio_pg ((gpio_register_map_t*)0x4005E000)
#define reg_gpio_ph ((gpio_register_map_t*)0x4005F000)
#define reg_gpio_pj ((gpio_register_map_t*)0x40060000)
#define reg_gpio_pk ((gpio_register_map_t*)0x40061000)
#define reg_gpio_pl ((gpio_register_map_t*)0x40062000)
#define reg_gpio_pm ((gpio_register_map_t*)0x40063000)
#define reg_gpio_pn ((gpio_register_map_t*)0x40064000)
#define reg_gpio_pp ((gpio_register_map_t*)0x40065000)
#define reg_gpio_pq ((gpio_register_map_t*)0x40066000)
#define reg_gpio_pr ((gpio_register_map_t*)0x40067000)
#define reg_gpio_ps ((gpio_register_map_t*)0x40068000)
#define reg_gpio_pt ((gpio_register_map_t*)0x40069000)



#endif // __GPIO_REG_H__