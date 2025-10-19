#include <stdint.h>

#define SRAM_START 0x20000000U
#define SRAM_SIZE (256U * 1024U)
#define SRAM_END ((SRAM_START) + (SRAM_SIZE))

#define STACK_START SRAM_END

extern uint32_t _sbss;
extern uint32_t _sdata;
extern uint32_t _etext;
extern uint32_t _ebss;
extern uint32_t _edata;
extern uint32_t _la_data;

extern void main(void);

extern void Reset_Handler(void);
void Default_Handler                (void) __attribute__((weak));

/* Cortex-M4 Processor Exceptions */
 void NMI_Handler             (void) __attribute__((weak, alias("Default_Handler")));
 void HardFault_Handler       (void) __attribute__((weak, alias("Default_Handler")));
 void MemManage_Handler       (void) __attribute__((weak, alias("Default_Handler")));
 void BusFault_Handler        (void) __attribute__((weak, alias("Default_Handler")));
 void UsageFault_Handler      (void) __attribute__((weak, alias("Default_Handler")));
 void SVC_Handler             (void) __attribute__((weak, alias("Default_Handler")));
 void DebugMon_Handler        (void) __attribute__((weak, alias("Default_Handler")));
 void PendSV_Handler          (void) __attribute__((weak, alias("Default_Handler")));
 void SysTick_Handler         (void) __attribute__((weak, alias("Default_Handler")));

/* device specific interrupt handler */
 void GPIOA_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOB_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOC_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOD_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOE_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART0_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART1_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void SSI0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void PWM0_FAULT_IRQHandler   (void) __attribute__((weak, alias("Default_Handler")));
 void PWM0_0_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void PWM0_1_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void PWM0_2_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void QEI0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void ADC0SS0_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC0SS1_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC0SS2_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC0SS3_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void WATCHDOG_IRQHandler     (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER0A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER0B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER1A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER1B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER2A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER2B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void COMP0_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void COMP1_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void COMP2_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void SYSCTL_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void FLASH_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOF_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOG_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOH_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART2_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void SSI1_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER3A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER3B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void I2C1_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void CAN0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void CAN1_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void EMAC0_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void HIBERNATE_IRQHandler    (void) __attribute__((weak, alias("Default_Handler")));
 void USB0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void PWM0_3_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void UDMA_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void UDMAERR_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC1SS0_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC1SS1_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC1SS2_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void ADC1SS3_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void EPI0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOJ_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOK_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOL_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void SSI2_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void SSI3_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void UART3_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART4_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART5_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART6_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void UART7_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void I2C2_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C3_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER4A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER4B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER5A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER5B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void SYSEXC_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void I2C4_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C5_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOM_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void GPION_IRQHandler        (void) __attribute__((weak, alias("Default_Handler")));
 void TAMPER0_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP0_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP1_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP2_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP3_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP4_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP5_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP6_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOP7_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ0_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ1_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ2_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ3_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ4_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ5_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ6_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void GPIOQ7_IRQHandler       (void) __attribute__((weak, alias("Default_Handler")));
 void SHA0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void AES0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void DES0_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER6A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER6B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER7A_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void TIMER7B_IRQHandler      (void) __attribute__((weak, alias("Default_Handler")));
 void I2C6_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C7_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C8_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));
 void I2C9_IRQHandler         (void) __attribute__((weak, alias("Default_Handler")));


/* The vector table.  Note that the proper constructs must be placed on this to */
/* ensure that it ends up at physical address 0x0000.0000 or at the start of    */
/* the program if located at a start address other than 0.                      */

void (* const interruptVectors[])(void) __attribute__ ((section (".isr_vector"))) =
{
    (void (* const)(void))STACK_START,
    Reset_Handler,                          /* The reset handler         */
    NMI_Handler,                            /* The NMI handler           */
    HardFault_Handler,                      /* The hard fault handler    */
    MemManage_Handler,                      /* The MPU fault handler     */
    BusFault_Handler,                       /* The bus fault handler     */
    UsageFault_Handler,                     /* The usage fault handler   */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    SVC_Handler,                            /* SVCall handler            */
    DebugMon_Handler,                       /* Debug monitor handler     */
    0,                                      /* Reserved                  */
    PendSV_Handler,                         /* The PendSV handler        */
    SysTick_Handler,                        /* The SysTick handler       */
    GPIOA_IRQHandler,                       /* GPIO Port A               */
    GPIOB_IRQHandler,                       /* GPIO Port B               */
    GPIOC_IRQHandler,                       /* GPIO Port C               */
    GPIOD_IRQHandler,                       /* GPIO Port D               */
    GPIOE_IRQHandler,                       /* GPIO Port E               */
    UART0_IRQHandler,                       /* UART0 Rx and Tx           */
    UART1_IRQHandler,                       /* UART1 Rx and Tx           */
    SSI0_IRQHandler,                        /* SSI0 Rx and Tx            */
    I2C0_IRQHandler,                        /* I2C0 Master and Slave     */
    PWM0_FAULT_IRQHandler,                  /* PWM Fault                 */
    PWM0_0_IRQHandler,                      /* PWM Generator 0           */
    PWM0_1_IRQHandler,                      /* PWM Generator 1           */
    PWM0_2_IRQHandler,                      /* PWM Generator 2           */
    QEI0_IRQHandler,                        /* Quadrature Encoder 0      */
    ADC0SS0_IRQHandler,                     /* ADC Sequence 0            */
    ADC0SS1_IRQHandler,                     /* ADC Sequence 1            */
    ADC0SS2_IRQHandler,                     /* ADC Sequence 2            */
    ADC0SS3_IRQHandler,                     /* ADC Sequence 3            */
    WATCHDOG_IRQHandler,                    /* Watchdog timer            */
    TIMER0A_IRQHandler,                     /* Timer 0 subtimer A        */
    TIMER0B_IRQHandler,                     /* Timer 0 subtimer B        */
    TIMER1A_IRQHandler,                     /* Timer 1 subtimer A        */
    TIMER1B_IRQHandler,                     /* Timer 1 subtimer B        */
    TIMER2A_IRQHandler,                     /* Timer 2 subtimer A        */
    TIMER2B_IRQHandler,                     /* Timer 2 subtimer B        */
    COMP0_IRQHandler,                       /* Analog Comparator 0       */
    COMP1_IRQHandler,                       /* Analog Comparator 1       */
    COMP2_IRQHandler,                       /* Analog Comparator 2       */
    SYSCTL_IRQHandler,                      /* System Control            */
    FLASH_IRQHandler,                       /* FLASH Control             */
    GPIOF_IRQHandler,                       /* GPIO Port F               */
    GPIOG_IRQHandler,                       /* GPIO Port G               */
    GPIOH_IRQHandler,                       /* GPIO Port H               */
    UART2_IRQHandler,                       /* UART2 Rx and Tx           */
    SSI1_IRQHandler,                        /* SSI1 Rx and Tx            */
    TIMER3A_IRQHandler,                     /* Timer 3 subtimer A        */
    TIMER3B_IRQHandler,                     /* Timer 3 subtimer B        */
    I2C1_IRQHandler,                        /* I2C1 Master and Slave     */
    CAN0_IRQHandler,                        /* CAN0                      */
    CAN1_IRQHandler,                        /* CAN1                      */
    EMAC0_IRQHandler,                       /* Ethernet                  */
    HIBERNATE_IRQHandler,                   /* Hibernate                 */
    USB0_IRQHandler,                        /* USB0                      */
    PWM0_3_IRQHandler,                      /* PWM Generator 3           */
    UDMA_IRQHandler,                        /* uDMA Software Transfer    */
    UDMAERR_IRQHandler,                     /* uDMA Error                */
    ADC1SS0_IRQHandler,                     /* ADC1 Sequence 0           */
    ADC1SS1_IRQHandler,                     /* ADC1 Sequence 1           */
    ADC1SS2_IRQHandler,                     /* ADC1 Sequence 2           */
    ADC1SS3_IRQHandler,                     /* ADC1 Sequence 3           */
    EPI0_IRQHandler,                        /* External Bus Interface 0  */
    GPIOJ_IRQHandler,                       /* GPIO Port J               */
    GPIOK_IRQHandler,                       /* GPIO Port K               */
    GPIOL_IRQHandler,                       /* GPIO Port L               */
    SSI2_IRQHandler,                        /* SSI2 Rx and Tx            */
    SSI3_IRQHandler,                        /* SSI3 Rx and Tx            */
    UART3_IRQHandler,                       /* UART3 Rx and Tx           */
    UART4_IRQHandler,                       /* UART4 Rx and Tx           */
    UART5_IRQHandler,                       /* UART5 Rx and Tx           */
    UART6_IRQHandler,                       /* UART6 Rx and Tx           */
    UART7_IRQHandler,                       /* UART7 Rx and Tx           */
    I2C2_IRQHandler,                        /* I2C2 Master and Slave     */
    I2C3_IRQHandler,                        /* I2C3 Master and Slave     */
    TIMER4A_IRQHandler,                     /* Timer 4 subtimer A        */
    TIMER4B_IRQHandler,                     /* Timer 4 subtimer B        */
    TIMER5A_IRQHandler,                     /* Timer 5 subtimer A        */
    TIMER5B_IRQHandler,                     /* Timer 5 subtimer B        */
    SYSEXC_IRQHandler,                      /* FPU                       */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    I2C4_IRQHandler,                        /* I2C4 Master and Slave     */
    I2C5_IRQHandler,                        /* I2C5 Master and Slave     */
    GPIOM_IRQHandler,                       /* GPIO Port M               */
    GPION_IRQHandler,                       /* GPIO Port N               */
    0,                                      /* Reserved                  */
    TAMPER0_IRQHandler,                     /* Tamper                    */
    GPIOP0_IRQHandler,                      /* GPIO Port P(Summary or P0)*/
    GPIOP1_IRQHandler,                      /* GPIO Port P1              */
    GPIOP2_IRQHandler,                      /* GPIO Port P2              */
    GPIOP3_IRQHandler,                      /* GPIO Port P3              */
    GPIOP4_IRQHandler,                      /* GPIO Port P4              */
    GPIOP5_IRQHandler,                      /* GPIO Port P5              */
    GPIOP6_IRQHandler,                      /* GPIO Port P6              */
    GPIOP7_IRQHandler,                      /* GPIO Port P7              */
    GPIOQ0_IRQHandler,                      /* GPIO Port Q(Summary or Q0)*/
    GPIOQ1_IRQHandler,                      /* GPIO Port Q1              */
    GPIOQ2_IRQHandler,                      /* GPIO Port Q2              */
    GPIOQ3_IRQHandler,                      /* GPIO Port Q3              */
    GPIOQ4_IRQHandler,                      /* GPIO Port Q4              */
    GPIOQ5_IRQHandler,                      /* GPIO Port Q5              */
    GPIOQ6_IRQHandler,                      /* GPIO Port Q6              */
    GPIOQ7_IRQHandler,                      /* GPIO Port Q7              */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    SHA0_IRQHandler,                        /* SHA/MD5 0                 */
    AES0_IRQHandler,                        /* AES 0                     */
    DES0_IRQHandler,                        /* DES3DES 0                 */
    0,                                      /* Reserved                  */
    TIMER6A_IRQHandler,                     /* Timer 6 subtimer A        */
    TIMER6B_IRQHandler,                     /* Timer 6 subtimer B        */
    TIMER7A_IRQHandler,                     /* Timer 7 subtimer A        */
    TIMER7B_IRQHandler,                     /* Timer 7 subtimer B        */
    I2C6_IRQHandler,                        /* I2C6 Master and Slave     */
    I2C7_IRQHandler,                        /* I2C7 Master and Slave     */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    I2C8_IRQHandler,                        /* I2C8 Master and Slave     */
    I2C9_IRQHandler,                        /* I2C9 Master and Slave     */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0                                       /* Reserved                  */
};

void Default_Handler(void)
{
    return;
}

void Reset_Handler(void)
{
    //copy .data secition to SRAM
    uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t* pDst = (uint8_t*)&_sdata;
    uint8_t* pSrc = (uint8_t*)&_la_data;

    for(uint32_t i = 0; i < size; i++)
        *pDst++ = *pSrc++;

    //Init the .bss section to zero in SRAM
    size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    pDst = (uint8_t*)&_sbss;
    
    for(uint32_t i = 0; i < size; i++)
        *pDst++ = 0;

    //call main()
    main();
}