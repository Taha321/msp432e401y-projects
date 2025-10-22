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

extern void reset_handler(void);
void default_handler                (void) __attribute__((weak));

/* Cortex-M4 Processor Exceptions */
 void nmi_handler             (void) __attribute__((weak, alias("default_handler")));
 void hardFault_handler       (void) __attribute__((weak, alias("default_handler")));
 void mem_manage_handler       (void) __attribute__((weak, alias("default_handler")));
 void busFault_handler        (void) __attribute__((weak, alias("default_handler")));
 void usageFault_handler      (void) __attribute__((weak, alias("default_handler")));
 void svc_handler             (void) __attribute__((weak, alias("default_handler")));
 void debugMon_handler        (void) __attribute__((weak, alias("default_handler")));
 void pendSV_handler          (void) __attribute__((weak, alias("default_handler")));
 void sysTick_handler         (void) __attribute__((weak, alias("default_handler")));

/* device specific interrupt handler */
 void gpio_a_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_b_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_c_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_d_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_e_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart0_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart1_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void ssi0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void pwm0_fault_irq_handler   (void) __attribute__((weak, alias("default_handler")));
 void pwm0_0_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void pwm0_1_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void pwm0_2_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void qei0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void adc0_ss0_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc0_ss1_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc0_ss2_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc0_ss3_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void watchdog_irq_handler     (void) __attribute__((weak, alias("default_handler")));
 void timer0_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer0_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer1_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer1_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer2_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer2_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void comp0_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void comp1_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void comp2_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void sysctl_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void flash_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_f_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_g_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_h_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart2_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void ssi1_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void timer3_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer3_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void i2c1_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void can0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void can1_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void emac0_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void hibernate_irq_handler    (void) __attribute__((weak, alias("default_handler")));
 void usb0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void pwm0_3_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void udma_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void udma_err_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc1_ss0_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc1_ss1_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc1_ss2_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void adc1_ss3_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void epi0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void gpio_j_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_k_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_l_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void ssi2_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void ssi3_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void uart3_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart4_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart5_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart6_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void uart7_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void i2c2_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c3_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void timer4_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer4_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer5_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer5_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void sysexc_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void i2c4_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c5_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void gpio_m_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void gpio_n_irq_handler        (void) __attribute__((weak, alias("default_handler")));
 void tamper0_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_0_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_1_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_2_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_3_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_4_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_5_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_6_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_p_7_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_0_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_1_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_2_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_3_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_4_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_5_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_6_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void gpio_q_7_irq_handler       (void) __attribute__((weak, alias("default_handler")));
 void sha0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void aes0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void des0_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void timer6_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer6_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer7_a_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void timer7_b_irq_handler      (void) __attribute__((weak, alias("default_handler")));
 void i2c6_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c7_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c8_irq_handler         (void) __attribute__((weak, alias("default_handler")));
 void i2c9_irq_handler         (void) __attribute__((weak, alias("default_handler")));


/* The vector table.  Note that the proper constructs must be placed on this to */
/* ensure that it ends up at physical address 0x0000.0000 or at the start of    */
/* the program if located at a start address other than 0.                      */

void (* const interruptVectors[])(void) __attribute__ ((section (".isr_vector"))) =
{
    (void (* const)(void))STACK_START,
    reset_handler,                          /* The reset handler         */
    nmi_handler,                            /* The NMI handler           */
    hardFault_handler,                      /* The hard fault handler    */
    mem_manage_handler,                      /* The MPU fault handler     */
    busFault_handler,                       /* The bus fault handler     */
    usageFault_handler,                     /* The usage fault handler   */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    svc_handler,                            /* SVCall handler            */
    debugMon_handler,                       /* Debug monitor handler     */
    0,                                      /* Reserved                  */
    pendSV_handler,                         /* The PendSV handler        */
    sysTick_handler,                        /* The SysTick handler       */
    gpio_a_irq_handler,                       /* GPIO Port A               */
    gpio_b_irq_handler,                       /* GPIO Port B               */
    gpio_c_irq_handler,                       /* GPIO Port C               */
    gpio_d_irq_handler,                       /* GPIO Port D               */
    gpio_e_irq_handler,                       /* GPIO Port E               */
    uart0_irq_handler,                       /* UART0 Rx and Tx           */
    uart1_irq_handler,                       /* UART1 Rx and Tx           */
    ssi0_irq_handler,                        /* SSI0 Rx and Tx            */
    i2c0_irq_handler,                        /* I2C0 Master and Slave     */
    pwm0_fault_irq_handler,                  /* PWM Fault                 */
    pwm0_0_irq_handler,                      /* PWM Generator 0           */
    pwm0_1_irq_handler,                      /* PWM Generator 1           */
    pwm0_2_irq_handler,                      /* PWM Generator 2           */
    qei0_irq_handler,                        /* Quadrature Encoder 0      */
    adc0_ss0_irq_handler,                     /* ADC Sequence 0            */
    adc0_ss1_irq_handler,                     /* ADC Sequence 1            */
    adc0_ss2_irq_handler,                     /* ADC Sequence 2            */
    adc0_ss3_irq_handler,                     /* ADC Sequence 3            */
    watchdog_irq_handler,                    /* Watchdog timer            */
    timer0_a_irq_handler,                     /* Timer 0 subtimer A        */
    timer0_b_irq_handler,                     /* Timer 0 subtimer B        */
    timer1_a_irq_handler,                     /* Timer 1 subtimer A        */
    timer1_b_irq_handler,                     /* Timer 1 subtimer B        */
    timer2_a_irq_handler,                     /* Timer 2 subtimer A        */
    timer2_b_irq_handler,                     /* Timer 2 subtimer B        */
    comp0_irq_handler,                       /* Analog Comparator 0       */
    comp1_irq_handler,                       /* Analog Comparator 1       */
    comp2_irq_handler,                       /* Analog Comparator 2       */
    sysctl_irq_handler,                      /* System Control            */
    flash_irq_handler,                       /* FLASH Control             */
    gpio_f_irq_handler,                       /* GPIO Port F               */
    gpio_g_irq_handler,                       /* GPIO Port G               */
    gpio_h_irq_handler,                       /* GPIO Port H               */
    uart2_irq_handler,                       /* UART2 Rx and Tx           */
    ssi1_irq_handler,                        /* SSI1 Rx and Tx            */
    timer3_a_irq_handler,                     /* Timer 3 subtimer A        */
    timer3_b_irq_handler,                     /* Timer 3 subtimer B        */
    i2c1_irq_handler,                        /* I2C1 Master and Slave     */
    can0_irq_handler,                        /* CAN0                      */
    can1_irq_handler,                        /* CAN1                      */
    emac0_irq_handler,                       /* Ethernet                  */
    hibernate_irq_handler,                   /* Hibernate                 */
    usb0_irq_handler,                        /* USB0                      */
    pwm0_3_irq_handler,                      /* PWM Generator 3           */
    udma_irq_handler,                        /* uDMA Software Transfer    */
    udma_err_irq_handler,                     /* uDMA Error                */
    adc1_ss0_irq_handler,                     /* ADC1 Sequence 0           */
    adc1_ss1_irq_handler,                     /* ADC1 Sequence 1           */
    adc1_ss2_irq_handler,                     /* ADC1 Sequence 2           */
    adc1_ss3_irq_handler,                     /* ADC1 Sequence 3           */
    epi0_irq_handler,                        /* External Bus Interface 0  */
    gpio_j_irq_handler,                       /* GPIO Port J               */
    gpio_k_irq_handler,                       /* GPIO Port K               */
    gpio_l_irq_handler,                       /* GPIO Port L               */
    ssi2_irq_handler,                        /* SSI2 Rx and Tx            */
    ssi3_irq_handler,                        /* SSI3 Rx and Tx            */
    uart3_irq_handler,                       /* UART3 Rx and Tx           */
    uart4_irq_handler,                       /* UART4 Rx and Tx           */
    uart5_irq_handler,                       /* UART5 Rx and Tx           */
    uart6_irq_handler,                       /* UART6 Rx and Tx           */
    uart7_irq_handler,                       /* UART7 Rx and Tx           */
    i2c2_irq_handler,                        /* I2C2 Master and Slave     */
    i2c3_irq_handler,                        /* I2C3 Master and Slave     */
    timer4_a_irq_handler,                     /* Timer 4 subtimer A        */
    timer4_b_irq_handler,                     /* Timer 4 subtimer B        */
    timer5_a_irq_handler,                     /* Timer 5 subtimer A        */
    timer5_b_irq_handler,                     /* Timer 5 subtimer B        */
    sysexc_irq_handler,                      /* FPU                       */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    i2c4_irq_handler,                        /* I2C4 Master and Slave     */
    i2c5_irq_handler,                        /* I2C5 Master and Slave     */
    gpio_m_irq_handler,                       /* GPIO Port M               */
    gpio_n_irq_handler,                       /* GPIO Port N               */
    0,                                      /* Reserved                  */
    tamper0_irq_handler,                     /* Tamper                    */
    gpio_p_0_irq_handler,                      /* GPIO Port P(Summary or P0)*/
    gpio_p_1_irq_handler,                      /* GPIO Port P1              */
    gpio_p_2_irq_handler,                      /* GPIO Port P2              */
    gpio_p_3_irq_handler,                      /* GPIO Port P3              */
    gpio_p_4_irq_handler,                      /* GPIO Port P4              */
    gpio_p_5_irq_handler,                      /* GPIO Port P5              */
    gpio_p_6_irq_handler,                      /* GPIO Port P6              */
    gpio_p_7_irq_handler,                      /* GPIO Port P7              */
    gpio_q_0_irq_handler,                      /* GPIO Port Q(Summary or Q0)*/
    gpio_q_1_irq_handler,                      /* GPIO Port Q1              */
    gpio_q_2_irq_handler,                      /* GPIO Port Q2              */
    gpio_q_3_irq_handler,                      /* GPIO Port Q3              */
    gpio_q_4_irq_handler,                      /* GPIO Port Q4              */
    gpio_q_5_irq_handler,                      /* GPIO Port Q5              */
    gpio_q_6_irq_handler,                      /* GPIO Port Q6              */
    gpio_q_7_irq_handler,                      /* GPIO Port Q7              */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    sha0_irq_handler,                        /* SHA/MD5 0                 */
    aes0_irq_handler,                        /* AES 0                     */
    des0_irq_handler,                        /* DES3DES 0                 */
    0,                                      /* Reserved                  */
    timer6_a_irq_handler,                     /* Timer 6 subtimer A        */
    timer6_b_irq_handler,                     /* Timer 6 subtimer B        */
    timer7_a_irq_handler,                     /* Timer 7 subtimer A        */
    timer7_b_irq_handler,                     /* Timer 7 subtimer B        */
    i2c6_irq_handler,                        /* I2C6 Master and Slave     */
    i2c7_irq_handler,                        /* I2C7 Master and Slave     */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    i2c8_irq_handler,                        /* I2C8 Master and Slave     */
    i2c9_irq_handler,                        /* I2C9 Master and Slave     */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0                                       /* Reserved                  */
};

void default_handler(void)
{
    return;
}

void reset_handler(void)
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