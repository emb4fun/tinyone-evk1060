/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File      : MIMXRT1062_Vectors.s
Purpose   : Exception and interrupt vectors for MIMXRT1062 devices.

Additional information:
  Preprocessor Definitions
    __NO_EXTERNAL_INTERRUPTS
      If defined,
        the vector table will contain only the internal exceptions
        and interrupts.
    __VECTORS_IN_RAM
      If defined,
        an area of RAM, large enough to store the vector table,
        will be reserved.

    __OPTIMIZATION_SMALL
      If defined,
        all weak definitions of interrupt handlers will share the
        same implementation.
      If not defined,
        all weak definitions of interrupt handlers will be defined
        with their own implementation.
*/
        .syntax unified

/*********************************************************************
*
*       Macros
*
**********************************************************************
*/

//
// Directly place a vector (word) in the vector table
//
.macro VECTOR Name=
        .section .vectors, "ax"
        .code 16
        .word \Name
.endm

//
// Declare an exception handler with a weak definition
//
.macro EXC_HANDLER Name=
        //
        // Insert vector in vector table
        //
        .section .vectors, "ax"
        .word \Name
        //
        // Insert dummy handler in init section
        //
        .section .init.\Name, "ax"
        .thumb_func
        .weak \Name
        .balign 2
\Name:
        1: b 1b   // Endless loop
.endm

//
// Declare an interrupt handler with a weak definition
//
.macro ISR_HANDLER Name=
        //
        // Insert vector in vector table
        //
        .section .vectors, "ax"
        .word \Name
        //
        // Insert dummy handler in init section
        //
#if defined(__OPTIMIZATION_SMALL)
        .section .init, "ax"
        .weak \Name
        .thumb_set \Name,Dummy_Handler
#else
        .section .init.\Name, "ax"
        .thumb_func
        .weak \Name
        .balign 2
\Name:
        1: b 1b   // Endless loop
#endif
.endm

//
// Place a reserved vector in vector table
//
.macro ISR_RESERVED
        .section .vectors, "ax"
        .word 0
.endm

//
// Place a reserved vector in vector table
//
.macro ISR_RESERVED_DUMMY
        .section .vectors, "ax"
        .word Dummy_Handler
.endm

/*********************************************************************
*
*       Externals
*
**********************************************************************
*/
        .extern __stack_end__
        .extern Reset_Handler
        .extern HardFault_Handler

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*  Setup of the vector table and weak definition of interrupt handlers
*
*/
        .section .vectors, "ax"
        .code 16
        .balign 512
        .global _vectors
_vectors:
        //
        // Internal exceptions and interrupts
        //
        VECTOR __stack_end__
        VECTOR Reset_Handler
        EXC_HANDLER NMI_Handler
        VECTOR HardFault_Handler
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        EXC_HANDLER SVC_Handler
        ISR_RESERVED
        ISR_RESERVED
        EXC_HANDLER PendSV_Handler
        EXC_HANDLER SysTick_Handler
        //
        // External interrupts
        //
#ifndef __NO_EXTERNAL_INTERRUPTS
        ISR_HANDLER DMA0_DMA16_IRQHandler
        ISR_HANDLER DMA1_DMA17_IRQHandler
        ISR_HANDLER DMA2_DMA18_IRQHandler
        ISR_HANDLER DMA3_DMA19_IRQHandler
        ISR_HANDLER DMA4_DMA20_IRQHandler
        ISR_HANDLER DMA5_DMA21_IRQHandler
        ISR_HANDLER DMA6_DMA22_IRQHandler
        ISR_HANDLER DMA7_DMA23_IRQHandler
        ISR_HANDLER DMA8_DMA24_IRQHandler
        ISR_HANDLER DMA9_DMA25_IRQHandler
        ISR_HANDLER DMA10_DMA26_IRQHandler
        ISR_HANDLER DMA11_DMA27_IRQHandler
        ISR_HANDLER DMA12_DMA28_IRQHandler
        ISR_HANDLER DMA13_DMA29_IRQHandler
        ISR_HANDLER DMA14_DMA30_IRQHandler
        ISR_HANDLER DMA15_DMA31_IRQHandler
        ISR_HANDLER DMA_ERROR_IRQHandler
        ISR_HANDLER CTI0_ERROR_IRQHandler
        ISR_HANDLER CTI1_ERROR_IRQHandler
        ISR_HANDLER CORE_IRQHandler
        ISR_HANDLER LPUART1_IRQHandler
        ISR_HANDLER LPUART2_IRQHandler
        ISR_HANDLER LPUART3_IRQHandler
        ISR_HANDLER LPUART4_IRQHandler
        ISR_HANDLER LPUART5_IRQHandler
        ISR_HANDLER LPUART6_IRQHandler
        ISR_HANDLER LPUART7_IRQHandler
        ISR_HANDLER LPUART8_IRQHandler
        ISR_HANDLER LPI2C1_IRQHandler
        ISR_HANDLER LPI2C2_IRQHandler
        ISR_HANDLER LPI2C3_IRQHandler
        ISR_HANDLER LPI2C4_IRQHandler
        ISR_HANDLER LPSPI1_IRQHandler
        ISR_HANDLER LPSPI2_IRQHandler
        ISR_HANDLER LPSPI3_IRQHandler
        ISR_HANDLER LPSPI4_IRQHandler
        ISR_HANDLER CAN1_IRQHandler
        ISR_HANDLER CAN2_IRQHandler
        ISR_HANDLER FLEXRAM_IRQHandler
        ISR_HANDLER KPP_IRQHandler
        ISR_HANDLER TSC_DIG_IRQHandler
        ISR_HANDLER GPR_IRQHandler
        ISR_HANDLER LCDIF_IRQHandler
        ISR_HANDLER CSI_IRQHandler
        ISR_HANDLER PXP_IRQHandler
        ISR_HANDLER WDOG2_IRQHandler
        ISR_HANDLER SNVS_HP_WRAPPER_IRQHandler
        ISR_HANDLER SNVS_HP_WRAPPER_TZ_IRQHandler
        ISR_HANDLER SNVS_LP_WRAPPER_IRQHandler
        ISR_HANDLER CSU_IRQHandler
        ISR_HANDLER DCP_IRQHandler
        ISR_HANDLER DCP_VMI_IRQHandler
        ISR_HANDLER Reserved68_IRQHandler
        ISR_HANDLER TRNG_IRQHandler
        ISR_HANDLER SJC_IRQHandler
        ISR_HANDLER BEE_IRQHandler
        ISR_HANDLER SAI1_IRQHandler
        ISR_HANDLER SAI2_IRQHandler
        ISR_HANDLER SAI3_RX_IRQHandler
        ISR_HANDLER SAI3_TX_IRQHandler
        ISR_HANDLER SPDIF_IRQHandler
        ISR_HANDLER PMU_EVENT_IRQHandler
        ISR_HANDLER Reserved78_IRQHandler
        ISR_HANDLER TEMP_LOW_HIGH_IRQHandler
        ISR_HANDLER TEMP_PANIC_IRQHandler
        ISR_HANDLER USB_PHY1_IRQHandler
        ISR_HANDLER USB_PHY2_IRQHandler
        ISR_HANDLER ADC1_IRQHandler
        ISR_HANDLER ADC2_IRQHandler
        ISR_HANDLER DCDC_IRQHandler
        ISR_HANDLER Reserved86_IRQHandler
        ISR_HANDLER Reserved87_IRQHandler
        ISR_HANDLER GPIO1_INT0_IRQHandler
        ISR_HANDLER GPIO1_INT1_IRQHandler
        ISR_HANDLER GPIO1_INT2_IRQHandler
        ISR_HANDLER GPIO1_INT3_IRQHandler
        ISR_HANDLER GPIO1_INT4_IRQHandler
        ISR_HANDLER GPIO1_INT5_IRQHandler
        ISR_HANDLER GPIO1_INT6_IRQHandler
        ISR_HANDLER GPIO1_INT7_IRQHandler
        ISR_HANDLER GPIO1_Combined_0_15_IRQHandler
        ISR_HANDLER GPIO1_Combined_16_31_IRQHandler
        ISR_HANDLER GPIO2_Combined_0_15_IRQHandler
        ISR_HANDLER GPIO2_Combined_16_31_IRQHandler
        ISR_HANDLER GPIO3_Combined_0_15_IRQHandler
        ISR_HANDLER GPIO3_Combined_16_31_IRQHandler
        ISR_HANDLER GPIO4_Combined_0_15_IRQHandler
        ISR_HANDLER GPIO4_Combined_16_31_IRQHandler
        ISR_HANDLER GPIO5_Combined_0_15_IRQHandler
        ISR_HANDLER GPIO5_Combined_16_31_IRQHandler
        ISR_HANDLER FLEXIO1_IRQHandler
        ISR_HANDLER FLEXIO2_IRQHandler
        ISR_HANDLER WDOG1_IRQHandler
        ISR_HANDLER RTWDOG_IRQHandler
        ISR_HANDLER EWM_IRQHandler
        ISR_HANDLER CCM_1_IRQHandler
        ISR_HANDLER CCM_2_IRQHandler
        ISR_HANDLER GPC_IRQHandler
        ISR_HANDLER SRC_IRQHandler
        ISR_HANDLER Reserved115_IRQHandler
        ISR_HANDLER GPT1_IRQHandler
        ISR_HANDLER GPT2_IRQHandler
        ISR_HANDLER PWM1_0_IRQHandler
        ISR_HANDLER PWM1_1_IRQHandler
        ISR_HANDLER PWM1_2_IRQHandler
        ISR_HANDLER PWM1_3_IRQHandler
        ISR_HANDLER PWM1_FAULT_IRQHandler
        ISR_HANDLER FLEXSPI2_IRQHandler
        ISR_HANDLER FLEXSPI_IRQHandler
        ISR_HANDLER SEMC_IRQHandler
        ISR_HANDLER USDHC1_IRQHandler
        ISR_HANDLER USDHC2_IRQHandler
        ISR_HANDLER USB_OTG2_IRQHandler
        ISR_HANDLER USB_OTG1_IRQHandler
        ISR_HANDLER ENET_IRQHandler
        ISR_HANDLER ENET_1588_Timer_IRQHandler
        ISR_HANDLER XBAR1_IRQ_0_1_IRQHandler
        ISR_HANDLER XBAR1_IRQ_2_3_IRQHandler
        ISR_HANDLER ADC_ETC_IRQ0_IRQHandler
        ISR_HANDLER ADC_ETC_IRQ1_IRQHandler
        ISR_HANDLER ADC_ETC_IRQ2_IRQHandler
        ISR_HANDLER ADC_ETC_ERROR_IRQHandler
        ISR_HANDLER PIT_IRQHandler
        ISR_HANDLER ACMP1_IRQHandler
        ISR_HANDLER ACMP2_IRQHandler
        ISR_HANDLER ACMP3_IRQHandler
        ISR_HANDLER ACMP4_IRQHandler
        ISR_HANDLER Reserved143_IRQHandler
        ISR_HANDLER Reserved144_IRQHandler
        ISR_HANDLER ENC1_IRQHandler
        ISR_HANDLER ENC2_IRQHandler
        ISR_HANDLER ENC3_IRQHandler
        ISR_HANDLER ENC4_IRQHandler
        ISR_HANDLER TMR1_IRQHandler
        ISR_HANDLER TMR2_IRQHandler
        ISR_HANDLER TMR3_IRQHandler
        ISR_HANDLER TMR4_IRQHandler
        ISR_HANDLER PWM2_0_IRQHandler
        ISR_HANDLER PWM2_1_IRQHandler
        ISR_HANDLER PWM2_2_IRQHandler
        ISR_HANDLER PWM2_3_IRQHandler
        ISR_HANDLER PWM2_FAULT_IRQHandler
        ISR_HANDLER PWM3_0_IRQHandler
        ISR_HANDLER PWM3_1_IRQHandler
        ISR_HANDLER PWM3_2_IRQHandler
        ISR_HANDLER PWM3_3_IRQHandler
        ISR_HANDLER PWM3_FAULT_IRQHandler
        ISR_HANDLER PWM4_0_IRQHandler
        ISR_HANDLER PWM4_1_IRQHandler
        ISR_HANDLER PWM4_2_IRQHandler
        ISR_HANDLER PWM4_3_IRQHandler
        ISR_HANDLER PWM4_FAULT_IRQHandler
        ISR_HANDLER ENET2_IRQHandler
        ISR_HANDLER ENET2_1588_Timer_IRQHandler
        ISR_HANDLER CAN3_IRQHandler
        ISR_HANDLER Reserved171_IRQHandler
        ISR_HANDLER FLEXIO3_IRQHandler
        ISR_HANDLER GPIO6_7_8_9_IRQHandler
#endif
        //
        .section .vectors, "ax"
_vectors_end:

#ifdef __VECTORS_IN_RAM
        //
        // Reserve space with the size of the vector table
        // in the designated RAM section.
        //
        .section .vectors_ram, "ax"
        .balign 512
        .global _vectors_ram

_vectors_ram:
        .space _vectors_end - _vectors, 0
#endif

/*********************************************************************
*
*  Dummy handler to be used for reserved interrupt vectors
*  and weak implementation of interrupts.
*
*/
        .section .init.Dummy_Handler, "ax"
        .thumb_func
        .weak Dummy_Handler
        .balign 2
Dummy_Handler:
        1: b 1b   // Endless loop


/*************************** End of file ****************************/
