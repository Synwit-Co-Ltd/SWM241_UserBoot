;******************************************************************************************************************************************
; �ļ�����:	startup_SWM241.s
; ����˵��:	SWM241��Ƭ���������ļ�
; ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
; ע������:
; �汾����: V1.0.0		2016��1��30��
; ������¼:
;
;
;******************************************************************************************************************************************
; @attention
;
; THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION
; REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE
; FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
; OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
; -ECTION WITH THEIR PRODUCTS.
;
; COPYRIGHT 2012 Synwit Technology
;******************************************************************************************************************************************


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size	  EQU	 0x400;

				AREA	STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem	   SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size	   EQU	 0x000;

				AREA	HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem		SPACE   Heap_Size
__heap_limit


				PRESERVE8
				THUMB

; Vector Table Mapped to Address 0 at Reset

				AREA	RESET, DATA, READONLY
				EXPORT  __Vectors
				EXPORT  __Vectors_End
				EXPORT  __Vectors_Size

__Vectors	    DCD	 Stack_Mem + Stack_Size	; Top of Stack
				DCD	 Reset_Handler			 ; Reset Handler
				DCD	 NMI_Handler			   ; NMI Handler
				DCD	 HardFault_Handler		 ; Hard Fault Handler
				DCD	 0
				DCD	 0
				DCD	 0
				DCD	 0
				DCD	 0
				DCD	 0
				DCD	 0
				DCD	 SVC_Handler			   ; SVCall Handler
				DCD	 0
				DCD	 0
				DCD	 PendSV_Handler			; PendSV Handler
				DCD	 SysTick_Handler		   ; SysTick Handler

				; External Interrupts
				DCD    UART0_Handler
				DCD    TIMR0_Handler
				DCD    SPI0_Handler
				DCD    UART1_Handler
				DCD    UART2_Handler
				DCD    TIMR1_Handler
				DCD    DMA_Handler
				DCD    PWM0_Handler
				DCD    I2C1_Handler
				DCD    TIMR2_Handler
				DCD    TIMR3_Handler
				DCD    WDT_Handler
				DCD    I2C0_Handler
				DCD    UART3_Handler
				DCD    ADC_Handler
				DCD    TIMR4_Handler
				DCD    GPIOC0_GPIOD1_Handler
				DCD    GPIOB1_GPIOC2_Handler
				DCD    TIMR5_GPIOC3_Handler
				DCD    GPIOA0_GPIOD6_Handler
				DCD    TIMR6_GPIOC1_Handler
				DCD    GPIOA1_GPIOD8_Handler
				DCD    GPIOB7_GPIOD9_Handler
				DCD    GPIOB5_GPIOD10_Handler
				DCD    GPIOA2_GPIOB2_GPIOD13_Handler
				DCD    TIMR7_XTALSTOP_GPIOD12_Handler
				DCD    PWM1_GPIOA_Handler
				DCD    PWM2_GPIOB_Handler
				DCD    BOD_PWMBRK_GPIOD11_Handler
				DCD    PWM3_GPIOC_SAFETY_Handler
				DCD    SPI1_HALL_GPIOA9_Handler
				DCD    RTC_GPIOD_Handler

__Vectors_End

__Vectors_Size 	EQU 	__Vectors_End - __Vectors



				AREA	|.text|, CODE, READONLY

; Reset Handler

Reset_Handler   PROC
				EXPORT  Reset_Handler			[WEAK]
				IMPORT  __main

				LDR	 R0, =__main
				BX	  R0
				ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler	 PROC
				EXPORT  NMI_Handler				[WEAK]
				B	   .
				ENDP

HardFault_Handler	PROC
				EXPORT  HardFault_Handler		[WEAK]
				B	   .
				ENDP

SVC_Handler	 PROC
				EXPORT  SVC_Handler			    [WEAK]
				B	   .
				ENDP

PendSV_Handler  PROC
				EXPORT  PendSV_Handler			[WEAK]
				B	   .
				ENDP

SysTick_Handler PROC
				EXPORT  SysTick_Handler		   	[WEAK]
				B	   .
				ENDP

UART0_Handler  PROC
				EXPORT  UART0_Handler                   [WEAK]
				B       .
				ENDP

TIMR0_Handler  PROC
				EXPORT  TIMR0_Handler                   [WEAK]
				B       .
				ENDP

SPI0_Handler  PROC
				EXPORT  SPI0_Handler                    [WEAK]
				B       .
				ENDP

UART1_Handler  PROC
				EXPORT  UART1_Handler                   [WEAK]
				B       .
				ENDP

UART2_Handler  PROC
				EXPORT  UART2_Handler                   [WEAK]
				B       .
				ENDP

TIMR1_Handler  PROC
				EXPORT  TIMR1_Handler                   [WEAK]
				B       .
				ENDP

DMA_Handler  PROC
				EXPORT  DMA_Handler                     [WEAK]
				B       .
				ENDP

PWM0_Handler  PROC
				EXPORT  PWM0_Handler                    [WEAK]
				B       .
				ENDP

I2C1_Handler  PROC
				EXPORT  I2C1_Handler                    [WEAK]
				B       .
				ENDP

TIMR2_Handler  PROC
				EXPORT  TIMR2_Handler                   [WEAK]
				B       .
				ENDP

TIMR3_Handler  PROC
				EXPORT  TIMR3_Handler                   [WEAK]
				B       .
				ENDP

WDT_Handler  PROC
				EXPORT  WDT_Handler                     [WEAK]
				B       .
				ENDP

I2C0_Handler  PROC
				EXPORT  I2C0_Handler                    [WEAK]
				B       .
				ENDP

UART3_Handler  PROC
				EXPORT  UART3_Handler                   [WEAK]
				B       .
				ENDP

ADC_Handler  PROC
				EXPORT  ADC_Handler                     [WEAK]
				B       .
				ENDP

TIMR4_Handler  PROC
				EXPORT  TIMR4_Handler                   [WEAK]
				B       .
				ENDP

GPIOC0_GPIOD1_Handler  PROC
				EXPORT  GPIOC0_GPIOD1_Handler           [WEAK]
				B       .
				ENDP

GPIOB1_GPIOC2_Handler  PROC
				EXPORT  GPIOB1_GPIOC2_Handler           [WEAK]
				B       .
				ENDP

TIMR5_GPIOC3_Handler  PROC
				EXPORT  TIMR5_GPIOC3_Handler            [WEAK]
				B       .
				ENDP

GPIOA0_GPIOD6_Handler  PROC
				EXPORT  GPIOA0_GPIOD6_Handler           [WEAK]
				B       .
				ENDP

TIMR6_GPIOC1_Handler  PROC
				EXPORT  TIMR6_GPIOC1_Handler            [WEAK]
				B       .
				ENDP

GPIOA1_GPIOD8_Handler  PROC
				EXPORT  GPIOA1_GPIOD8_Handler           [WEAK]
				B       .
				ENDP

GPIOB7_GPIOD9_Handler  PROC
				EXPORT  GPIOB7_GPIOD9_Handler           [WEAK]
				B       .
				ENDP

GPIOB5_GPIOD10_Handler  PROC
				EXPORT  GPIOB5_GPIOD10_Handler          [WEAK]
				B       .
				ENDP

GPIOA2_GPIOB2_GPIOD13_Handler  PROC
				EXPORT  GPIOA2_GPIOB2_GPIOD13_Handler   [WEAK]
				B       .
				ENDP

TIMR7_XTALSTOP_GPIOD12_Handler  PROC
				EXPORT  TIMR7_XTALSTOP_GPIOD12_Handler  [WEAK]
				B       .
				ENDP

PWM1_GPIOA_Handler  PROC
				EXPORT  PWM1_GPIOA_Handler              [WEAK]
				B       .
				ENDP

PWM2_GPIOB_Handler  PROC
				EXPORT  PWM2_GPIOB_Handler              [WEAK]
				B       .
				ENDP

BOD_PWMBRK_GPIOD11_Handler  PROC
				EXPORT  BOD_PWMBRK_GPIOD11_Handler      [WEAK]
				B       .
				ENDP

PWM3_GPIOC_SAFETY_Handler  PROC
				EXPORT  PWM3_GPIOC_SAFETY_Handler       [WEAK]
				B       .
				ENDP

SPI1_HALL_GPIOA9_Handler  PROC
				EXPORT  SPI1_HALL_GPIOA9_Handler        [WEAK]
				B       .
				ENDP

RTC_GPIOD_Handler  PROC
				EXPORT  RTC_GPIOD_Handler               [WEAK]
				B       .
				ENDP

				ALIGN


; User Initial Stack & Heap

				IF	  :DEF:__MICROLIB

				EXPORT  __initial_sp
				EXPORT  __heap_base
				EXPORT  __heap_limit

				ELSE

				IMPORT  __use_two_region_memory
				EXPORT  __user_initial_stackheap
__user_initial_stackheap

				LDR	 R0, =  Heap_Mem
				LDR	 R1, =(Stack_Mem + Stack_Size)
				LDR	 R2, = (Heap_Mem +  Heap_Size)
				LDR	 R3, = Stack_Mem
				BX	 LR

				ALIGN

				ENDIF


				END
