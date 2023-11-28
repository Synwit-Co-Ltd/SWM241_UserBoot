#ifndef __SWM241_H__
#define __SWM241_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers **********************************************/
  NonINTMableInt_IRQn = -14,	/*!< 2 Non INTMable Interrupt								 */
  HardFault_IRQn	  = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt						 */
  SVCall_IRQn		  = -5,	 /*!< 11 Cortex-M0 SV Call Interrupt						     */
  PendSV_IRQn		  = -2,	 /*!< 14 Cortex-M0 Pend SV Interrupt						     */
  SysTick_IRQn		  = -1,	 /*!< 15 Cortex-M0 System Tick Interrupt					     */

/******  Cortex-M0 specific Interrupt Numbers ************************************************/
  UART0_IRQn                  = 0,
  TIMR0_IRQn                  = 1,
  SPI0_IRQn                   = 2,
  UART1_IRQn                  = 3,
  UART2_IRQn                  = 4,
  TIMR1_IRQn                  = 5,
  DMA_IRQn                    = 6,
  PWM0_IRQn                   = 7,
  I2C1_IRQn                   = 8,
  TIMR2_IRQn                  = 9,
  TIMR3_IRQn                  = 10,
  WDT_IRQn                    = 11,
  I2C0_IRQn                   = 12,
  UART3_IRQn                  = 13,
  ADC_IRQn                    = 14,
  TIMR4_IRQn                  = 15,
  GPIOC0_GPIOD1_IRQn          = 16,
  GPIOB1_GPIOC2_IRQn          = 17,
  TIMR5_GPIOC3_IRQn           = 18,
  GPIOA0_GPIOD6_IRQn          = 19,
  TIMR6_GPIOC1_IRQn           = 20,
  GPIOA1_GPIOD8_IRQn          = 21,
  GPIOB7_GPIOD9_IRQn          = 22,
  GPIOB5_GPIOD10_IRQn         = 23,
  GPIOA2_GPIOB2_GPIOD13_IRQn  = 24,
  TIMR7_XTALSTOP_GPIOD12_IRQn = 25,
  PWM1_GPIOA_IRQn             = 26,
  PWM2_GPIOB_IRQn             = 27,
  BOD_PWMBRK_GPIOD11_IRQn     = 28,
  PWM3_GPIOC_SAFETY_IRQn      = 29,
  SPI1_HALL_GPIOA9_IRQn       = 30,
  RTC_GPIOD_IRQn              = 31
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT		    0	   /*!< UART does not provide a MPU present or not	     */
#define __NVIC_PRIO_BITS		2	   /*!< UART Supports 2 Bits for the Priority Levels	 */
#define __Vendor_SysTickConfig  0	   /*!< Set to 1 if different SysTick Config is used	 */

#if   defined ( __CC_ARM )
  #pragma anon_unions
#endif

#include <stdio.h>
#include "core_cm0.h"				   /* Cortex-M0 processor and core peripherals		     */
#include "system_SWM241.h"


/******************************************************************************/
/*				Device Specific Peripheral registers structures			 */
/******************************************************************************/
typedef struct {
	__IO uint32_t CLKSEL;				    //Clock Select

	__IO uint32_t CLKDIVx_ON;				//[0] CLK_DIVxʱ��Դ����

	__IO uint32_t CLKEN0;					//Clock Enable
	
	__IO uint32_t CLKEN1;

	__IO uint32_t SLEEP;
	
	uint32_t RESERVED[4];
	
	__IO uint32_t RSTSR;					//Reset Status
	
	uint32_t RESERVED2[2];
	
	__IO uint32_t RTCWKCR;					//RTC Wakeup Control
	
	__IO uint32_t RTCWKSR;
	
	uint32_t RESERVED3[2];
	
	__IO uint32_t SYSIE;
	
	__IO uint32_t SYSIF;
	
	__IO uint32_t RAMERR;
	
	uint32_t RESERVED4[13];
	
	__I  uint32_t CHIPID[4];
	
	__IO uint32_t BACKUP[4];
	
	 uint32_t RESERVED5[12];

	__IO uint32_t PRNGCR;
	__IO uint32_t PRNGDL;
	__IO uint32_t PRNGDH;

	uint32_t RESERVED6[9];
		 
	__IO uint32_t PAWKEN;				    //PORTA Wakeup Enable
	__IO uint32_t PBWKEN;
	__IO uint32_t PCWKEN;
	__IO uint32_t PDWKEN;
	
    uint32_t RESERVED7[8];

	__IO uint32_t PAWKSR;				    //PORTA Wakeup Status��д1����
	__IO uint32_t PBWKSR;
	__IO uint32_t PCWKSR;
	__IO uint32_t PDWKSR;
	
	uint32_t RESERVED8[(0x720-0x13C)/4-1];
	
	__IO uint32_t PRSTEN;					//���踴λʹ�ܣ�ֻ�е�PRSTEN��ֵΪ0x55ʱ������дPRSTR1��PRSTR2
	__IO uint32_t PRSTR1;
	__IO uint32_t PRSTR2;

    //Analog Control: 0x400AA000
    uint32_t RESERVED9[(0x400AA000-0x40000728)/4-1];
	
	__IO uint32_t HRCCR;					//High speed RC Control Register
	uint32_t RESERVED10[3];
    
    __IO uint32_t BODCR;
	__IO uint32_t BODSR;
	
	uint32_t RESERVED11[2];
	
	__IO uint32_t XTALCR;
	__IO uint32_t XTALSR;
	
	uint32_t RESERVED12[10];
	
    __IO uint32_t LRCCR;					//Low speed RC Control Register
} SYS_TypeDef;


#define SYS_CLKSEL_SYS_Pos			0		//ϵͳʱ��ѡ��	1 HRC	0 CLK_DIVx
#define SYS_CLKSEL_SYS_Msk			(0x01 << SYS_CLKSEL_SYS_Pos)
#define SYS_CLKSEL_CLK_DIVx_Pos		1		//ѡ��CLK_DIVx  0 CLK_DIV1   1 CLK_DIV8
#define SYS_CLKSEL_CLK_DIVx_Msk		(0x01 << SYS_CLKSEL_CLK_DIVx_Pos)
#define SYS_CLKSEL_CLK_Pos			2		//Clock Source	0 LRC   2 XTAL_32K   3 XTAL   4 HRC
#define SYS_CLKSEL_CLK_Msk			(0x07 << SYS_CLKSEL_CLK_Pos)
#define SYS_CLKSEL_RTC_Pos			5		//RTCʱ��Դѡ��  0 LRC   1 XTAL_32K
#define SYS_CLKSEL_RTC_Msk			(0x01 << SYS_CLKSEL_RTC_Pos)
#define SYS_CLKSEL_WDT_Pos			12		//���Ź�ʱ��ѡ��  0 HRC   1 XTAL   2 LRC   3 XTAL_32K
#define SYS_CLKSEL_WDT_Msk			(0x03 << SYS_CLKSEL_WDT_Pos)
#define SYS_CLKSEL_RTCTRIM_Pos		14		//RTC Trim�ο�ʱ��  0 XTAL   1 XTAL/2   2 XTAL/4   3 XTAL/8
#define SYS_CLKSEL_RTCTRIM_Msk		(0x03 << SYS_CLKSEL_RTCTRIM_Pos)
#define SYS_CLKSEL_ADC_Pos			16		//ADCʱ��ѡ��  0 HRC   1 XTAL   2 PLL   3 PLL
#define SYS_CLKSEL_ADC_Msk			(0x03 << SYS_CLKSEL_ADC_Pos)
#define SYS_CLKSEL_ADCDIV_Pos		18		//ADCʱ�ӷ�Ƶ  0 1��Ƶ   1 1��Ƶ   2 4��Ƶ   3 8��Ƶ
#define SYS_CLKSEL_ADCDIV_Msk		(0x03 << SYS_CLKSEL_ADCDIV_Pos)
#define SYS_CLKSEL_WKUP_Pos			24
#define SYS_CLKSEL_WKUP_Msk			(0x01 << SYS_CLKSEL_WKUP_Pos)

#define SYS_CLKDIV_ON_Pos           0
#define SYS_CLKDIV_ON_Msk           (0x01 << SYS_CLKDIV_ON_Pos)

#define SYS_CLKEN0_GPIOA_Pos		0
#define SYS_CLKEN0_GPIOA_Msk		(0x01 << SYS_CLKEN0_GPIOA_Pos)
#define SYS_CLKEN0_GPIOB_Pos		1
#define SYS_CLKEN0_GPIOB_Msk		(0x01 << SYS_CLKEN0_GPIOB_Pos)
#define SYS_CLKEN0_GPIOC_Pos		2
#define SYS_CLKEN0_GPIOC_Msk		(0x01 << SYS_CLKEN0_GPIOC_Pos)
#define SYS_CLKEN0_GPIOD_Pos		3
#define SYS_CLKEN0_GPIOD_Msk		(0x01 << SYS_CLKEN0_GPIOD_Pos)
#define SYS_CLKEN0_UART0_Pos		6
#define SYS_CLKEN0_UART0_Msk		(0x01 << SYS_CLKEN0_UART0_Pos)
#define SYS_CLKEN0_UART1_Pos		7
#define SYS_CLKEN0_UART1_Msk		(0x01 << SYS_CLKEN0_UART1_Pos)
#define SYS_CLKEN0_UART2_Pos		8
#define SYS_CLKEN0_UART2_Msk		(0x01 << SYS_CLKEN0_UART2_Pos)
#define SYS_CLKEN0_UART3_Pos		9
#define SYS_CLKEN0_UART3_Msk		(0x01 << SYS_CLKEN0_UART3_Pos)
#define SYS_CLKEN0_WDT_Pos			10
#define SYS_CLKEN0_WDT_Msk			(0x01 << SYS_CLKEN0_WDT_Pos)
#define SYS_CLKEN0_TIMR_Pos			11
#define SYS_CLKEN0_TIMR_Msk			(0x01 << SYS_CLKEN0_TIMR_Pos)
#define SYS_CLKEN0_PWM_Pos			12
#define SYS_CLKEN0_PWM_Msk			(0x01 << SYS_CLKEN0_PWM_Pos)
#define SYS_CLKEN0_SPI0_Pos			13
#define SYS_CLKEN0_SPI0_Msk			(0x01 << SYS_CLKEN0_SPI0_Pos)
#define SYS_CLKEN0_SPI1_Pos			14
#define SYS_CLKEN0_SPI1_Msk			(0x01 << SYS_CLKEN0_SPI1_Pos)
#define SYS_CLKEN0_I2C0_Pos			15
#define SYS_CLKEN0_I2C0_Msk			(0x01 << SYS_CLKEN0_I2C0_Pos)
#define SYS_CLKEN0_I2C1_Pos			16
#define SYS_CLKEN0_I2C1_Msk			(0x01 << SYS_CLKEN0_I2C1_Pos)
#define SYS_CLKEN0_CRC_Pos			19
#define SYS_CLKEN0_CRC_Msk			(0x01 << SYS_CLKEN0_CRC_Pos)
#define SYS_CLKEN0_DIV_Pos			21
#define SYS_CLKEN0_DIV_Msk			(0x01 << SYS_CLKEN0_DIV_Pos)
#define SYS_CLKEN0_ANAC_Pos			25		//ģ����Ƶ�Ԫʱ��ʹ��
#define SYS_CLKEN0_ANAC_Msk			(0x01 << SYS_CLKEN0_ANAC_Pos)
#define SYS_CLKEN0_ADC_Pos			26
#define SYS_CLKEN0_ADC_Msk			(0x01 << SYS_CLKEN0_ADC_Pos)
#define SYS_CLKEN0_CAN_Pos			28
#define SYS_CLKEN0_CAN_Msk			(0x01 << SYS_CLKEN0_CAN_Pos)
#define SYS_CLKEN0_SLCD_Pos			29
#define SYS_CLKEN0_SLCD_Msk			(0x01 << SYS_CLKEN0_SLCD_Pos)
#define SYS_CLKEN0_SLED_Pos			31
#define SYS_CLKEN0_SLED_Msk			(0x01u<< SYS_CLKEN0_SLED_Pos)
		
#define SYS_CLKEN1_RTC_Pos			19
#define SYS_CLKEN1_RTC_Msk			(0x01 << SYS_CLKEN1_RTC_Pos)

#define SYS_SLEEP_SLEEP_Pos			0		//����λ��1��ϵͳ������SLEEPģʽ
#define SYS_SLEEP_SLEEP_Msk			(0x01 << SYS_SLEEP_SLEEP_Pos)
#define SYS_SLEEP_STOP_Pos			1		//����λ��1��ϵͳ������STOP SLEEPģʽ
#define SYS_SLEEP_STOP_Msk			(0x01 << SYS_SLEEP_STOP_Pos)

#define SYS_RSTSR_POR_Pos			0		//1 ���ֹ�POR��λ��д1����
#define SYS_RSTSR_POR_Msk			(0x01 << SYS_RSTSR_POR_Pos)
#define SYS_RSTSR_WDT_Pos			1		//1 ���ֹ�WDT��λ��д1����
#define SYS_RSTSR_WDT_Msk			(0x01 << SYS_RSTSR_WDT_Pos)
#define SYS_RSTSR_IAA_Pos			4		//Illegal Address Access
#define SYS_RSTSR_IAA_Msk			(0x01 << SYS_RSTSR_IAA_Pos)

#define SYS_RTCWKCR_EN_Pos			0		//RTC����ʹ��
#define SYS_RTCWKCR_EN_Msk			(0x01 << SYS_RTCWKCR_EN_Pos)

#define SYS_RTCWKSR_FLAG_Pos		0		//RTC���ѱ�־��д1����
#define SYS_RTCWKSR_FLAG_Msk		(0x01 << SYS_RTCWKSR_FLAG_Pos)

#define SYS_PRSTR1_GPIOA_Pos		0		//1 ��λGPIOA    0 ����λ
#define SYS_PRSTR1_GPIOA_Msk		(0x01 << SYS_PRSTR1_GPIOA_Pos)
#define SYS_PRSTR1_GPIOB_Pos		1
#define SYS_PRSTR1_GPIOB_Msk		(0x01 << SYS_PRSTR1_GPIOB_Pos)
#define SYS_PRSTR1_GPIOC_Pos		2
#define SYS_PRSTR1_GPIOC_Msk		(0x01 << SYS_PRSTR1_GPIOC_Pos)
#define SYS_PRSTR1_GPIOD_Pos		3
#define SYS_PRSTR1_GPIOD_Msk		(0x01 << SYS_PRSTR1_GPIOD_Pos)
#define SYS_PRSTR1_UART0_Pos		6
#define SYS_PRSTR1_UART0_Msk		(0x01 << SYS_PRSTR1_UART0_Pos)
#define SYS_PRSTR1_UART1_Pos		7
#define SYS_PRSTR1_UART1_Msk		(0x01 << SYS_PRSTR1_UART1_Pos)
#define SYS_PRSTR1_UART2_Pos		8
#define SYS_PRSTR1_UART2_Msk		(0x01 << SYS_PRSTR1_UART2_Pos)
#define SYS_PRSTR1_UART3_Pos		9
#define SYS_PRSTR1_UART3_Msk		(0x01 << SYS_PRSTR1_UART3_Pos)
#define SYS_PRSTR1_WDT_Pos			10
#define SYS_PRSTR1_WDT_Msk			(0x01 << SYS_PRSTR1_WDT_Pos)
#define SYS_PRSTR1_TIMR_Pos			11
#define SYS_PRSTR1_TIMR_Msk			(0x01 << SYS_PRSTR1_TIMR_Pos)
#define SYS_PRSTR1_PWM_Pos			12
#define SYS_PRSTR1_PWM_Msk			(0x01 << SYS_PRSTR1_PWM_Pos)
#define SYS_PRSTR1_SPI0_Pos			13
#define SYS_PRSTR1_SPI0_Msk			(0x01 << SYS_PRSTR1_SPI0_Pos)
#define SYS_PRSTR1_SPI1_Pos			14
#define SYS_PRSTR1_SPI1_Msk			(0x01 << SYS_PRSTR1_SPI1_Pos)
#define SYS_PRSTR1_I2C0_Pos			15
#define SYS_PRSTR1_I2C0_Msk			(0x01 << SYS_PRSTR1_I2C0_Pos)
#define SYS_PRSTR1_I2C1_Pos			16
#define SYS_PRSTR1_I2C1_Msk			(0x01 << SYS_PRSTR1_I2C1_Pos)
#define SYS_PRSTR1_CRC_Pos			19
#define SYS_PRSTR1_CRC_Msk			(0x01 << SYS_PRSTR1_CRC_Pos)
#define SYS_PRSTR1_DIV_Pos			21
#define SYS_PRSTR1_DIV_Msk			(0x01 << SYS_PRSTR1_DIV_Pos)
#define SYS_PRSTR1_ANAC_Pos			25
#define SYS_PRSTR1_ANAC_Msk			(0x01 << SYS_PRSTR1_ANAC_Pos)
#define SYS_PRSTR1_ADC0_Pos			26
#define SYS_PRSTR1_ADC0_Msk			(0x01 << SYS_PRSTR1_ADC0_Pos)
#define SYS_PRSTR1_CAN_Pos			28
#define SYS_PRSTR1_CAN_Msk			(0x01u<< SYS_PRSTR1_CAN_Pos)
#define SYS_PRSTR1_SLCD_Pos			29
#define SYS_PRSTR1_SLCD_Msk			(0x01u<< SYS_PRSTR1_SLCD_Pos)
#define SYS_PRSTR1_SLED_Pos			31
#define SYS_PRSTR1_SLED_Msk			(0x01u<< SYS_PRSTR1_SLED_Pos)

#define SYS_PRSTR2_RTC_Pos			19
#define SYS_PRSTR2_RTC_Msk			(0x01 << SYS_PRSTR2_RTC_Pos)

#define SYS_HRCCR_ON_Pos			0		//High speed RC ON
#define SYS_HRCCR_ON_Msk			(0x01 << SYS_HRCCR_ON_Pos)

#define SYS_BODCR_IE_Pos		    1		//Interrupt Enable
#define SYS_BODCR_IE_Msk		    (0x01 << SYS_BODCR_IE_Pos)
#define SYS_BODCR_INTLVL_Pos		4		//BOD�жϴ�����ƽ��0 1.9V   1 2.1V   2 2.3V   3 2.5V   4 2.7V   5 3.5V   6 4.1V
#define SYS_BODCR_INTLVL_Msk		(0x07 << SYS_BODCR_INTLVL_Pos)
#define SYS_BODCR_RSTLVL_Pos		7		//BOD��λ��ƽ��0 1.7V   1 1.9V   2 2.1V   3 2.7V   4 3.5V
#define SYS_BODCR_RSTLVL_Msk		(0x07 << SYS_BODCR_RSTLVL_Pos)

#define SYS_BODSR_IF_Pos			0		//�жϱ�־��д1����
#define SYS_BODSR_IF_Msk			(0x01 << SYS_BODSR_IF_Pos)

#define SYS_XTALCR_32KON_Pos		0		//XTAL_32K On
#define SYS_XTALCR_32KON_Msk		(0x01 << SYS_XTALCR_32KON_Pos)
#define SYS_XTALCR_ON_Pos			1		//XTAL On
#define SYS_XTALCR_ON_Msk			(0x01 << SYS_XTALCR_ON_Pos)
#define SYS_XTALCR_32KDET_Pos		4		//XTAL_32K Stop Detect
#define SYS_XTALCR_32KDET_Msk		(0x01 << SYS_XTALCR_32KDET_Pos)
#define SYS_XTALCR_DET_Pos			5		//XTAL Stop Detect
#define SYS_XTALCR_DET_Msk			(0x01 << SYS_XTALCR_DET_Pos)
#define SYS_XTALCR_32KDRV_Pos		8		//XTAL_32K ������������΢��Ƶ��
#define SYS_XTALCR_32KDRV_Msk		(0x0F << SYS_XTALCR_32KDRV_Pos)
#define SYS_XTALCR_DRV_Pos			16		//XTAL ������������΢��Ƶ��
#define SYS_XTALCR_DRV_Msk			(0x1F << SYS_XTALCR_DRV_Pos)

#define SYS_XTALSR_32KSTOP_Pos		0		//XTAL_32K Stop��д1����
#define SYS_XTALSR_32KSTOP_Msk		(0x01 << SYS_XTALSR_32KSTOP_Pos)
#define SYS_XTALSR_STOP_Pos			1		//XTAL Stop��д1����
#define SYS_XTALSR_STOP_Msk			(0x01 << SYS_XTALSR_STOP_Pos)

#define SYS_LRCCR_ON_Pos			0		//Low Speed RC On
#define SYS_LRCCR_ON_Msk			(0x01 << SYS_LRCCR_ON_Pos)




typedef struct {
	__IO uint32_t FUNSEL0;
	
	__IO uint32_t FUNSEL1;
	
		 uint32_t RESERVED[62];
	
	__IO uint32_t PULLU;                    //PULLU[n]Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 ������ֹ
	
		 uint32_t RESERVED2[63];
	
	__IO uint32_t PULLD;                    //PULLD[n]Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 ������ֹ
	
		 uint32_t RESERVED3[63];
	
	__IO uint32_t INEN;                     //INEN[n] Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 �����ֹ
	
		 uint32_t RESERVED4[63];
	
	__IO uint32_t OPEND;                    //OPEND[n]Ϊ PINn���� ��©ʹ��λ�� 1 ��©ʹ��	0 ��©��ֹ
} PORT_TypeDef;


#define PORT_FUNSEL0_PIN0_Pos		0
#define PORT_FUNSEL0_PIN0_Msk		(0x0F << PORT_FUNSEL0_PIN0_Pos)
#define PORT_FUNSEL0_PIN1_Pos		4
#define PORT_FUNSEL0_PIN1_Msk		(0x0F << PORT_FUNSEL0_PIN1_Pos)
#define PORT_FUNSEL0_PIN2_Pos		8
#define PORT_FUNSEL0_PIN2_Msk		(0x0F << PORT_FUNSEL0_PIN2_Pos)
#define PORT_FUNSEL0_PIN3_Pos		12
#define PORT_FUNSEL0_PIN3_Msk		(0x0F << PORT_FUNSEL0_PIN3_Pos)
#define PORT_FUNSEL0_PIN4_Pos		16
#define PORT_FUNSEL0_PIN4_Msk		(0x0F << PORT_FUNSEL0_PIN4_Pos)
#define PORT_FUNSEL0_PIN5_Pos		20
#define PORT_FUNSEL0_PIN5_Msk		(0x0F << PORT_FUNSEL0_PIN5_Pos)
#define PORT_FUNSEL0_PIN6_Pos		24
#define PORT_FUNSEL0_PIN6_Msk		(0x0F << PORT_FUNSEL0_PIN6_Pos)
#define PORT_FUNSEL0_PIN7_Pos		28
#define PORT_FUNSEL0_PIN7_Msk		(0x0Fu<< PORT_FUNSEL0_PIN7_Pos)

#define PORT_FUNSEL1_PIN8_Pos		0
#define PORT_FUNSEL1_PIN8_Msk		(0x0F << PORT_FUNSEL1_PIN8_Pos)
#define PORT_FUNSEL1_PIN9_Pos		4
#define PORT_FUNSEL1_PIN9_Msk		(0x0F << PORT_FUNSEL1_PIN9_Pos)
#define PORT_FUNSEL1_PIN10_Pos		8
#define PORT_FUNSEL1_PIN10_Msk		(0x0F << PORT_FUNSEL1_PIN10_Pos)
#define PORT_FUNSEL1_PIN11_Pos		12
#define PORT_FUNSEL1_PIN11_Msk		(0x0F << PORT_FUNSEL1_PIN11_Pos)
#define PORT_FUNSEL1_PIN12_Pos		16
#define PORT_FUNSEL1_PIN12_Msk		(0x0F << PORT_FUNSEL1_PIN12_Pos)
#define PORT_FUNSEL1_PIN13_Pos		20
#define PORT_FUNSEL1_PIN13_Msk		(0x0F << PORT_FUNSEL1_PIN13_Pos)
#define PORT_FUNSEL1_PIN14_Pos		24
#define PORT_FUNSEL1_PIN14_Msk		(0x0F << PORT_FUNSEL1_PIN14_Pos)
#define PORT_FUNSEL1_PIN15_Pos		28
#define PORT_FUNSEL1_PIN15_Msk		(0x0Fu<< PORT_FUNSEL1_PIN15_Pos)

#define PORT_PULLU_PIN0_Pos			0
#define PORT_PULLU_PIN0_Msk			(0x01 << PORT_PULLU_PIN0_Pos)
#define PORT_PULLU_PIN1_Pos			1
#define PORT_PULLU_PIN1_Msk			(0x01 << PORT_PULLU_PIN1_Pos)
#define PORT_PULLU_PIN2_Pos			2
#define PORT_PULLU_PIN2_Msk			(0x01 << PORT_PULLU_PIN2_Pos)
#define PORT_PULLU_PIN3_Pos			3
#define PORT_PULLU_PIN3_Msk			(0x01 << PORT_PULLU_PIN3_Pos)
#define PORT_PULLU_PIN4_Pos			4
#define PORT_PULLU_PIN4_Msk			(0x01 << PORT_PULLU_PIN4_Pos)
#define PORT_PULLU_PIN5_Pos			5
#define PORT_PULLU_PIN5_Msk			(0x01 << PORT_PULLU_PIN5_Pos)
#define PORT_PULLU_PIN6_Pos			6
#define PORT_PULLU_PIN6_Msk			(0x01 << PORT_PULLU_PIN6_Pos)
#define PORT_PULLU_PIN7_Pos			7
#define PORT_PULLU_PIN7_Msk			(0x01 << PORT_PULLU_PIN7_Pos)
#define PORT_PULLU_PIN8_Pos			8
#define PORT_PULLU_PIN8_Msk			(0x01 << PORT_PULLU_PIN8_Pos)
#define PORT_PULLU_PIN9_Pos			9
#define PORT_PULLU_PIN9_Msk			(0x01 << PORT_PULLU_PIN9_Pos)
#define PORT_PULLU_PIN10_Pos		10
#define PORT_PULLU_PIN10_Msk		(0x01 << PORT_PULLU_PIN10_Pos)
#define PORT_PULLU_PIN11_Pos		11
#define PORT_PULLU_PIN11_Msk		(0x01 << PORT_PULLU_PIN11_Pos)
#define PORT_PULLU_PIN12_Pos		12
#define PORT_PULLU_PIN12_Msk		(0x01 << PORT_PULLU_PIN12_Pos)
#define PORT_PULLU_PIN13_Pos		13
#define PORT_PULLU_PIN13_Msk		(0x01 << PORT_PULLU_PIN13_Pos)
#define PORT_PULLU_PIN14_Pos		14
#define PORT_PULLU_PIN14_Msk		(0x01 << PORT_PULLU_PIN14_Pos)
#define PORT_PULLU_PIN15_Pos		15
#define PORT_PULLU_PIN15_Msk		(0x01 << PORT_PULLU_PIN15_Pos)

#define PORT_PULLD_PIN0_Pos			0
#define PORT_PULLD_PIN0_Msk			(0x01 << PORT_PULLD_PIN0_Pos)
#define PORT_PULLD_PIN1_Pos			1
#define PORT_PULLD_PIN1_Msk			(0x01 << PORT_PULLD_PIN1_Pos)
#define PORT_PULLD_PIN2_Pos			2
#define PORT_PULLD_PIN2_Msk			(0x01 << PORT_PULLD_PIN2_Pos)
#define PORT_PULLD_PIN3_Pos			3
#define PORT_PULLD_PIN3_Msk			(0x01 << PORT_PULLD_PIN3_Pos)
#define PORT_PULLD_PIN4_Pos			4
#define PORT_PULLD_PIN4_Msk			(0x01 << PORT_PULLD_PIN4_Pos)
#define PORT_PULLD_PIN5_Pos			5
#define PORT_PULLD_PIN5_Msk			(0x01 << PORT_PULLD_PIN5_Pos)
#define PORT_PULLD_PIN6_Pos			6
#define PORT_PULLD_PIN6_Msk			(0x01 << PORT_PULLD_PIN6_Pos)
#define PORT_PULLD_PIN7_Pos			7
#define PORT_PULLD_PIN7_Msk			(0x01 << PORT_PULLD_PIN7_Pos)
#define PORT_PULLD_PIN8_Pos			8
#define PORT_PULLD_PIN8_Msk			(0x01 << PORT_PULLD_PIN8_Pos)
#define PORT_PULLD_PIN9_Pos			9
#define PORT_PULLD_PIN9_Msk			(0x01 << PORT_PULLD_PIN9_Pos)
#define PORT_PULLD_PIN10_Pos		10
#define PORT_PULLD_PIN10_Msk		(0x01 << PORT_PULLD_PIN10_Pos)
#define PORT_PULLD_PIN11_Pos		11
#define PORT_PULLD_PIN11_Msk		(0x01 << PORT_PULLD_PIN11_Pos)
#define PORT_PULLD_PIN12_Pos		12
#define PORT_PULLD_PIN12_Msk		(0x01 << PORT_PULLD_PIN12_Pos)
#define PORT_PULLD_PIN13_Pos		13
#define PORT_PULLD_PIN13_Msk		(0x01 << PORT_PULLD_PIN13_Pos)
#define PORT_PULLD_PIN14_Pos		14
#define PORT_PULLD_PIN14_Msk		(0x01 << PORT_PULLD_PIN14_Pos)
#define PORT_PULLD_PIN15_Pos		15
#define PORT_PULLD_PIN15_Msk		(0x01 << PORT_PULLD_PIN15_Pos)

#define PORT_INEN_PIN0_Pos			0
#define PORT_INEN_PIN0_Msk			(0x01 << PORT_INEN_PIN0_Pos)
#define PORT_INEN_PIN1_Pos			1
#define PORT_INEN_PIN1_Msk			(0x01 << PORT_INEN_PIN1_Pos)
#define PORT_INEN_PIN2_Pos			2
#define PORT_INEN_PIN2_Msk			(0x01 << PORT_INEN_PIN2_Pos)
#define PORT_INEN_PIN3_Pos			3
#define PORT_INEN_PIN3_Msk			(0x01 << PORT_INEN_PIN3_Pos)
#define PORT_INEN_PIN4_Pos			4
#define PORT_INEN_PIN4_Msk			(0x01 << PORT_INEN_PIN4_Pos)
#define PORT_INEN_PIN5_Pos			5
#define PORT_INEN_PIN5_Msk			(0x01 << PORT_INEN_PIN5_Pos)
#define PORT_INEN_PIN6_Pos			6
#define PORT_INEN_PIN6_Msk			(0x01 << PORT_INEN_PIN6_Pos)
#define PORT_INEN_PIN7_Pos			7
#define PORT_INEN_PIN7_Msk			(0x01 << PORT_INEN_PIN7_Pos)
#define PORT_INEN_PIN8_Pos			8
#define PORT_INEN_PIN8_Msk			(0x01 << PORT_INEN_PIN8_Pos)
#define PORT_INEN_PIN9_Pos			9
#define PORT_INEN_PIN9_Msk			(0x01 << PORT_INEN_PIN9_Pos)
#define PORT_INEN_PIN10_Pos			10
#define PORT_INEN_PIN10_Msk			(0x01 << PORT_INEN_PIN10_Pos)
#define PORT_INEN_PIN11_Pos			11
#define PORT_INEN_PIN11_Msk			(0x01 << PORT_INEN_PIN11_Pos)
#define PORT_INEN_PIN12_Pos			12
#define PORT_INEN_PIN12_Msk			(0x01 << PORT_INEN_PIN12_Pos)
#define PORT_INEN_PIN13_Pos			13
#define PORT_INEN_PIN13_Msk			(0x01 << PORT_INEN_PIN13_Pos)
#define PORT_INEN_PIN14_Pos			14
#define PORT_INEN_PIN14_Msk			(0x01 << PORT_INEN_PIN14_Pos)
#define PORT_INEN_PIN15_Pos			15
#define PORT_INEN_PIN15_Msk			(0x01 << PORT_INEN_PIN15_Pos)

#define PORT_OPEND_PIN0_Pos			0
#define PORT_OPEND_PIN0_Msk			(0x01 << PORT_OPEND_PIN0_Pos)
#define PORT_OPEND_PIN1_Pos			1
#define PORT_OPEND_PIN1_Msk			(0x01 << PORT_OPEND_PIN1_Pos)
#define PORT_OPEND_PIN2_Pos			2
#define PORT_OPEND_PIN2_Msk			(0x01 << PORT_OPEND_PIN2_Pos)
#define PORT_OPEND_PIN3_Pos			3
#define PORT_OPEND_PIN3_Msk			(0x01 << PORT_OPEND_PIN3_Pos)
#define PORT_OPEND_PIN4_Pos			4
#define PORT_OPEND_PIN4_Msk			(0x01 << PORT_OPEND_PIN4_Pos)
#define PORT_OPEND_PIN5_Pos			5
#define PORT_OPEND_PIN5_Msk			(0x01 << PORT_OPEND_PIN5_Pos)
#define PORT_OPEND_PIN6_Pos			6
#define PORT_OPEND_PIN6_Msk			(0x01 << PORT_OPEND_PIN6_Pos)
#define PORT_OPEND_PIN7_Pos			7
#define PORT_OPEND_PIN7_Msk			(0x01 << PORT_OPEND_PIN7_Pos)
#define PORT_OPEND_PIN8_Pos			8
#define PORT_OPEND_PIN8_Msk			(0x01 << PORT_OPEND_PIN8_Pos)
#define PORT_OPEND_PIN9_Pos			9
#define PORT_OPEND_PIN9_Msk			(0x01 << PORT_OPEND_PIN9_Pos)
#define PORT_OPEND_PIN10_Pos		10
#define PORT_OPEND_PIN10_Msk		(0x01 << PORT_OPEND_PIN10_Pos)
#define PORT_OPEND_PIN11_Pos		11
#define PORT_OPEND_PIN11_Msk		(0x01 << PORT_OPEND_PIN11_Pos)
#define PORT_OPEND_PIN12_Pos		12
#define PORT_OPEND_PIN12_Msk		(0x01 << PORT_OPEND_PIN12_Pos)
#define PORT_OPEND_PIN13_Pos		13
#define PORT_OPEND_PIN13_Msk		(0x01 << PORT_OPEND_PIN13_Pos)
#define PORT_OPEND_PIN14_Pos		14
#define PORT_OPEND_PIN14_Msk		(0x01 << PORT_OPEND_PIN14_Pos)
#define PORT_OPEND_PIN15_Pos		15
#define PORT_OPEND_PIN15_Msk		(0x01 << PORT_OPEND_PIN15_Pos)




typedef struct {
	__IO uint32_t ODR;
#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7
#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11
#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

	__IO uint32_t DIR;					    //0 ����	1 ���

	__IO uint32_t INTLVLTRG;				//Interrupt Level Trigger  1 ��ƽ�����ж�	0 ���ش����ж�

	__IO uint32_t INTBE;					//Both Edge����INTLVLTRG��Ϊ���ش����ж�ʱ����λ��1��ʾ�����غ��½��ض������жϣ���0ʱ����������INTRISEENѡ��

	__IO uint32_t INTRISEEN;				//Interrupt Rise Edge Enable   1 ������/�ߵ�ƽ�����ж�	0 �½���/�͵�ƽ�����ж�

	__IO uint32_t INTEN;					//1 �ж�ʹ��	0 �жϽ�ֹ

	__IO uint32_t INTRAWSTAT;			    //�жϼ�ⵥԪ�Ƿ��⵽�˴����жϵ����� 1 ��⵽���жϴ�������	0 û�м�⵽�жϴ�������

	__IO uint32_t INTSTAT;				    //INTSTAT.PIN0 = INTRAWSTAT.PIN0 & INTEN.PIN0

	__IO uint32_t INTCLR;				    //д1����жϱ�־��ֻ�Ա��ش����ж�����
	
	__IO uint32_t DMAEN;
	
		 uint32_t RESERVED[2];
	
	__IO uint32_t IDR;
	
		 uint32_t RESERVED2[3];
	
	__IO uint32_t DATAPIN0;					//PIN0���ŵ�DATA�Ĵ������������Ŷ�Ӧ����32λ�Ĵ���������ʵ��ԭ��д����
	__IO uint32_t DATAPIN1;
	__IO uint32_t DATAPIN2;
	__IO uint32_t DATAPIN3;
	__IO uint32_t DATAPIN4;
	__IO uint32_t DATAPIN5;
	__IO uint32_t DATAPIN6;
	__IO uint32_t DATAPIN7;
	__IO uint32_t DATAPIN8;
	__IO uint32_t DATAPIN9;
	__IO uint32_t DATAPIN10;
	__IO uint32_t DATAPIN11;
	__IO uint32_t DATAPIN12;
	__IO uint32_t DATAPIN13;
	__IO uint32_t DATAPIN14;
	__IO uint32_t DATAPIN15;
} GPIO_TypeDef;


#define GPIO_ODR_PIN0_Pos			0
#define GPIO_ODR_PIN0_Msk			(0x01 << GPIO_ODR_PIN0_Pos)
#define GPIO_ODR_PIN1_Pos			1
#define GPIO_ODR_PIN1_Msk			(0x01 << GPIO_ODR_PIN1_Pos)
#define GPIO_ODR_PIN2_Pos			2
#define GPIO_ODR_PIN2_Msk			(0x01 << GPIO_ODR_PIN2_Pos)
#define GPIO_ODR_PIN3_Pos			3
#define GPIO_ODR_PIN3_Msk			(0x01 << GPIO_ODR_PIN3_Pos)
#define GPIO_ODR_PIN4_Pos			4
#define GPIO_ODR_PIN4_Msk			(0x01 << GPIO_ODR_PIN4_Pos)
#define GPIO_ODR_PIN5_Pos			5
#define GPIO_ODR_PIN5_Msk			(0x01 << GPIO_ODR_PIN5_Pos)
#define GPIO_ODR_PIN6_Pos			6
#define GPIO_ODR_PIN6_Msk			(0x01 << GPIO_ODR_PIN6_Pos)
#define GPIO_ODR_PIN7_Pos			7
#define GPIO_ODR_PIN7_Msk			(0x01 << GPIO_ODR_PIN7_Pos)
#define GPIO_ODR_PIN8_Pos			8
#define GPIO_ODR_PIN8_Msk			(0x01 << GPIO_ODR_PIN8_Pos)
#define GPIO_ODR_PIN9_Pos			9
#define GPIO_ODR_PIN9_Msk			(0x01 << GPIO_ODR_PIN9_Pos)
#define GPIO_ODR_PIN10_Pos			10
#define GPIO_ODR_PIN10_Msk			(0x01 << GPIO_ODR_PIN10_Pos)
#define GPIO_ODR_PIN11_Pos			11
#define GPIO_ODR_PIN11_Msk			(0x01 << GPIO_ODR_PIN11_Pos)
#define GPIO_ODR_PIN12_Pos			12
#define GPIO_ODR_PIN12_Msk			(0x01 << GPIO_ODR_PIN12_Pos)
#define GPIO_ODR_PIN13_Pos			13
#define GPIO_ODR_PIN13_Msk			(0x01 << GPIO_ODR_PIN13_Pos)
#define GPIO_ODR_PIN14_Pos			14
#define GPIO_ODR_PIN14_Msk			(0x01 << GPIO_ODR_PIN14_Pos)
#define GPIO_ODR_PIN15_Pos			15
#define GPIO_ODR_PIN15_Msk			(0x01 << GPIO_ODR_PIN15_Pos)

#define GPIO_DIR_PIN0_Pos			0
#define GPIO_DIR_PIN0_Msk			(0x01 << GPIO_DIR_PIN0_Pos)
#define GPIO_DIR_PIN1_Pos			1
#define GPIO_DIR_PIN1_Msk			(0x01 << GPIO_DIR_PIN1_Pos)
#define GPIO_DIR_PIN2_Pos			2
#define GPIO_DIR_PIN2_Msk			(0x01 << GPIO_DIR_PIN2_Pos)
#define GPIO_DIR_PIN3_Pos			3
#define GPIO_DIR_PIN3_Msk			(0x01 << GPIO_DIR_PIN3_Pos)
#define GPIO_DIR_PIN4_Pos			4
#define GPIO_DIR_PIN4_Msk			(0x01 << GPIO_DIR_PIN4_Pos)
#define GPIO_DIR_PIN5_Pos			5
#define GPIO_DIR_PIN5_Msk			(0x01 << GPIO_DIR_PIN5_Pos)
#define GPIO_DIR_PIN6_Pos			6
#define GPIO_DIR_PIN6_Msk			(0x01 << GPIO_DIR_PIN6_Pos)
#define GPIO_DIR_PIN7_Pos			7
#define GPIO_DIR_PIN7_Msk			(0x01 << GPIO_DIR_PIN7_Pos)
#define GPIO_DIR_PIN8_Pos			8
#define GPIO_DIR_PIN8_Msk			(0x01 << GPIO_DIR_PIN8_Pos)
#define GPIO_DIR_PIN9_Pos			9
#define GPIO_DIR_PIN9_Msk			(0x01 << GPIO_DIR_PIN9_Pos)
#define GPIO_DIR_PIN10_Pos			10
#define GPIO_DIR_PIN10_Msk			(0x01 << GPIO_DIR_PIN10_Pos)
#define GPIO_DIR_PIN11_Pos			11
#define GPIO_DIR_PIN11_Msk			(0x01 << GPIO_DIR_PIN11_Pos)
#define GPIO_DIR_PIN12_Pos			12
#define GPIO_DIR_PIN12_Msk			(0x01 << GPIO_DIR_PIN12_Pos)
#define GPIO_DIR_PIN13_Pos			13
#define GPIO_DIR_PIN13_Msk			(0x01 << GPIO_DIR_PIN13_Pos)
#define GPIO_DIR_PIN14_Pos			14
#define GPIO_DIR_PIN14_Msk			(0x01 << GPIO_DIR_PIN14_Pos)
#define GPIO_DIR_PIN15_Pos			15
#define GPIO_DIR_PIN15_Msk			(0x01 << GPIO_DIR_PIN15_Pos)

#define GPIO_INTLVLTRG_PIN0_Pos		0
#define GPIO_INTLVLTRG_PIN0_Msk		(0x01 << GPIO_INTLVLTRG_PIN0_Pos)
#define GPIO_INTLVLTRG_PIN1_Pos		1
#define GPIO_INTLVLTRG_PIN1_Msk		(0x01 << GPIO_INTLVLTRG_PIN1_Pos)
#define GPIO_INTLVLTRG_PIN2_Pos		2
#define GPIO_INTLVLTRG_PIN2_Msk		(0x01 << GPIO_INTLVLTRG_PIN2_Pos)
#define GPIO_INTLVLTRG_PIN3_Pos		3
#define GPIO_INTLVLTRG_PIN3_Msk		(0x01 << GPIO_INTLVLTRG_PIN3_Pos)
#define GPIO_INTLVLTRG_PIN4_Pos		4
#define GPIO_INTLVLTRG_PIN4_Msk		(0x01 << GPIO_INTLVLTRG_PIN4_Pos)
#define GPIO_INTLVLTRG_PIN5_Pos		5
#define GPIO_INTLVLTRG_PIN5_Msk		(0x01 << GPIO_INTLVLTRG_PIN5_Pos)
#define GPIO_INTLVLTRG_PIN6_Pos		6
#define GPIO_INTLVLTRG_PIN6_Msk		(0x01 << GPIO_INTLVLTRG_PIN6_Pos)
#define GPIO_INTLVLTRG_PIN7_Pos		7
#define GPIO_INTLVLTRG_PIN7_Msk		(0x01 << GPIO_INTLVLTRG_PIN7_Pos)
#define GPIO_INTLVLTRG_PIN8_Pos		8
#define GPIO_INTLVLTRG_PIN8_Msk		(0x01 << GPIO_INTLVLTRG_PIN8_Pos)
#define GPIO_INTLVLTRG_PIN9_Pos		9
#define GPIO_INTLVLTRG_PIN9_Msk		(0x01 << GPIO_INTLVLTRG_PIN9_Pos)
#define GPIO_INTLVLTRG_PIN10_Pos	10
#define GPIO_INTLVLTRG_PIN10_Msk	(0x01 << GPIO_INTLVLTRG_PIN10_Pos)
#define GPIO_INTLVLTRG_PIN11_Pos	11
#define GPIO_INTLVLTRG_PIN11_Msk	(0x01 << GPIO_INTLVLTRG_PIN11_Pos)
#define GPIO_INTLVLTRG_PIN12_Pos	12
#define GPIO_INTLVLTRG_PIN12_Msk	(0x01 << GPIO_INTLVLTRG_PIN12_Pos)
#define GPIO_INTLVLTRG_PIN13_Pos	13
#define GPIO_INTLVLTRG_PIN13_Msk	(0x01 << GPIO_INTLVLTRG_PIN13_Pos)
#define GPIO_INTLVLTRG_PIN14_Pos	14
#define GPIO_INTLVLTRG_PIN14_Msk	(0x01 << GPIO_INTLVLTRG_PIN14_Pos)
#define GPIO_INTLVLTRG_PIN15_Pos	15
#define GPIO_INTLVLTRG_PIN15_Msk	(0x01 << GPIO_INTLVLTRG_PIN15_Pos)

#define GPIO_INTBE_PIN0_Pos			0
#define GPIO_INTBE_PIN0_Msk			(0x01 << GPIO_INTBE_PIN0_Pos)
#define GPIO_INTBE_PIN1_Pos			1
#define GPIO_INTBE_PIN1_Msk			(0x01 << GPIO_INTBE_PIN1_Pos)
#define GPIO_INTBE_PIN2_Pos			2
#define GPIO_INTBE_PIN2_Msk			(0x01 << GPIO_INTBE_PIN2_Pos)
#define GPIO_INTBE_PIN3_Pos			3
#define GPIO_INTBE_PIN3_Msk			(0x01 << GPIO_INTBE_PIN3_Pos)
#define GPIO_INTBE_PIN4_Pos			4
#define GPIO_INTBE_PIN4_Msk			(0x01 << GPIO_INTBE_PIN4_Pos)
#define GPIO_INTBE_PIN5_Pos			5
#define GPIO_INTBE_PIN5_Msk			(0x01 << GPIO_INTBE_PIN5_Pos)
#define GPIO_INTBE_PIN6_Pos			6
#define GPIO_INTBE_PIN6_Msk			(0x01 << GPIO_INTBE_PIN6_Pos)
#define GPIO_INTBE_PIN7_Pos			7
#define GPIO_INTBE_PIN7_Msk			(0x01 << GPIO_INTBE_PIN7_Pos)
#define GPIO_INTBE_PIN8_Pos			8
#define GPIO_INTBE_PIN8_Msk			(0x01 << GPIO_INTBE_PIN8_Pos)
#define GPIO_INTBE_PIN9_Pos			9
#define GPIO_INTBE_PIN9_Msk			(0x01 << GPIO_INTBE_PIN9_Pos)
#define GPIO_INTBE_PIN10_Pos		10
#define GPIO_INTBE_PIN10_Msk		(0x01 << GPIO_INTBE_PIN10_Pos)
#define GPIO_INTBE_PIN11_Pos		11
#define GPIO_INTBE_PIN11_Msk		(0x01 << GPIO_INTBE_PIN11_Pos)
#define GPIO_INTBE_PIN12_Pos		12
#define GPIO_INTBE_PIN12_Msk		(0x01 << GPIO_INTBE_PIN12_Pos)
#define GPIO_INTBE_PIN13_Pos		13
#define GPIO_INTBE_PIN13_Msk		(0x01 << GPIO_INTBE_PIN13_Pos)
#define GPIO_INTBE_PIN14_Pos		14
#define GPIO_INTBE_PIN14_Msk		(0x01 << GPIO_INTBE_PIN14_Pos)
#define GPIO_INTBE_PIN15_Pos		15
#define GPIO_INTBE_PIN15_Msk		(0x01 << GPIO_INTBE_PIN15_Pos)

#define GPIO_INTRISEEN_PIN0_Pos		0
#define GPIO_INTRISEEN_PIN0_Msk		(0x01 << GPIO_INTRISEEN_PIN0_Pos)
#define GPIO_INTRISEEN_PIN1_Pos		1
#define GPIO_INTRISEEN_PIN1_Msk		(0x01 << GPIO_INTRISEEN_PIN1_Pos)
#define GPIO_INTRISEEN_PIN2_Pos		2
#define GPIO_INTRISEEN_PIN2_Msk		(0x01 << GPIO_INTRISEEN_PIN2_Pos)
#define GPIO_INTRISEEN_PIN3_Pos		3
#define GPIO_INTRISEEN_PIN3_Msk		(0x01 << GPIO_INTRISEEN_PIN3_Pos)
#define GPIO_INTRISEEN_PIN4_Pos		4
#define GPIO_INTRISEEN_PIN4_Msk		(0x01 << GPIO_INTRISEEN_PIN4_Pos)
#define GPIO_INTRISEEN_PIN5_Pos		5
#define GPIO_INTRISEEN_PIN5_Msk		(0x01 << GPIO_INTRISEEN_PIN5_Pos)
#define GPIO_INTRISEEN_PIN6_Pos		6
#define GPIO_INTRISEEN_PIN6_Msk		(0x01 << GPIO_INTRISEEN_PIN6_Pos)
#define GPIO_INTRISEEN_PIN7_Pos		7
#define GPIO_INTRISEEN_PIN7_Msk		(0x01 << GPIO_INTRISEEN_PIN7_Pos)
#define GPIO_INTRISEEN_PIN8_Pos		8
#define GPIO_INTRISEEN_PIN8_Msk		(0x01 << GPIO_INTRISEEN_PIN8_Pos)
#define GPIO_INTRISEEN_PIN9_Pos		9
#define GPIO_INTRISEEN_PIN9_Msk		(0x01 << GPIO_INTRISEEN_PIN9_Pos)
#define GPIO_INTRISEEN_PIN10_Pos	10
#define GPIO_INTRISEEN_PIN10_Msk	(0x01 << GPIO_INTRISEEN_PIN10_Pos)
#define GPIO_INTRISEEN_PIN11_Pos	11
#define GPIO_INTRISEEN_PIN11_Msk	(0x01 << GPIO_INTRISEEN_PIN11_Pos)
#define GPIO_INTRISEEN_PIN12_Pos	12
#define GPIO_INTRISEEN_PIN12_Msk	(0x01 << GPIO_INTRISEEN_PIN12_Pos)
#define GPIO_INTRISEEN_PIN13_Pos	13
#define GPIO_INTRISEEN_PIN13_Msk	(0x01 << GPIO_INTRISEEN_PIN13_Pos)
#define GPIO_INTRISEEN_PIN14_Pos	14
#define GPIO_INTRISEEN_PIN14_Msk	(0x01 << GPIO_INTRISEEN_PIN14_Pos)
#define GPIO_INTRISEEN_PIN15_Pos	15
#define GPIO_INTRISEEN_PIN15_Msk	(0x01 << GPIO_INTRISEEN_PIN15_Pos)

#define GPIO_INTEN_PIN0_Pos			0
#define GPIO_INTEN_PIN0_Msk			(0x01 << GPIO_INTEN_PIN0_Pos)
#define GPIO_INTEN_PIN1_Pos			1
#define GPIO_INTEN_PIN1_Msk			(0x01 << GPIO_INTEN_PIN1_Pos)
#define GPIO_INTEN_PIN2_Pos			2
#define GPIO_INTEN_PIN2_Msk			(0x01 << GPIO_INTEN_PIN2_Pos)
#define GPIO_INTEN_PIN3_Pos			3
#define GPIO_INTEN_PIN3_Msk			(0x01 << GPIO_INTEN_PIN3_Pos)
#define GPIO_INTEN_PIN4_Pos			4
#define GPIO_INTEN_PIN4_Msk			(0x01 << GPIO_INTEN_PIN4_Pos)
#define GPIO_INTEN_PIN5_Pos			5
#define GPIO_INTEN_PIN5_Msk			(0x01 << GPIO_INTEN_PIN5_Pos)
#define GPIO_INTEN_PIN6_Pos			6
#define GPIO_INTEN_PIN6_Msk			(0x01 << GPIO_INTEN_PIN6_Pos)
#define GPIO_INTEN_PIN7_Pos			7
#define GPIO_INTEN_PIN7_Msk			(0x01 << GPIO_INTEN_PIN7_Pos)
#define GPIO_INTEN_PIN8_Pos			8
#define GPIO_INTEN_PIN8_Msk			(0x01 << GPIO_INTEN_PIN8_Pos)
#define GPIO_INTEN_PIN9_Pos			9
#define GPIO_INTEN_PIN9_Msk			(0x01 << GPIO_INTEN_PIN9_Pos)
#define GPIO_INTEN_PIN10_Pos		10
#define GPIO_INTEN_PIN10_Msk		(0x01 << GPIO_INTEN_PIN10_Pos)
#define GPIO_INTEN_PIN11_Pos		11
#define GPIO_INTEN_PIN11_Msk		(0x01 << GPIO_INTEN_PIN11_Pos)
#define GPIO_INTEN_PIN12_Pos		12
#define GPIO_INTEN_PIN12_Msk		(0x01 << GPIO_INTEN_PIN12_Pos)
#define GPIO_INTEN_PIN13_Pos		13
#define GPIO_INTEN_PIN13_Msk		(0x01 << GPIO_INTEN_PIN13_Pos)
#define GPIO_INTEN_PIN14_Pos		14
#define GPIO_INTEN_PIN14_Msk		(0x01 << GPIO_INTEN_PIN14_Pos)
#define GPIO_INTEN_PIN15_Pos		15
#define GPIO_INTEN_PIN15_Msk		(0x01 << GPIO_INTEN_PIN15_Pos)

#define GPIO_INTRAWSTAT_PIN0_Pos	0
#define GPIO_INTRAWSTAT_PIN0_Msk	(0x01 << GPIO_INTRAWSTAT_PIN0_Pos)
#define GPIO_INTRAWSTAT_PIN1_Pos	1
#define GPIO_INTRAWSTAT_PIN1_Msk	(0x01 << GPIO_INTRAWSTAT_PIN1_Pos)
#define GPIO_INTRAWSTAT_PIN2_Pos	2
#define GPIO_INTRAWSTAT_PIN2_Msk	(0x01 << GPIO_INTRAWSTAT_PIN2_Pos)
#define GPIO_INTRAWSTAT_PIN3_Pos	3
#define GPIO_INTRAWSTAT_PIN3_Msk	(0x01 << GPIO_INTRAWSTAT_PIN3_Pos)
#define GPIO_INTRAWSTAT_PIN4_Pos	4
#define GPIO_INTRAWSTAT_PIN4_Msk	(0x01 << GPIO_INTRAWSTAT_PIN4_Pos)
#define GPIO_INTRAWSTAT_PIN5_Pos	5
#define GPIO_INTRAWSTAT_PIN5_Msk	(0x01 << GPIO_INTRAWSTAT_PIN5_Pos)
#define GPIO_INTRAWSTAT_PIN6_Pos	6
#define GPIO_INTRAWSTAT_PIN6_Msk	(0x01 << GPIO_INTRAWSTAT_PIN6_Pos)
#define GPIO_INTRAWSTAT_PIN7_Pos	7
#define GPIO_INTRAWSTAT_PIN7_Msk	(0x01 << GPIO_INTRAWSTAT_PIN7_Pos)
#define GPIO_INTRAWSTAT_PIN8_Pos	8
#define GPIO_INTRAWSTAT_PIN8_Msk	(0x01 << GPIO_INTRAWSTAT_PIN8_Pos)
#define GPIO_INTRAWSTAT_PIN9_Pos	9
#define GPIO_INTRAWSTAT_PIN9_Msk	(0x01 << GPIO_INTRAWSTAT_PIN9_Pos)
#define GPIO_INTRAWSTAT_PIN10_Pos	10
#define GPIO_INTRAWSTAT_PIN10_Msk	(0x01 << GPIO_INTRAWSTAT_PIN10_Pos)
#define GPIO_INTRAWSTAT_PIN11_Pos	11
#define GPIO_INTRAWSTAT_PIN11_Msk	(0x01 << GPIO_INTRAWSTAT_PIN11_Pos)
#define GPIO_INTRAWSTAT_PIN12_Pos	12
#define GPIO_INTRAWSTAT_PIN12_Msk	(0x01 << GPIO_INTRAWSTAT_PIN12_Pos)
#define GPIO_INTRAWSTAT_PIN13_Pos	13
#define GPIO_INTRAWSTAT_PIN13_Msk	(0x01 << GPIO_INTRAWSTAT_PIN13_Pos)
#define GPIO_INTRAWSTAT_PIN14_Pos	14
#define GPIO_INTRAWSTAT_PIN14_Msk	(0x01 << GPIO_INTRAWSTAT_PIN14_Pos)
#define GPIO_INTRAWSTAT_PIN15_Pos	15
#define GPIO_INTRAWSTAT_PIN15_Msk	(0x01 << GPIO_INTRAWSTAT_PIN15_Pos)

#define GPIO_INTSTAT_PIN0_Pos		0
#define GPIO_INTSTAT_PIN0_Msk		(0x01 << GPIO_INTSTAT_PIN0_Pos)
#define GPIO_INTSTAT_PIN1_Pos		1
#define GPIO_INTSTAT_PIN1_Msk		(0x01 << GPIO_INTSTAT_PIN1_Pos)
#define GPIO_INTSTAT_PIN2_Pos		2
#define GPIO_INTSTAT_PIN2_Msk		(0x01 << GPIO_INTSTAT_PIN2_Pos)
#define GPIO_INTSTAT_PIN3_Pos		3
#define GPIO_INTSTAT_PIN3_Msk		(0x01 << GPIO_INTSTAT_PIN3_Pos)
#define GPIO_INTSTAT_PIN4_Pos		4
#define GPIO_INTSTAT_PIN4_Msk		(0x01 << GPIO_INTSTAT_PIN4_Pos)
#define GPIO_INTSTAT_PIN5_Pos		5
#define GPIO_INTSTAT_PIN5_Msk		(0x01 << GPIO_INTSTAT_PIN5_Pos)
#define GPIO_INTSTAT_PIN6_Pos		6
#define GPIO_INTSTAT_PIN6_Msk		(0x01 << GPIO_INTSTAT_PIN6_Pos)
#define GPIO_INTSTAT_PIN7_Pos		7
#define GPIO_INTSTAT_PIN7_Msk		(0x01 << GPIO_INTSTAT_PIN7_Pos)
#define GPIO_INTSTAT_PIN8_Pos		8
#define GPIO_INTSTAT_PIN8_Msk		(0x01 << GPIO_INTSTAT_PIN8_Pos)
#define GPIO_INTSTAT_PIN9_Pos		9
#define GPIO_INTSTAT_PIN9_Msk		(0x01 << GPIO_INTSTAT_PIN9_Pos)
#define GPIO_INTSTAT_PIN10_Pos		10
#define GPIO_INTSTAT_PIN10_Msk		(0x01 << GPIO_INTSTAT_PIN10_Pos)
#define GPIO_INTSTAT_PIN11_Pos		11
#define GPIO_INTSTAT_PIN11_Msk		(0x01 << GPIO_INTSTAT_PIN11_Pos)
#define GPIO_INTSTAT_PIN12_Pos		12
#define GPIO_INTSTAT_PIN12_Msk		(0x01 << GPIO_INTSTAT_PIN12_Pos)
#define GPIO_INTSTAT_PIN13_Pos		13
#define GPIO_INTSTAT_PIN13_Msk		(0x01 << GPIO_INTSTAT_PIN13_Pos)
#define GPIO_INTSTAT_PIN14_Pos		14
#define GPIO_INTSTAT_PIN14_Msk		(0x01 << GPIO_INTSTAT_PIN14_Pos)
#define GPIO_INTSTAT_PIN15_Pos		15
#define GPIO_INTSTAT_PIN15_Msk		(0x01 << GPIO_INTSTAT_PIN15_Pos)

#define GPIO_INTCLR_PIN0_Pos		0
#define GPIO_INTCLR_PIN0_Msk		(0x01 << GPIO_INTCLR_PIN0_Pos)
#define GPIO_INTCLR_PIN1_Pos		1
#define GPIO_INTCLR_PIN1_Msk		(0x01 << GPIO_INTCLR_PIN1_Pos)
#define GPIO_INTCLR_PIN2_Pos		2
#define GPIO_INTCLR_PIN2_Msk		(0x01 << GPIO_INTCLR_PIN2_Pos)
#define GPIO_INTCLR_PIN3_Pos		3
#define GPIO_INTCLR_PIN3_Msk		(0x01 << GPIO_INTCLR_PIN3_Pos)
#define GPIO_INTCLR_PIN4_Pos		4
#define GPIO_INTCLR_PIN4_Msk		(0x01 << GPIO_INTCLR_PIN4_Pos)
#define GPIO_INTCLR_PIN5_Pos		5
#define GPIO_INTCLR_PIN5_Msk		(0x01 << GPIO_INTCLR_PIN5_Pos)
#define GPIO_INTCLR_PIN6_Pos		6
#define GPIO_INTCLR_PIN6_Msk		(0x01 << GPIO_INTCLR_PIN6_Pos)
#define GPIO_INTCLR_PIN7_Pos		7
#define GPIO_INTCLR_PIN7_Msk		(0x01 << GPIO_INTCLR_PIN7_Pos)
#define GPIO_INTCLR_PIN8_Pos		8
#define GPIO_INTCLR_PIN8_Msk		(0x01 << GPIO_INTCLR_PIN8_Pos)
#define GPIO_INTCLR_PIN9_Pos		9
#define GPIO_INTCLR_PIN9_Msk		(0x01 << GPIO_INTCLR_PIN9_Pos)
#define GPIO_INTCLR_PIN10_Pos		10
#define GPIO_INTCLR_PIN10_Msk		(0x01 << GPIO_INTCLR_PIN10_Pos)
#define GPIO_INTCLR_PIN11_Pos		11
#define GPIO_INTCLR_PIN11_Msk		(0x01 << GPIO_INTCLR_PIN11_Pos)
#define GPIO_INTCLR_PIN12_Pos		12
#define GPIO_INTCLR_PIN12_Msk		(0x01 << GPIO_INTCLR_PIN12_Pos)
#define GPIO_INTCLR_PIN13_Pos		13
#define GPIO_INTCLR_PIN13_Msk		(0x01 << GPIO_INTCLR_PIN13_Pos)
#define GPIO_INTCLR_PIN14_Pos		14
#define GPIO_INTCLR_PIN14_Msk		(0x01 << GPIO_INTCLR_PIN14_Pos)
#define GPIO_INTCLR_PIN15_Pos		15
#define GPIO_INTCLR_PIN15_Msk		(0x01 << GPIO_INTCLR_PIN15_Pos)

#define GPIO_IDR_PIN0_Pos			0
#define GPIO_IDR_PIN0_Msk			(0x01 << GPIO_IDR_PIN0_Pos)
#define GPIO_IDR_PIN1_Pos			1
#define GPIO_IDR_PIN1_Msk			(0x01 << GPIO_IDR_PIN1_Pos)
#define GPIO_IDR_PIN2_Pos			2
#define GPIO_IDR_PIN2_Msk			(0x01 << GPIO_IDR_PIN2_Pos)
#define GPIO_IDR_PIN3_Pos			3
#define GPIO_IDR_PIN3_Msk			(0x01 << GPIO_IDR_PIN3_Pos)
#define GPIO_IDR_PIN4_Pos			4
#define GPIO_IDR_PIN4_Msk			(0x01 << GPIO_IDR_PIN4_Pos)
#define GPIO_IDR_PIN5_Pos			5
#define GPIO_IDR_PIN5_Msk			(0x01 << GPIO_IDR_PIN5_Pos)
#define GPIO_IDR_PIN6_Pos			6
#define GPIO_IDR_PIN6_Msk			(0x01 << GPIO_IDR_PIN6_Pos)
#define GPIO_IDR_PIN7_Pos			7
#define GPIO_IDR_PIN7_Msk			(0x01 << GPIO_IDR_PIN7_Pos)
#define GPIO_IDR_PIN8_Pos			8
#define GPIO_IDR_PIN8_Msk			(0x01 << GPIO_IDR_PIN8_Pos)
#define GPIO_IDR_PIN9_Pos			9
#define GPIO_IDR_PIN9_Msk			(0x01 << GPIO_IDR_PIN9_Pos)
#define GPIO_IDR_PIN10_Pos			10
#define GPIO_IDR_PIN10_Msk			(0x01 << GPIO_IDR_PIN10_Pos)
#define GPIO_IDR_PIN11_Pos			11
#define GPIO_IDR_PIN11_Msk			(0x01 << GPIO_IDR_PIN11_Pos)
#define GPIO_IDR_PIN12_Pos			12
#define GPIO_IDR_PIN12_Msk			(0x01 << GPIO_IDR_PIN12_Pos)
#define GPIO_IDR_PIN13_Pos			13
#define GPIO_IDR_PIN13_Msk			(0x01 << GPIO_IDR_PIN13_Pos)
#define GPIO_IDR_PIN14_Pos			14
#define GPIO_IDR_PIN14_Msk			(0x01 << GPIO_IDR_PIN14_Pos)
#define GPIO_IDR_PIN15_Pos			15
#define GPIO_IDR_PIN15_Msk			(0x01 << GPIO_IDR_PIN15_Pos)




typedef struct {
	__IO uint32_t LOAD;						//��ʱ������ֵ��ʹ�ܺ�ʱ���Ӵ���ֵ��ʼ���µݼ�����

	__I  uint32_t VALUE;					//��ʱ����ǰֵ��LOAD-VALUE �ɼ������ʱʱ��

	__IO uint32_t CR;
	
		 uint32_t RESERVED;
	
	__IO uint32_t IE;

	__IO uint32_t IF;
	
	__IO uint32_t HALT;						//[0] 1 ��ͣ����    0 �ָ�����
	
	__IO uint32_t OCCR;
	
	__IO uint32_t OCMAT;
	__IO uint32_t RESERVED2;
	
	__IO uint32_t ICLOW;
	__IO uint32_t ICHIGH;
	
	__IO uint32_t PSC;						//[7:0] ��ʱ��Ԥ��Ƶ
} TIMR_TypeDef;


#define TIMR_LOAD_VALUE_Pos			0
#define TIMR_LOAD_VALUE_Msk			(0xFFFFFF << TIMR_LOAD_VALUE_Pos)
#define TIMR_LOAD_RELOAD_Pos		24		//reload VALUE to TIMR's internal Counter immediately. only for BTIMRx, not for TIMRx.
#define TIMR_LOAD_RELOAD_Msk		(0x01 << TIMR_LOAD_RELOAD_Pos)

#define TIMR_CR_CLKSRC_Pos			0		//ʱ��Դ��  0 �ڲ�ϵͳʱ��	2 �ⲿ�����������
#define TIMR_CR_CLKSRC_Msk			(0x03 << TIMR_CR_CLKSRC_Pos)
#define TIMR_CR_MODE_Pos			2		//����ģʽ��0 ��ʱ��    1 ���벶��    2 ����Ƚ�
#define TIMR_CR_MODE_Msk			(0x03 << TIMR_CR_MODE_Pos)
#define TIMR_CR_ICEDGE_Pos			4		//���벶��ģʽ�£����������ı��أ�0 BOTH   1 ������   0 �½���
#define TIMR_CR_ICEDGE_Msk			(0x03 << TIMR_CR_ICEDGE_Pos)

#define TIMR_IE_TO_Pos				0		//Time out
#define TIMR_IE_TO_Msk				(0x01 << TIMR_IE_TO_Pos)
#define TIMR_IE_OC0_Pos				1		//����Ƚϣ���һ����ת��
#define TIMR_IE_OC0_Msk				(0x01 << TIMR_IE_OC0_Pos)
#define TIMR_IE_OC1_Pos				2		//����Ƚϣ��ڶ�����ת��
#define TIMR_IE_OC1_Msk				(0x01 << TIMR_IE_OC1_Pos)
#define TIMR_IE_ICR_Pos				3		//���벶���������ж�
#define TIMR_IE_ICR_Msk				(0x01 << TIMR_IE_ICR_Pos)
#define TIMR_IE_ICF_Pos				4		//���벶���½����ж�
#define TIMR_IE_ICF_Msk				(0x01 << TIMR_IE_ICF_Pos)

#define TIMR_IF_TO_Pos				0		//��ʱ�жϱ�־��д1����
#define TIMR_IF_TO_Msk				(0x01 << TIMR_IF_TO_Pos)
#define TIMR_IF_OC0_Pos				1
#define TIMR_IF_OC0_Msk				(0x01 << TIMR_IF_OC0_Pos)
#define TIMR_IF_OC1_Pos				2
#define TIMR_IF_OC1_Msk				(0x01 << TIMR_IF_OC1_Pos)
#define TIMR_IF_ICR_Pos				3
#define TIMR_IF_ICR_Msk				(0x01 << TIMR_IF_ICR_Pos)
#define TIMR_IF_ICF_Pos				4
#define TIMR_IF_ICF_Msk				(0x01 << TIMR_IF_ICF_Pos)

#define TIMR_OCCR_FORCELVL_Pos		0		//Force Levle��ǿ�������ƽ
#define TIMR_OCCR_FORCELVL_Msk		(0x01 << TIMR_OCCR_FORCELVL_Pos)
#define TIMR_OCCR_INITLVL_Pos		1		//Initial Level, ��ʼ�����ƽ
#define TIMR_OCCR_INITLVL_Msk		(0x01 << TIMR_OCCR_INITLVL_Pos)
#define TIMR_OCCR_FORCEEN_Pos		2		//Force Enable, ǿ�����ʹ��
#define TIMR_OCCR_FORCEEN_Msk		(0x01 << TIMR_OCCR_FORCEEN_Pos)


typedef struct {
	__IO uint32_t HALLIE;					//[0] HALL�ж�ʹ��
	
	uint32_t RESERVED;
	
	__IO uint32_t HALLIF;
	
	__IO uint32_t HALLEN;					//[0] HALL���ܿ���
	
	__IO uint32_t HALLDR;					//HALL���������ؽ�������������ֵ - ��ǰֵ������˼Ĵ���
	
	uint32_t RESERVED2[2];
	
	__IO uint32_t HALLSR;
	
	uint32_t RESERVED3[8];
	
	__IO uint32_t EN;
} TIMRG_TypeDef;

#define TIMRG_HALLIF_IN0_Pos		0		//HALL�����ź�0�����жϱ�־��д1����
#define TIMRG_HALLIF_IN0_Msk		(0x01 << TIMRG_HALLIF_IN0_Pos)
#define TIMRG_HALLIF_IN1_Pos		1
#define TIMRG_HALLIF_IN1_Msk		(0x01 << TIMRG_HALLIF_IN1_Pos)
#define TIMRG_HALLIF_IN2_Pos		2
#define TIMRG_HALLIF_IN2_Msk		(0x01 << TIMRG_HALLIF_IN2_Pos)

#define TIMRG_HALLSR_IN0_Pos		0		//HALL�����ź�0��ǰ��ƽ
#define TIMRG_HALLSR_IN0_Msk		(0x01 << TIMRG_HALLSR_IN0_Pos)
#define TIMRG_HALLSR_IN1_Pos		1
#define TIMRG_HALLSR_IN1_Msk		(0x01 << TIMRG_HALLSR_IN1_Pos)
#define TIMRG_HALLSR_IN2_Pos		2
#define TIMRG_HALLSR_IN2_Msk		(0x01 << TIMRG_HALLSR_IN2_Pos)

#define TIMRG_EN_TIMR0_Pos			0
#define TIMRG_EN_TIMR0_Msk			(0x01 << TIMRG_EN_TIMR0_Pos)
#define TIMRG_EN_TIMR1_Pos			1
#define TIMRG_EN_TIMR1_Msk			(0x01 << TIMRG_EN_TIMR1_Pos)
#define TIMRG_EN_TIMR2_Pos			2
#define TIMRG_EN_TIMR2_Msk			(0x01 << TIMRG_EN_TIMR2_Pos)
#define TIMRG_EN_TIMR3_Pos			3
#define TIMRG_EN_TIMR3_Msk			(0x01 << TIMRG_EN_TIMR3_Pos)
#define TIMRG_EN_TIMR4_Pos			4
#define TIMRG_EN_TIMR4_Msk			(0x01 << TIMRG_EN_TIMR4_Pos)
#define TIMRG_EN_TIMR5_Pos			5
#define TIMRG_EN_TIMR5_Msk			(0x01 << TIMRG_EN_TIMR5_Pos)
#define TIMRG_EN_TIMR6_Pos			6
#define TIMRG_EN_TIMR6_Msk			(0x01 << TIMRG_EN_TIMR6_Pos)
#define TIMRG_EN_TIMR7_Pos			7
#define TIMRG_EN_TIMR7_Msk			(0x01 << TIMRG_EN_TIMR7_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t CTRL;
	
	__IO uint32_t BAUD;
	
	__IO uint32_t FIFO;
	
	__IO uint32_t LINCR;
	
	union {
		__IO uint32_t CTSCR;
		
		__IO uint32_t RTSCR;
	};
	
	__IO uint32_t CFG;
	
	__IO uint32_t TOCR;						//Timeout Control Register
} UART_TypeDef;


#define UART_DATA_DATA_Pos			0
#define UART_DATA_DATA_Msk			(0x1FF << UART_DATA_DATA_Pos)
#define UART_DATA_VALID_Pos			9		//��DATA�ֶ�����Ч�Ľ�������ʱ����λӲ����1����ȡ���ݺ��Զ�����
#define UART_DATA_VALID_Msk			(0x01 << UART_DATA_VALID_Pos)
#define UART_DATA_PAERR_Pos			10		//Parity Error
#define UART_DATA_PAERR_Msk			(0x01 << UART_DATA_PAERR_Pos)

#define UART_CTRL_TXIDLE_Pos		0		//TX IDLE: 0 ���ڷ�������	1 ����״̬��û�����ݷ���
#define UART_CTRL_TXIDLE_Msk		(0x01 << UART_CTRL_TXIDLE_Pos)
#define UART_CTRL_TXFF_Pos		    1		//TX FIFO Full
#define UART_CTRL_TXFF_Msk		    (0x01 << UART_CTRL_TXFF_Pos)
#define UART_CTRL_TXIE_Pos			2		//TX �ж�ʹ��: 1 TX FF �����������趨����ʱ�����ж�
#define UART_CTRL_TXIE_Msk			(0x01 << UART_CTRL_TXIE_Pos)
#define UART_CTRL_RXNE_Pos			3		//RX FIFO Not Empty
#define UART_CTRL_RXNE_Msk			(0x01 << UART_CTRL_RXNE_Pos)
#define UART_CTRL_RXIE_Pos			4		//RX �ж�ʹ��: 1 RX FF �����ݴﵽ�趨����ʱ�����ж�
#define UART_CTRL_RXIE_Msk			(0x01 << UART_CTRL_RXIE_Pos)
#define UART_CTRL_RXOV_Pos			5		//RX FIFO Overflow��д1����
#define UART_CTRL_RXOV_Msk			(0x01 << UART_CTRL_RXOV_Pos)
#define UART_CTRL_TXDOIE_Pos		6		//TX Done �ж�ʹ�ܣ�����FIFO���ҷ��ͷ�����λ�Ĵ����ѽ����һλ���ͳ�ȥ
#define UART_CTRL_TXDOIE_Msk		(0x01 << UART_CTRL_TXDOIE_Pos)
#define UART_CTRL_EN_Pos			9
#define UART_CTRL_EN_Msk			(0x01 << UART_CTRL_EN_Pos)
#define UART_CTRL_LOOP_Pos			10
#define UART_CTRL_LOOP_Msk			(0x01 << UART_CTRL_LOOP_Pos)
#define UART_CTRL_TOIE_Pos			14		//TimeOut �ж�ʹ�ܣ����յ��ϸ��ַ��󣬳��� TOTIME/BAUDRAUD ��û�н��յ��µ�����
#define UART_CTRL_TOIE_Msk			(0x01 << UART_CTRL_TOIE_Pos)
#define UART_CTRL_BRKDET_Pos		15		//LIN Break Detect����⵽LIN Break����RX���ϼ�⵽����11λ�͵�ƽ
#define UART_CTRL_BRKDET_Msk		(0x01 << UART_CTRL_BRKDET_Pos)
#define UART_CTRL_BRKIE_Pos			16		//LIN Break Detect �ж�ʹ��
#define UART_CTRL_BRKIE_Msk			(0x01 << UART_CTRL_BRKIE_Pos)
#define UART_CTRL_GENBRK_Pos		17		//Generate LIN Break������LIN Break
#define UART_CTRL_GENBRK_Msk		(0x01 << UART_CTRL_GENBRK_Pos)
#define UART_CTRL_DATA9b_Pos		18		//1 9λ����λ    0 8λ����λ
#define UART_CTRL_DATA9b_Msk		(0x01 << UART_CTRL_DATA9b_Pos)
#define UART_CTRL_PARITY_Pos		19		//000 ��У��    001 ��У��   011 żУ��   101 �̶�Ϊ1    111 �̶�Ϊ0
#define UART_CTRL_PARITY_Msk		(0x07 << UART_CTRL_PARITY_Pos)
#define UART_CTRL_STOP2b_Pos		22		//1 2λֹͣλ    0 1λֹͣλ
#define UART_CTRL_STOP2b_Msk		(0x03 << UART_CTRL_STOP2b_Pos)

#define UART_BAUD_BAUD_Pos			0		//���ڲ����� = SYS_Freq/16/BAUD - 1
#define UART_BAUD_BAUD_Msk			(0x3FFF << UART_BAUD_BAUD_Pos)
#define UART_BAUD_TXD_Pos			14		//ͨ����λ��ֱ�Ӷ�ȡ����TXD�����ϵĵ�ƽ
#define UART_BAUD_TXD_Msk			(0x01 << UART_BAUD_TXD_Pos)
#define UART_BAUD_RXD_Pos			15		//ͨ����λ��ֱ�Ӷ�ȡ����RXD�����ϵĵ�ƽ
#define UART_BAUD_RXD_Msk			(0x01 << UART_BAUD_RXD_Pos)
#define UART_BAUD_RXTOIF_Pos		16		//����&��ʱ���жϱ�־ = RXIF | TOIF
#define UART_BAUD_RXTOIF_Msk		(0x01 << UART_BAUD_RXTOIF_Pos)
#define UART_BAUD_TXIF_Pos			17		//�����жϱ�־ = TXTHRF & TXIE
#define UART_BAUD_TXIF_Msk			(0x01 << UART_BAUD_TXIF_Pos)
#define UART_BAUD_BRKIF_Pos			18		//LIN Break Detect �жϱ�־����⵽LIN Breakʱ��BRKIE=1����λ��Ӳ����λ
#define UART_BAUD_BRKIF_Msk			(0x01 << UART_BAUD_BRKIF_Pos)
#define UART_BAUD_RXTHRF_Pos		19		//RX FIFO Threshold Flag��RX FIFO�����ݴﵽ�趨������RXLVL >  RXTHR��ʱӲ����1
#define UART_BAUD_RXTHRF_Msk		(0x01 << UART_BAUD_RXTHRF_Pos)
#define UART_BAUD_TXTHRF_Pos		20		//TX FIFO Threshold Flag��TX FIFO�����������趨������TXLVL <= TXTHR��ʱӲ����1
#define UART_BAUD_TXTHRF_Msk		(0x01 << UART_BAUD_TXTHRF_Pos)
#define UART_BAUD_TOIF_Pos			21		//TimeOut �жϱ�־������ TOTIME/BAUDRAUD ��û�н��յ��µ�����ʱ��TOIE=1����λ��Ӳ����λ
#define UART_BAUD_TOIF_Msk			(0x01 << UART_BAUD_TOIF_Pos)
#define UART_BAUD_RXIF_Pos			22		//�����жϱ�־ = RXTHRF & RXIE
#define UART_BAUD_RXIF_Msk			(0x01 << UART_BAUD_RXIF_Pos)
#define UART_BAUD_ABREN_Pos			23		//Auto Baudrate Enable��д1�����Զ�������У׼����ɺ��Զ�����
#define UART_BAUD_ABREN_Msk			(0x01 << UART_BAUD_ABREN_Pos)
#define UART_BAUD_ABRBIT_Pos		24		//Auto Baudrate Bit�����ڼ��㲨���ʵļ��λ����0 1λ��ͨ������ʼλ           ������㲨���ʣ�Ҫ���Ͷ˷���0xFF
											//                                             1 2λ��ͨ������ʼλ��1λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0xFE
											//                                             1 4λ��ͨ������ʼλ��3λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0xF8
											//                                             1 8λ��ͨ������ʼλ��7λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0x80
#define UART_BAUD_ABRBIT_Msk		(0x03 << UART_BAUD_ABRBIT_Pos)
#define UART_BAUD_ABRERR_Pos		26		//Auto Baudrate Error��0 �Զ�������У׼�ɹ�     1 �Զ�������У׼ʧ��
#define UART_BAUD_ABRERR_Msk		(0x01 << UART_BAUD_ABRERR_Pos)
#define UART_BAUD_TXDOIF_Pos		27		//TX Done �жϱ�־������FIFO���ҷ��ͷ�����λ�Ĵ����ѽ����һλ���ͳ�ȥ
#define UART_BAUD_TXDOIF_Msk		(0x01 << UART_BAUD_TXDOIF_Pos)
#define UART_BAUD_FRAC_Pos			28		//�����ʷ�ƵֵС������
#define UART_BAUD_FRAC_Msk			(0x0Fu<< UART_BAUD_FRAC_Pos)

#define UART_FIFO_RXLVL_Pos			0		//RX FIFO Level��RX FIFO ���ַ�����
#define UART_FIFO_RXLVL_Msk			(0xFF << UART_FIFO_RXLVL_Pos)
#define UART_FIFO_TXLVL_Pos			8		//TX FIFO Level��TX FIFO ���ַ�����
#define UART_FIFO_TXLVL_Msk			(0xFF << UART_FIFO_TXLVL_Pos)
#define UART_FIFO_RXTHR_Pos			16		//RX FIFO Threshold��RX�жϴ������ޣ��ж�ʹ��ʱ RXLVL >  RXTHR ����RX�ж�
#define UART_FIFO_RXTHR_Msk			(0xFF << UART_FIFO_RXTHR_Pos)
#define UART_FIFO_TXTHR_Pos			24		//TX FIFO Threshold��TX�жϴ������ޣ��ж�ʹ��ʱ TXLVL <= TXTHR ����TX�ж�
#define UART_FIFO_TXTHR_Msk			(0xFFu<< UART_FIFO_TXTHR_Pos)

#define UART_LINCR_BRKDETIE_Pos		0		//��⵽LIN Break�ж�ʹ��
#define UART_LINCR_BRKDETIE_Msk		(0x01 << UART_LINCR_BRKDETIE_Pos)
#define UART_LINCR_BRKDETIF_Pos		1		//��⵽LIN Break�ж�״̬
#define UART_LINCR_BRKDETIF_Msk		(0x01 << UART_LINCR_BRKDETIF_Pos)
#define UART_LINCR_GENBRKIE_Pos		2		//����LIN Break����ж�ʹ��
#define UART_LINCR_GENBRKIE_Msk		(0x01 << UART_LINCR_GENBRKIE_Pos)
#define UART_LINCR_GENBRKIF_Pos		3		//����LIN Break����ж�״̬
#define UART_LINCR_GENBRKIF_Msk		(0x01 << UART_LINCR_GENBRKIF_Pos)
#define UART_LINCR_GENBRK_Pos		4		//����LIN Break����������Զ�����
#define UART_LINCR_GENBRK_Msk		(0x01 << UART_LINCR_GENBRK_Pos)

#define UART_CTSCR_EN_Pos			0		//CTS����ʹ��
#define UART_CTSCR_EN_Msk			(0x01 << UART_CTSCR_EN_Pos)
#define UART_CTSCR_POL_Pos			2		//CTS�źż��ԣ�0 ����Ч��CTS����Ϊ�ͱ�ʾ���Է�������
#define UART_CTSCR_POL_Msk			(0x01 << UART_CTSCR_POL_Pos)
#define UART_CTSCR_STAT_Pos			7		//CTS�źŵĵ�ǰ״̬
#define UART_CTSCR_STAT_Msk			(0x01 << UART_CTSCR_STAT_Pos)

#define UART_RTSCR_EN_Pos			1		//RTS����ʹ��
#define UART_RTSCR_EN_Msk			(0x01 << UART_RTSCR_EN_Pos)
#define UART_RTSCR_POL_Pos			3		//RTS�źż���    0 ����Ч��RTS����Ϊ�ͱ�ʾ���Խ�������
#define UART_RTSCR_POL_Msk			(0x01 << UART_RTSCR_POL_Pos)
#define UART_RTSCR_THR_Pos			4		//RTS���صĴ�����ֵ    0 1�ֽ�    1 2�ֽ�    2 4�ֽ�    3 6�ֽ�
#define UART_RTSCR_THR_Msk			(0x07 << UART_RTSCR_THR_Pos)
#define UART_RTSCR_STAT_Pos			8		//RTS�źŵĵ�ǰ״̬
#define UART_RTSCR_STAT_Msk			(0x01 << UART_RTSCR_STAT_Pos)

#define UART_CFG_RXEN_Pos			0		//RX Enable
#define UART_CFG_RXEN_Msk			(0x01 << UART_CFG_RXEN_Pos)
#define UART_CFG_MSBF_Pos			1		//���շ���MSB First
#define UART_CFG_MSBF_Msk			(0x01 << UART_CFG_MSBF_Pos)
#define UART_CFG_BRKTXLEN_Pos		2		//1��ʾ1bit���Դ����ƣ�Ĭ��ֵ13
#define UART_CFG_BRKTXLEN_Msk		(0x0F << UART_CFG_BRKTXLEN_Pos)
#define UART_CFG_BRKRXLEN_Pos		6		//0��ʾ1bit���Դ����ƣ�Ĭ��ֵ12
#define UART_CFG_BRKRXLEN_Msk		(0x0F << UART_CFG_BRKRXLEN_Pos)
#define UART_CFG_RXINV_Pos			10		//���յ�ƽ��ת
#define UART_CFG_RXINV_Msk			(0x01 << UART_CFG_RXINV_Pos)
#define UART_CFG_TXINV_Pos			11		//���͵�ƽ��ת
#define UART_CFG_TXINV_Msk			(0x01 << UART_CFG_TXINV_Pos)

#define UART_TOCR_TIME_Pos			0		//��ʱʱ�䳤�ȣ���λΪ 10/BAUDRATE ��
#define UART_TOCR_TIME_Msk			(0xFFF<< UART_TOCR_TIME_Pos)
#define UART_TOCR_MODE_Pos			12		//0 ֻ�е�FIFO������ʱ�Ŵ�����ʱ�ж�    1 ��ʹFIFO��û����Ҳ�ɴ�����ʱ�ж�
#define UART_TOCR_MODE_Msk			(0x01 << UART_TOCR_MODE_Pos)
#define UART_TOCR_IFCLR_Pos			13		//TO Interrupt Flag Clear��д1�����ʱ�жϱ�־
#define UART_TOCR_IFCLR_Msk			(0x01 << UART_TOCR_IFCLR_Pos)




typedef struct {
	__IO uint32_t CTRL;

	__IO uint32_t DATA;

	__IO uint32_t STAT;

	__IO uint32_t IE;

	__IO uint32_t IF;
} SPI_TypeDef;


#define SPI_CTRL_CLKDIV_Pos			0		//Clock Divider, SPI����ʱ�� = SYS_Freq/pow(2, CLKDIV+2)
#define SPI_CTRL_CLKDIV_Msk			(0x07 << SPI_CTRL_CLKDIV_Pos)
#define SPI_CTRL_EN_Pos				3
#define SPI_CTRL_EN_Msk				(0x01 << SPI_CTRL_EN_Pos)
#define SPI_CTRL_SIZE_Pos			4		//Data Size Select, ȡֵ3--15����ʾ4--16λ
#define SPI_CTRL_SIZE_Msk			(0x0F << SPI_CTRL_SIZE_Pos)
#define SPI_CTRL_CPHA_Pos			8		//0 ��SCLK�ĵ�һ�������ز�������	1 ��SCLK�ĵڶ��������ز�������
#define SPI_CTRL_CPHA_Msk			(0x01 << SPI_CTRL_CPHA_Pos)
#define SPI_CTRL_CPOL_Pos			9		//0 ����״̬��SCLKΪ�͵�ƽ		  1 ����״̬��SCLKΪ�ߵ�ƽ
#define SPI_CTRL_CPOL_Msk			(0x01 << SPI_CTRL_CPOL_Pos)
#define SPI_CTRL_FFS_Pos			10		//Frame Format Select, 0 SPI	1 TI SSI	2 I2S	3 SPI Flash
#define SPI_CTRL_FFS_Msk			(0x03 << SPI_CTRL_FFS_Pos)
#define SPI_CTRL_MSTR_Pos			12		//Master, 1 ��ģʽ	0 ��ģʽ
#define SPI_CTRL_MSTR_Msk			(0x01 << SPI_CTRL_MSTR_Pos)
#define SPI_CTRL_FAST_Pos			13		//1 SPI����ʱ�� = SYS_Freq/2    0 SPI����ʱ����SPI->CTRL.CLKDIV����
#define SPI_CTRL_FAST_Msk			(0x01 << SPI_CTRL_FAST_Pos)
#define SPI_CTRL_DMATXEN_Pos		14		//1 ͨ��DMAдFIFO    0 ͨ��MCUдFIFO
#define SPI_CTRL_DMATXEN_Msk		(0x01 << SPI_CTRL_DMATXEN_Pos)
#define SPI_CTRL_DMARXEN_Pos		15		//1 ͨ��DMA��FIFO    0 ͨ��MCU��FIFO
#define SPI_CTRL_DMARXEN_Msk		(0x01 << SPI_CTRL_DMARXEN_Pos)
#define SPI_CTRL_FILTE_Pos			16		//1 ��SPI�����źŽ���ȥ������    0 ��SPI�����źŲ�����ȥ������
#define SPI_CTRL_FILTE_Msk			(0x01 << SPI_CTRL_FILTE_Pos)
#define SPI_CTRL_SSN_H_Pos			17		//0 ���������SSNʼ��Ϊ0    	 1 ���������ÿ�ַ�֮��ὫSSN���߰��SCLK����
#define SPI_CTRL_SSN_H_Msk			(0x01 << SPI_CTRL_SSN_H_Pos)
#define SPI_CTRL_RFTHR_Pos			18		//RX FIFO Threshold��0 ����FIFO��������1������   ...   7 ����FIFO��������8������
#define SPI_CTRL_RFTHR_Msk			(0x07 << SPI_CTRL_RFTHR_Pos)
#define SPI_CTRL_TFTHR_Pos			21		//TX FIFO Threshold��0 ����FIFO��������0������   ...   7 ����FIFO��������7������
#define SPI_CTRL_TFTHR_Msk			(0x07 << SPI_CTRL_TFTHR_Pos)
#define SPI_CTRL_RFCLR_Pos			24		//RX FIFO Clear
#define SPI_CTRL_RFCLR_Msk			(0x01 << SPI_CTRL_RFCLR_Pos)
#define SPI_CTRL_TFCLR_Pos			25		//TX FIFO Clear
#define SPI_CTRL_TFCLR_Msk			(0x01 << SPI_CTRL_TFCLR_Pos)
#define SPI_CTRL_LSBF_Pos			28		//LSB Fisrt
#define SPI_CTRL_LSBF_Msk			(0x01 << SPI_CTRL_LSBF_Pos)

#define SPI_STAT_WTC_Pos			0		//Word Transmit Complete��ÿ�������һ����������Ӳ����1�����д1����
#define SPI_STAT_WTC_Msk			(0x01 << SPI_STAT_WTC_Pos)
#define SPI_STAT_TFE_Pos			1		//����FIFO Empty
#define SPI_STAT_TFE_Msk			(0x01 << SPI_STAT_TFE_Pos)
#define SPI_STAT_TFNF_Pos			2		//����FIFO Not Full
#define SPI_STAT_TFNF_Msk			(0x01 << SPI_STAT_TFNF_Pos)
#define SPI_STAT_RFNE_Pos			3		//����FIFO Not Empty
#define SPI_STAT_RFNE_Msk			(0x01 << SPI_STAT_RFNE_Pos)
#define SPI_STAT_RFF_Pos			4		//����FIFO Full
#define SPI_STAT_RFF_Msk			(0x01 << SPI_STAT_RFF_Pos)
#define SPI_STAT_RFOV_Pos			5		//����FIFO Overflow
#define SPI_STAT_RFOV_Msk			(0x01 << SPI_STAT_RFOV_Pos)
#define SPI_STAT_TFLVL_Pos			6		//����FIFO�����ݸ����� 0 TFNF=0ʱ��ʾFIFO����8�����ݣ�TFNF=1ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_TFLVL_Msk			(0x07 << SPI_STAT_TFLVL_Pos)
#define SPI_STAT_RFLVL_Pos			9		//����FIFO�����ݸ����� 0 RFF =1ʱ��ʾFIFO����8�����ݣ�RFF =0ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_RFLVL_Msk			(0x07 << SPI_STAT_RFLVL_Pos)
#define SPI_STAT_BUSY_Pos			15
#define SPI_STAT_BUSY_Msk			(0x01 << SPI_STAT_BUSY_Pos)

#define SPI_IE_RFOV_Pos				0
#define SPI_IE_RFOV_Msk				(0x01 << SPI_IE_RFOV_Pos)
#define SPI_IE_RFF_Pos				1
#define SPI_IE_RFF_Msk				(0x01 << SPI_IE_RFF_Pos)
#define SPI_IE_RFHF_Pos				2
#define SPI_IE_RFHF_Msk				(0x01 << SPI_IE_RFHF_Pos)
#define SPI_IE_TFE_Pos				3
#define SPI_IE_TFE_Msk				(0x01 << SPI_IE_TFE_Pos)
#define SPI_IE_TFHF_Pos				4		//����FIFO�����ݸ�������4
#define SPI_IE_TFHF_Msk				(0x01 << SPI_IE_TFHF_Pos)
#define SPI_IE_RFTHR_Pos			5		//����FIFO�����ݸ�������CTRL.RFTHR�趨ֵ�ж�ʹ��
#define SPI_IE_RFTHR_Msk			(0x01 << SPI_IE_RFTHR_Pos)
#define SPI_IE_TFTHR_Pos			6		//����FIFO�����ݸ���С��CTRL.TFTHR�趨ֵ�ж�ʹ��
#define SPI_IE_TFTHR_Msk			(0x01 << SPI_IE_TFTHR_Pos)
#define SPI_IE_WTC_Pos				8		//Word Transmit Complete
#define SPI_IE_WTC_Msk				(0x01 << SPI_IE_WTC_Pos)
#define SPI_IE_FTC_Pos				9		//Frame Transmit Complete
#define SPI_IE_FTC_Msk				(0x01 << SPI_IE_FTC_Pos)
#define SPI_IE_SSFALL_Pos			10		//Slave Select Fall Edge
#define SPI_IE_SSFALL_Msk			(0x01 << SPI_IE_SSFALL_Pos)
#define SPI_IE_SSRISE_Pos			11		//Slave Select Rise Edge
#define SPI_IE_SSRISE_Msk			(0x01 << SPI_IE_SSRISE_Pos)

#define SPI_IF_RFOV_Pos				0		//д1����
#define SPI_IF_RFOV_Msk				(0x01 << SPI_IF_RFOV_Pos)
#define SPI_IF_RFF_Pos				1		//д1����
#define SPI_IF_RFF_Msk				(0x01 << SPI_IF_RFF_Pos)
#define SPI_IF_RFHF_Pos				2		//д1����
#define SPI_IF_RFHF_Msk				(0x01 << SPI_IF_RFHF_Pos)
#define SPI_IF_TFE_Pos				3		//д1����
#define SPI_IF_TFE_Msk				(0x01 << SPI_IF_TFE_Pos)
#define SPI_IF_TFHF_Pos				4		//д1����
#define SPI_IF_TFHF_Msk				(0x01 << SPI_IF_TFHF_Pos)
#define SPI_IF_RFTHR_Pos			5		//д1����
#define SPI_IF_RFTHR_Msk			(0x01 << SPI_IF_RFTHR_Pos)
#define SPI_IF_TFTHR_Pos			6		//д1����
#define SPI_IF_TFTHR_Msk			(0x01 << SPI_IF_TFTHR_Pos)
#define SPI_IF_WTC_Pos				8		//Word Transmit Complete��ÿ�������һ����������Ӳ����1
#define SPI_IF_WTC_Msk				(0x01 << SPI_IF_WTC_Pos)
#define SPI_IF_FTC_Pos				9		//Frame Transmit Complete��WTC��λʱ��TX FIFO�ǿյģ���FTC��λ
#define SPI_IF_FTC_Msk				(0x01 << SPI_IF_FTC_Pos)
#define SPI_IF_SSFALL_Pos			10
#define SPI_IF_SSFALL_Msk			(0x01 << SPI_IF_SSFALL_Pos)
#define SPI_IF_SSRISE_Pos			11
#define SPI_IF_SSRISE_Msk			(0x01 << SPI_IF_SSRISE_Pos)


typedef struct {
	__IO uint32_t CR;

	__IO uint32_t SR;

	__IO uint32_t TR;						//Transfer Register

	__IO uint32_t RXDATA;
	
	__IO uint32_t TXDATA;
	
	__IO uint32_t IF;
	
	__IO uint32_t IE;
	
		 uint32_t RESERVED1;
	
	__IO uint32_t MCR;						//Master Control Register
	
	__IO uint32_t CLK;
	
		 uint32_t RESERVED2[2];
	
	__IO uint32_t SCR;						//Slave Control Register
	
	__IO uint32_t SADDR;
} I2C_TypeDef;


#define I2C_CR_EN_Pos				0	
#define I2C_CR_EN_Msk				(0x01 << I2C_CR_EN_Pos)
#define I2C_CR_MASTER_Pos			1		//1 Master   0 Slave
#define I2C_CR_MASTER_Msk			(0x01 << I2C_CR_MASTER_Pos)
#define I2C_CR_HS_Pos				2		//1 High-Speed mode    0 Standard-mode or Fast-mode
#define I2C_CR_HS_Msk				(0x01 << I2C_CR_HS_Pos)
#define I2C_CR_DNF_Pos				3		//Digital Noise Filter, ��ȵ��� DNF+1 ���ĵ�ƽ����Ϊ��ë��
#define I2C_CR_DNF_Msk				(0x0F << I2C_CR_DNF_Pos)

#define I2C_SR_BUSY_Pos				0
#define I2C_SR_BUSY_Msk				(0x01 << I2C_SR_BUSY_Pos)
#define I2C_SR_SCL_Pos				1		//SCL Line Level
#define I2C_SR_SCL_Msk				(0x01 << I2C_SR_SCL_Pos)
#define I2C_SR_SDA_Pos				2		//SDA Line Level
#define I2C_SR_SDA_Msk				(0x01 << I2C_SR_SDA_Pos)

#define I2C_TR_TXACK_Pos			0		//��Ϊ����ʱ������ACKλ�ĵ�ƽֵ
#define I2C_TR_TXACK_Msk			(0x01 << I2C_TR_TXACK_Pos)
#define I2C_TR_RXACK_Pos			1		//��Ϊ����ʱ�����յ���ACKλ��ƽֵ
#define I2C_TR_RXACK_Msk			(0x01 << I2C_TR_RXACK_Pos)
#define I2C_TR_TXCLR_Pos			2		//TX Data Clear, �Զ�����
#define I2C_TR_TXCLR_Msk			(0x01 << I2C_TR_TXCLR_Pos)
#define I2C_TR_SLVACT_Pos			8		//Slave Active, �ӻ�ģʽ�±�ѡ��ʱ��λ
#define I2C_TR_SLVACT_Msk			(0x01 << I2C_TR_SLVACT_Pos)
#define I2C_TR_SLVRD_Pos			9		//Slave Read mode���ӻ�ģʽ�½��յ�������ʱ��λ
#define I2C_TR_SLVRD_Msk			(0x01 << I2C_TR_SLVRD_Pos)
#define I2C_TR_SLVWR_Pos			10		//Slave Write mode���ӻ�ģʽ�½��յ�д����ʱ��λ
#define I2C_TR_SLVWR_Msk			(0x01 << I2C_TR_SLVWR_Pos)
#define I2C_TR_SLVSTR_Pos			11		//Slave clock stretching
#define I2C_TR_SLVSTR_Msk			(0x01 << I2C_TR_SLVSTR_Pos)
#define I2C_TR_SLVRDS_Pos			12		//Slave RXDATA Status, 0 ��   1 ���յ���ַ   2 ���յ�����   3 ���յ�Master Code
#define I2C_TR_SLVRDS_Msk			(0x03 << I2C_TR_SLVRDS_Pos)

#define I2C_IF_TXE_Pos				0		//TX Empty��дTXDATA�����λ
#define I2C_IF_TXE_Msk				(0x01 << I2C_IF_TXE_Pos)
#define I2C_IF_RXNE_Pos				1		//RX Not Empty����RXDATA�����λ
#define I2C_IF_RXNE_Msk				(0x01 << I2C_IF_RXNE_Pos)
#define I2C_IF_RXOV_Pos				2		//RX Overflow��д1����
#define I2C_IF_RXOV_Msk				(0x01 << I2C_IF_RXOV_Pos)
#define I2C_IF_TXDONE_Pos			3		//TX Done��д1����
#define I2C_IF_TXDONE_Msk			(0x01 << I2C_IF_TXDONE_Pos)
#define I2C_IF_RXDONE_Pos			4		//RX Done��д1����
#define I2C_IF_RXDONE_Msk			(0x01 << I2C_IF_RXDONE_Pos)
#define I2C_IF_RXSTA_Pos			8		//�ӻ����յ���ʼλ��д1����
#define I2C_IF_RXSTA_Msk			(0x01 << I2C_IF_RXSTA_Pos)
#define I2C_IF_RXSTO_Pos			9		//�ӻ����յ�ֹͣλ��д1����
#define I2C_IF_RXSTO_Msk			(0x01 << I2C_IF_RXSTO_Pos)
#define I2C_IF_AL_Pos				16		//�����ٲö�ʧ���ߣ�д1����
#define I2C_IF_AL_Msk				(0x01 << I2C_IF_AL_Pos)
#define I2C_IF_MLTO_Pos				17		//Master SCL Low Timeout��д1����
#define I2C_IF_MLTO_Msk				(0x01 << I2C_IF_MLTO_Pos)

#define I2C_IE_TXE_Pos				0
#define I2C_IE_TXE_Msk				(0x01 << I2C_IE_TXE_Pos)
#define I2C_IE_RXNE_Pos				1
#define I2C_IE_RXNE_Msk				(0x01 << I2C_IE_RXNE_Pos)
#define I2C_IE_RXOV_Pos				2
#define I2C_IE_RXOV_Msk				(0x01 << I2C_IE_RXOV_Pos)
#define I2C_IE_TXDONE_Pos			3
#define I2C_IE_TXDONE_Msk			(0x01 << I2C_IE_TXDONE_Pos)
#define I2C_IE_RXDONE_Pos			4
#define I2C_IE_RXDONE_Msk			(0x01 << I2C_IE_RXDONE_Pos)
#define I2C_IE_RXSTA_Pos			8
#define I2C_IE_RXSTA_Msk			(0x01 << I2C_IE_RXSTA_Pos)
#define I2C_IE_RXSTO_Pos			9
#define I2C_IE_RXSTO_Msk			(0x01 << I2C_IE_RXSTO_Pos)
#define I2C_IE_AL_Pos				16
#define I2C_IE_AL_Msk				(0x01 << I2C_IE_AL_Pos)
#define I2C_IE_MLTO_Pos				17
#define I2C_IE_MLTO_Msk				(0x01 << I2C_IE_MLTO_Pos)

#define I2C_MCR_STA_Pos				0		//д1������ʼλ����ɺ��Զ�����
#define I2C_MCR_STA_Msk				(0x01 << I2C_MCR_STA_Pos)
#define I2C_MCR_RD_Pos				1
#define I2C_MCR_RD_Msk				(0x01 << I2C_MCR_RD_Pos)
#define I2C_MCR_WR_Pos				2
#define I2C_MCR_WR_Msk				(0x01 << I2C_MCR_WR_Pos)
#define I2C_MCR_STO_Pos				3		//д1����ֹͣλ����ɺ��Զ�����
#define I2C_MCR_STO_Msk				(0x01 << I2C_MCR_STO_Pos)

#define I2C_CLK_SCLL_Pos			0		//SCL Low Time
#define I2C_CLK_SCLL_Msk			(0xFF << I2C_CLK_SCLL_Pos)
#define I2C_CLK_SCLH_Pos			8		//SCL High Time
#define I2C_CLK_SCLH_Msk			(0xFF << I2C_CLK_SCLH_Pos)
#define I2C_CLK_DIV_Pos				16
#define I2C_CLK_DIV_Msk				(0xFF << I2C_CLK_DIV_Pos)
#define I2C_CLK_SDAH_Pos			24		//SDA Hold Time
#define I2C_CLK_SDAH_Msk			(0x0F << I2C_CLK_SDAH_Pos)

#define I2C_SCR_ADDR10_Pos			0		//1 10λ��ַ    0 7λ��ַ
#define I2C_SCR_ADDR10_Msk			(0x01 << I2C_SCR_ADDR10_Pos)
#define I2C_SCR_MCDE_Pos			1		//Master Code Detect Enable
#define I2C_SCR_MCDE_Msk			(0x01 << I2C_SCR_MCDE_Pos)
#define I2C_SCR_STRE_Pos			2		//Clock Stretching Enable
#define I2C_SCR_STRE_Msk			(0x01 << I2C_SCR_STRE_Pos)
#define I2C_SCR_ASDS_Pos			3		//Adaptive Stretching Data Setup
#define I2C_SCR_ASDS_Msk			(0x01 << I2C_SCR_ASDS_Pos)

#define I2C_SADDR_ADDR7_Pos			1		//7λ��ַģʽ�µĵ�ַ
#define I2C_SADDR_ADDR7_Msk			(0x7F << I2C_SADDR_ADDR7_Pos)
#define I2C_SADDR_ADDR10_Pos		0		//10λ��ַģʽ�µĵ�ַ
#define I2C_SADDR_ADDR10_Msk		(0x3FF<< I2C_SADDR_ADDR10_Pos)
#define I2C_SADDR_MASK7_Pos			17		//7λ��ַģʽ�µĵ�ַ���룬ADDR7 & (~MASK7) ������յ�ַ�Ƚ�
#define I2C_SADDR_MASK7_Msk			(0x7F << I2C_SADDR_MASK7_Pos)
#define I2C_SADDR_MASK10_Pos		16		//10λ��ַģʽ�µĵ�ַ���룬ֻ�����8λ
#define I2C_SADDR_MASK10_Msk		(0xFF << I2C_SADDR_MASK10_Pos)




typedef struct {
	__IO uint32_t CTRL;
	
	__IO uint32_t START;
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	struct {
		__IO uint32_t STAT;
		
		__IO uint32_t DATA;
		
		uint32_t RESERVED[2];
	} CH[12];
	
	__IO uint32_t CHSEL;					//ADC->CTRL.CH = PWM_Trigger ? ADC->CHSEL.PWM : ADC->CHSEL.SW
	
		 uint32_t RESERVED[47];
	
	__IO uint32_t FFSTAT;				   	//FIFO STAT
    
	__IO uint32_t FFDATA;				   	//FIFO DATA
	
	     uint32_t RESERVED2[2];
	
	__IO uint32_t CTRL1;
	
	__IO uint32_t CTRL2;
	
	__IO uint32_t CTRL3;
	
		 uint32_t RESERVED3;
	
	__IO uint32_t TRGMSK;					//��Ӧλ��1������Ӧͨ������ADC���ܱ�����
	
		 uint32_t RESERVED4[16];
		 
	__IO uint32_t CALIBSET;
	
	__IO uint32_t CALIBEN;
} ADC_TypeDef;


#define ADC_CTRL_CH0_Pos			0		//ͨ��ѡ��
#define ADC_CTRL_CH0_Msk			(0x01 << ADC_CTRL_CH0_Pos)
#define ADC_CTRL_CH1_Pos			1
#define ADC_CTRL_CH1_Msk			(0x01 << ADC_CTRL_CH1_Pos)
#define ADC_CTRL_CH2_Pos			2
#define ADC_CTRL_CH2_Msk			(0x01 << ADC_CTRL_CH2_Pos)
#define ADC_CTRL_CH3_Pos			3
#define ADC_CTRL_CH3_Msk			(0x01 << ADC_CTRL_CH3_Pos)
#define ADC_CTRL_CH4_Pos			4
#define ADC_CTRL_CH4_Msk			(0x01 << ADC_CTRL_CH4_Pos)
#define ADC_CTRL_CH5_Pos			5
#define ADC_CTRL_CH5_Msk			(0x01 << ADC_CTRL_CH5_Pos)
#define ADC_CTRL_CH6_Pos			6
#define ADC_CTRL_CH6_Msk			(0x01 << ADC_CTRL_CH6_Pos)
#define ADC_CTRL_CH7_Pos			7
#define ADC_CTRL_CH7_Msk			(0x01 << ADC_CTRL_CH7_Pos)
#define ADC_CTRL_CH8_Pos			8
#define ADC_CTRL_CH8_Msk			(0x01 << ADC_CTRL_CH8_Pos)
#define ADC_CTRL_CH9_Pos			9
#define ADC_CTRL_CH9_Msk			(0x01 << ADC_CTRL_CH9_Pos)
#define ADC_CTRL_CH10_Pos			10
#define ADC_CTRL_CH10_Msk			(0x01 << ADC_CTRL_CH10_Pos)
#define ADC_CTRL_CH11_Pos			11
#define ADC_CTRL_CH11_Msk			(0x01 << ADC_CTRL_CH11_Pos)
#define ADC_CTRL_EN_Pos				12
#define ADC_CTRL_EN_Msk				(0x01 << ADC_CTRL_EN_Pos)
#define ADC_CTRL_CONT_Pos			13		//Continuous conversion��ֻ���������ģʽ����Ч��0 ����ת����ת����ɺ�STARTλ�Զ����ֹͣת��
#define ADC_CTRL_CONT_Msk			(0x01 << ADC_CTRL_CONT_Pos)							//   1 ����ת����������һֱ������ת����ֱ��������STARTλ
#define ADC_CTRL_TRIG_Pos			14		//ת��������ʽ��0 �������ת��	  1 PWM����	  2 TIMR2����	 3 TIMR3����
#define ADC_CTRL_TRIG_Msk			(0x07 << ADC_CTRL_TRIG_Pos)
#define ADC_CTRL_DMAEN_Pos			17		//ֻ����ֻѡ��һ��ͨ����ʱ��ʹ��DMA���ܣ���RES2FF������1��DMA CH1 ͨ����ȡFFDATA�Ĵ�����ȡת�����
#define ADC_CTRL_DMAEN_Msk			(0x01 << ADC_CTRL_DMAEN_Pos)
#define ADC_CTRL_RES2FF_Pos			18		//Result to FIFO	1 ת���������FIFO	 0 ת�����������ӦCH��DATA�Ĵ���
#define ADC_CTRL_RES2FF_Msk			(0x01 << ADC_CTRL_RES2FF_Pos)
#define ADC_CTRL_FFCLR_Pos			19		//FIFO Clear
#define ADC_CTRL_FFCLR_Msk			(0x01 << ADC_CTRL_FFCLR_Pos)
#define ADC_CTRL_RST_Pos			20
#define ADC_CTRL_RST_Msk			(0x01 << ADC_CTRL_RST_Pos)
#define ADC_CTRL_AVG_Pos			21		//0 1�β���	  1 2�β���ȡƽ��ֵ	  3 4�β���ȡƽ��ֵ	  7 8�β���ȡƽ��ֵ	  15 16�β���ȡƽ��ֵ
#define ADC_CTRL_AVG_Msk			(0x0F << ADC_CTRL_AVG_Pos)

#define ADC_START_GO_Pos			0		//�������ģʽ�£�д1����ADC������ת�����ڵ���ģʽ��ת����ɺ�Ӳ���Զ����㣬��ɨ��ģʽ�±������д0ֹͣADCת��
#define ADC_START_GO_Msk			(0x01 << ADC_START_GO_Pos)
#define ADC_START_BUSY_Pos			4
#define ADC_START_BUSY_Msk			(0x01 << ADC_START_BUSY_Pos)

#define ADC_IE_CH0EOC_Pos			0		//End Of Convertion
#define ADC_IE_CH0EOC_Msk			(0x01 << ADC_IE_CH0EOC_Pos)
#define ADC_IE_CH0OVF_Pos			1		//Overflow
#define ADC_IE_CH0OVF_Msk			(0x01 << ADC_IE_CH0OVF_Pos)
#define ADC_IE_CH1EOC_Pos			2
#define ADC_IE_CH1EOC_Msk			(0x01 << ADC_IE_CH1EOC_Pos)
#define ADC_IE_CH1OVF_Pos			3
#define ADC_IE_CH1OVF_Msk			(0x01 << ADC_IE_CH1OVF_Pos)
#define ADC_IE_CH2EOC_Pos			4
#define ADC_IE_CH2EOC_Msk			(0x01 << ADC_IE_CH2EOC_Pos)
#define ADC_IE_CH2OVF_Pos			5
#define ADC_IE_CH2OVF_Msk			(0x01 << ADC_IE_CH2OVF_Pos)
#define ADC_IE_CH3EOC_Pos			6
#define ADC_IE_CH3EOC_Msk			(0x01 << ADC_IE_CH3EOC_Pos)
#define ADC_IE_CH3OVF_Pos			7
#define ADC_IE_CH3OVF_Msk			(0x01 << ADC_IE_CH3OVF_Pos)
#define ADC_IE_CH4EOC_Pos			8
#define ADC_IE_CH4EOC_Msk			(0x01 << ADC_IE_CH4EOC_Pos)
#define ADC_IE_CH4OVF_Pos			9
#define ADC_IE_CH4OVF_Msk			(0x01 << ADC_IE_CH4OVF_Pos)
#define ADC_IE_CH5EOC_Pos			10
#define ADC_IE_CH5EOC_Msk			(0x01 << ADC_IE_CH5EOC_Pos)
#define ADC_IE_CH5OVF_Pos			11
#define ADC_IE_CH5OVF_Msk			(0x01 << ADC_IE_CH5OVF_Pos)
#define ADC_IE_CH6EOC_Pos			12
#define ADC_IE_CH6EOC_Msk			(0x01 << ADC_IE_CH6EOC_Pos)
#define ADC_IE_CH6OVF_Pos			13
#define ADC_IE_CH6OVF_Msk			(0x01 << ADC_IE_CH6OVF_Pos)
#define ADC_IE_CH7EOC_Pos			14
#define ADC_IE_CH7EOC_Msk			(0x01 << ADC_IE_CH7EOC_Pos)
#define ADC_IE_CH7OVF_Pos			15
#define ADC_IE_CH7OVF_Msk			(0x01 << ADC_IE_CH7OVF_Pos)
#define ADC_IE_CH8EOC_Pos			16
#define ADC_IE_CH8EOC_Msk			(0x01 << ADC_IE_CH8EOC_Pos)
#define ADC_IE_CH8OVF_Pos			17
#define ADC_IE_CH8OVF_Msk			(0x01 << ADC_IE_CH8OVF_Pos)
#define ADC_IE_CH9EOC_Pos			18
#define ADC_IE_CH9EOC_Msk			(0x01 << ADC_IE_CH9EOC_Pos)
#define ADC_IE_CH9OVF_Pos			19
#define ADC_IE_CH9OVF_Msk			(0x01 << ADC_IE_CH9OVF_Pos)
#define ADC_IE_CH10EOC_Pos			20
#define ADC_IE_CH10EOC_Msk			(0x01 << ADC_IE_CH10EOC_Pos)
#define ADC_IE_CH10OVF_Pos			21
#define ADC_IE_CH10OVF_Msk			(0x01 << ADC_IE_CH10OVF_Pos)
#define ADC_IE_CH11EOC_Pos			22
#define ADC_IE_CH11EOC_Msk			(0x01 << ADC_IE_CH11EOC_Pos)
#define ADC_IE_CH11OVF_Pos			23
#define ADC_IE_CH11OVF_Msk			(0x01 << ADC_IE_CH11OVF_Pos)
#define ADC_IE_FIFOOV_Pos			24
#define ADC_IE_FIFOOV_Msk			(0x01 << ADC_IE_FIFOOV_Pos)
#define ADC_IE_FIFOHF_Pos			25
#define ADC_IE_FIFOHF_Msk			(0x01 << ADC_IE_FIFOHF_Pos)
#define ADC_IE_FIFOF_Pos			26
#define ADC_IE_FIFOF_Msk			(0x01 << ADC_IE_FIFOF_Pos)

#define ADC_IF_CH0EOC_Pos			0		//д1����
#define ADC_IF_CH0EOC_Msk			(0x01 << ADC_IF_CH0EOC_Pos)
#define ADC_IF_CH0OVF_Pos			1
#define ADC_IF_CH0OVF_Msk			(0x01 << ADC_IF_CH0OVF_Pos)
#define ADC_IF_CH1EOC_Pos			2
#define ADC_IF_CH1EOC_Msk			(0x01 << ADC_IF_CH1EOC_Pos)
#define ADC_IF_CH1OVF_Pos			3
#define ADC_IF_CH1OVF_Msk			(0x01 << ADC_IF_CH1OVF_Pos)
#define ADC_IF_CH2EOC_Pos			4
#define ADC_IF_CH2EOC_Msk			(0x01 << ADC_IF_CH2EOC_Pos)
#define ADC_IF_CH2OVF_Pos			5
#define ADC_IF_CH2OVF_Msk			(0x01 << ADC_IF_CH2OVF_Pos)
#define ADC_IF_CH3EOC_Pos			6
#define ADC_IF_CH3EOC_Msk			(0x01 << ADC_IF_CH3EOC_Pos)
#define ADC_IF_CH3OVF_Pos			7
#define ADC_IF_CH3OVF_Msk			(0x01 << ADC_IF_CH3OVF_Pos)
#define ADC_IF_CH4EOC_Pos			8
#define ADC_IF_CH4EOC_Msk			(0x01 << ADC_IF_CH4EOC_Pos)
#define ADC_IF_CH4OVF_Pos			9
#define ADC_IF_CH4OVF_Msk			(0x01 << ADC_IF_CH4OVF_Pos)
#define ADC_IF_CH5EOC_Pos			10
#define ADC_IF_CH5EOC_Msk			(0x01 << ADC_IF_CH5EOC_Pos)
#define ADC_IF_CH5OVF_Pos			11
#define ADC_IF_CH5OVF_Msk			(0x01 << ADC_IF_CH5OVF_Pos)
#define ADC_IF_CH6EOC_Pos			12
#define ADC_IF_CH6EOC_Msk			(0x01 << ADC_IF_CH6EOC_Pos)
#define ADC_IF_CH6OVF_Pos			13
#define ADC_IF_CH6OVF_Msk			(0x01 << ADC_IF_CH6OVF_Pos)
#define ADC_IF_CH7EOC_Pos			14
#define ADC_IF_CH7EOC_Msk			(0x01 << ADC_IF_CH7EOC_Pos)
#define ADC_IF_CH7OVF_Pos			15
#define ADC_IF_CH7OVF_Msk			(0x01 << ADC_IF_CH7OVF_Pos)
#define ADC_IF_CH8EOC_Pos			16
#define ADC_IF_CH8EOC_Msk			(0x01 << ADC_IF_CH8EOC_Pos)
#define ADC_IF_CH8OVF_Pos			17
#define ADC_IF_CH8OVF_Msk			(0x01 << ADC_IF_CH8OVF_Pos)
#define ADC_IF_CH9EOC_Pos			18
#define ADC_IF_CH9EOC_Msk			(0x01 << ADC_IF_CH9EOC_Pos)
#define ADC_IF_CH9OVF_Pos			19
#define ADC_IF_CH9OVF_Msk			(0x01 << ADC_IF_CH9OVF_Pos)
#define ADC_IF_CH10EOC_Pos			20
#define ADC_IF_CH10EOC_Msk			(0x01 << ADC_IF_CH10EOC_Pos)
#define ADC_IF_CH10OVF_Pos			21
#define ADC_IF_CH10OVF_Msk			(0x01 << ADC_IF_CH10OVF_Pos)
#define ADC_IF_CH11EOC_Pos			22
#define ADC_IF_CH11EOC_Msk			(0x01 << ADC_IF_CH11EOC_Pos)
#define ADC_IF_CH11OVF_Pos			23
#define ADC_IF_CH11OVF_Msk			(0x01 << ADC_IF_CH11OVF_Pos)
#define ADC_IF_FIFOOV_Pos			24
#define ADC_IF_FIFOOV_Msk			(0x01 << ADC_IF_FIFOOV_Pos)
#define ADC_IF_FIFOHF_Pos			25
#define ADC_IF_FIFOHF_Msk			(0x01 << ADC_IF_FIFOHF_Pos)
#define ADC_IF_FIFOF_Pos			26
#define ADC_IF_FIFOF_Msk			(0x01 << ADC_IF_FIFOF_Pos)

#define ADC_STAT_EOC_Pos			0		//д1����
#define ADC_STAT_EOC_Msk			(0x01 << ADC_STAT_EOC_Pos)
#define ADC_STAT_OVF_Pos			1		//�����ݼĴ������
#define ADC_STAT_OVF_Msk			(0x01 << ADC_STAT_OVF_Pos)

#define ADC_DATA_VALUE_Pos			0		//������ٴ�ת�������ݻḲ�Ǿ�����
#define ADC_DATA_VALUE_Msk			(0xFFF << ADC_DATA_VALUE_Pos)
#define ADC_DATA_CHNUM_Pos			12
#define ADC_DATA_CHNUM_Msk			(0x0F << ADC_DATA_CHNUM_Pos)

#define ADC_CHSEL_SW_Pos			0		//�������ת��ʱ������ͨ��
#define ADC_CHSEL_SW_Msk			(0xFFF<< ADC_CHSEL_SW_Pos)
#define ADC_CHSEL_PWM_Pos			16		//PWM ����ת��ʱ������ͨ��
#define ADC_CHSEL_PWM_Msk			(0xFFF<< ADC_CHSEL_PWM_Pos)

#define ADC_FFSTAT_OVF_Pos			0
#define ADC_FFSTAT_OVF_Msk			(0x01 << ADC_FFSTAT_OVF_Pos)
#define ADC_FFSTAT_HFULL_Pos		1
#define ADC_FFSTAT_HFULL_Msk		(0x01 << ADC_FFSTAT_HFULL_Pos)
#define ADC_FFSTAT_FULL_Pos			2
#define ADC_FFSTAT_FULL_Msk			(0x01 << ADC_FFSTAT_FULL_Pos)
#define ADC_FFSTAT_EMPTY_Pos		3
#define ADC_FFSTAT_EMPTY_Msk		(0x01 << ADC_FFSTAT_EMPTY_Pos)
#define ADC_FFSTAT_LEVEL_Pos		4
#define ADC_FFSTAT_LEVEL_Msk		(0x07 << ADC_FFSTAT_LEVEL_Pos)

#define ADC_FFDATA_VALUE_Pos		0		//������ٴ�ת�������ݻᱻ����
#define ADC_FFDATA_VALUE_Msk		(0xFFF << ADC_FFDATA_VALUE_Pos)
#define ADC_FFDATA_CHNUM_Pos		12
#define ADC_FFDATA_CHNUM_Msk		(0x0F << ADC_FFDATA_CHNUM_Pos)

#define ADC_CTRL1_CLKSRC_Pos		0		//0 SYS->CLKSEL�Ĵ������趨    1 �ⲿ����
#define ADC_CTRL1_CLKSRC_Msk		(0x01 << ADC_CTRL1_CLKSRC_Pos)

#define ADC_CTRL2_ADCEVCM_Pos		1		//ADC External VCM��ADC��PGA�����ģ��ƽѡ��
#define ADC_CTRL2_ADCEVCM_Msk		(0x01 << ADC_CTRL2_ADCEVCM_Pos)
#define ADC_CTRL2_PGAEVCM_Pos		2		//PGA External VCM��PGA���빲ģ��ƽѡ��
#define ADC_CTRL2_PGAEVCM_Msk		(0x01 << ADC_CTRL2_PGAEVCM_Pos)
#define ADC_CTRL2_PGAGAIN_Pos		3
#define ADC_CTRL2_PGAGAIN_Msk		(0x07 << ADC_CTRL2_PGAGAIN_Pos)
#define ADC_CTRL2_VCMSEL_Pos		8
#define ADC_CTRL2_VCMSEL_Msk		(0x07 << ADC_CTRL2_VCMSEL_Pos)
#define ADC_CTRL2_CLKDIV2_Pos		24
#define ADC_CTRL2_CLKDIV2_Msk		(0x1F << ADC_CTRL2_CLKDIV2_Pos)
#define ADC_CTRL2_CLKDIV1_Pos		29
#define ADC_CTRL2_CLKDIV1_Msk		(0x03 << ADC_CTRL2_CLKDIV1_Pos)

#define ADC_CTRL3_REFPSEL_Pos		2		//0 Vrefp pin   1 VDD
#define ADC_CTRL3_REFPSEL_Msk		(0x01 << ADC_CTRL3_REFPSEL_Pos)
#define ADC_CTRL3_CLKDIV0_Pos		3
#define ADC_CTRL3_CLKDIV0_Msk		(0x03 << ADC_CTRL3_CLKDIV0_Pos)

#define ADC_CALIBSET_OFFSET_Pos		0
#define ADC_CALIBSET_OFFSET_Msk		(0x1FF<< ADC_CALIBSET_OFFSET_Pos)
#define ADC_CALIBSET_K_Pos			16
#define ADC_CALIBSET_K_Msk			(0x1FF<< ADC_CALIBSET_K_Pos)

#define ADC_CALIBEN_OFFSET_Pos		0
#define ADC_CALIBEN_OFFSET_Msk		(0x01 << ADC_CALIBEN_OFFSET_Pos)
#define ADC_CALIBEN_K_Pos			1
#define ADC_CALIBEN_K_Msk			(0x01 << ADC_CALIBEN_K_Pos)




typedef struct {
	__IO uint32_t MODE;
	
	__IO uint32_t PERA;                     //[15:0] ����
	
	__IO uint32_t HIGHA;                    //[16:1] �ߵ�ƽ����ʱ��    [0] �Ƿ񽫸ߵ�ƽ���½����Ƴٰ��ʱ������
	
	__IO uint32_t DZA;                      //[9:0] ���������������Ƴ�ʱ��������С��HIGHA
	
	__IO uint32_t PERB;
	
	__IO uint32_t HIGHB;
	
	__IO uint32_t DZB;
	
	__IO uint32_t OUTCR;
	
	__IO uint32_t ADTRGA0;					//A· ADC Trigger Enable
    __IO uint32_t ADTRGA1;
	
	__IO uint32_t BRKEN;					//Brake Enable
	
	__IO uint32_t VALUEA;					//��ǰ����ֵ
	__IO uint32_t VALUEB;
	
	__IO uint32_t ADTRGB0;					//ע�⣺ֻ�е� PWMxA ͨ��ʹ��ʱ��PWMxB ͨ�����ܴ��� ADC
    __IO uint32_t ADTRGB1;
	
	__IO uint32_t ADTRGDIR;					//���ĶԳ�ģʽ�£���������������ʱ�Ƿ񴥷�ADC���������ݼ�����ʱ�Ƿ񴥷�ADC
} PWM_TypeDef;


#define PWM_MODE_MODE_Pos			0		//0 ���ض���ģʽ   1 ���ĶԳ�ģʽ
#define PWM_MODE_MODE_Msk			(0x07 << PWM_MODE_MODE_Pos)
#define PWM_MODE_CLKSRC_Pos			3		//0 APB    1 APB��Ƶ    2 PWM_PULSE0 ��������    3 PWM_PULSE1 ��������
#define PWM_MODE_CLKSRC_Msk			(0x03 << PWM_MODE_CLKSRC_Pos)
#define PWM_MODE_CLKDIV_Pos			5		//0 1��Ƶ    1 2��Ƶ    ...
#define PWM_MODE_CLKDIV_Msk			(0xFF << PWM_MODE_CLKDIV_Pos)

#define PWM_OUTCR_INIA_Pos			0		//1 ����Ӹߵ�ƽ��ʼ
#define PWM_OUTCR_INIA_Msk			(0x01 << PWM_OUTCR_INIA_Pos)
#define PWM_OUTCR_INIB_Pos			1		
#define PWM_OUTCR_INIB_Msk			(0x01 << PWM_OUTCR_INIB_Pos)
#define PWM_OUTCR_INVA_Pos			2		//1 PWMxA ���ȡ��
#define PWM_OUTCR_INVA_Msk			(0x01 << PWM_OUTCR_INVA_Pos)
#define PWM_OUTCR_INVB_Pos			3
#define PWM_OUTCR_INVB_Msk			(0x01 << PWM_OUTCR_INVB_Pos)
#define PWM_OUTCR_IDLEA_Pos			4		//1 ����ʱ����ߵ�ƽ
#define PWM_OUTCR_IDLEA_Msk			(0x01 << PWM_OUTCR_IDLEA_Pos)
#define PWM_OUTCR_IDLEB_Pos			5
#define PWM_OUTCR_IDLEB_Msk			(0x01 << PWM_OUTCR_IDLEB_Pos)
#define PWM_OUTCR_INVAN_Pos			6		//1 PWMxAN ���ȡ��
#define PWM_OUTCR_INVAN_Msk			(0x01 << PWM_OUTCR_INVAN_Pos)
#define PWM_OUTCR_INVBN_Pos			7
#define PWM_OUTCR_INVBN_Msk			(0x01 << PWM_OUTCR_INVBN_Pos)

#define PWM_ADTRG_MATCH_Pos			0		
#define PWM_ADTRG_MATCH_Msk			(0xFFFF << PWM_ADTRG_MATCH_Pos)
#define PWM_ADTRG_EN_Pos		    16		
#define PWM_ADTRG_EN_Msk		    (0x01 << PWM_ADTRG_EN_Pos)

#define PWM_BRKEN_EN_Pos			0
#define PWM_BRKEN_EN_Msk			(0x01 << PWM_BRKEN_EN_Pos)
#define PWM_BRKEN_S0_Pos			1		//ɲ���ź�0ʹ��
#define PWM_BRKEN_S0_Msk			(0x01 << PWM_BRKEN_S0_Pos)
#define PWM_BRKEN_S1_Pos			2		//ɲ���ź�1ʹ��
#define PWM_BRKEN_S1_Msk			(0x01 << PWM_BRKEN_S1_Pos)
#define PWM_BRKEN_S2_Pos			3		//ɲ���ź�2ʹ��
#define PWM_BRKEN_S2_Msk			(0x01 << PWM_BRKEN_S2_Pos)
#define PWM_BRKEN_OUT_Pos			4		//ɲ��ʱPWM�����ƽ
#define PWM_BRKEN_OUT_Msk			(0x01 << PWM_BRKEN_OUT_Pos)

#define PWM_ADTRGDIR_A0DEC_Pos		0		//���ĶԳ�ģʽ�£��������ݼ�����ʱ����������ڣ�ADTRGA0�Ƿ�������
#define PWM_ADTRGDIR_A0DEC_Msk		(0x01 << PWM_ADTRGDIR_A0DEC_Pos)
#define PWM_ADTRGDIR_A0INC_Pos		1		//���ĶԳ�ģʽ�£���������������ʱ����ǰ�����ڣ�ADTRGA0�Ƿ�������
#define PWM_ADTRGDIR_A0INC_Msk		(0x01 << PWM_ADTRGDIR_A0INC_Pos)
#define PWM_ADTRGDIR_A1DEC_Pos		2
#define PWM_ADTRGDIR_A1DEC_Msk		(0x01 << PWM_ADTRGDIR_A1DEC_Pos)
#define PWM_ADTRGDIR_A1INC_Pos		3
#define PWM_ADTRGDIR_A1INC_Msk		(0x01 << PWM_ADTRGDIR_A1INC_Pos)
#define PWM_ADTRGDIR_B0DEC_Pos		4
#define PWM_ADTRGDIR_B0DEC_Msk		(0x01 << PWM_ADTRGDIR_B0DEC_Pos)
#define PWM_ADTRGDIR_B0INC_Pos		5
#define PWM_ADTRGDIR_B0INC_Msk		(0x01 << PWM_ADTRGDIR_B0INC_Pos)
#define PWM_ADTRGDIR_B1DEC_Pos		6
#define PWM_ADTRGDIR_B1DEC_Msk		(0x01 << PWM_ADTRGDIR_B1DEC_Pos)
#define PWM_ADTRGDIR_B1INC_Pos		7
#define PWM_ADTRGDIR_B1INC_Msk		(0x01 << PWM_ADTRGDIR_B1INC_Pos)


typedef struct {
	__IO uint32_t CONFIG;
	
	__IO uint32_t FORCEO;					//Force Output
    
    __IO uint32_t BRKCR;					//Brake Control
    __IO uint32_t BRKIE;
    __I  uint32_t BRKIF;
    __IO uint32_t BRKIM;
    __IO uint32_t BRKIRS;
	
    __IO uint32_t IE;
    
	__IO uint32_t CHEN;
	
	__IO uint32_t IM;
    
	__IO uint32_t NCIRS;
    
	__IO uint32_t HEIRS;
    
	__I  uint32_t NCIF;
    
	__I  uint32_t HEIF;
	
	__IO uint32_t HCIE;						//�������ж�ʹ��
	
	__IO uint32_t HCIM;
	
	__IO uint32_t HCIRS;					//�ж�ԭʼ״̬��д1����
	
	__I  uint32_t HCIF;
	
	__IO uint32_t FORCEV;					//Force Value
} PWMG_TypeDef;


#define PWMG_CONFIG_P0RISE_Pos		3		//PULSE0�������أ�1 ������   0 �½���
#define PWMG_CONFIG_P0RISE_Msk		(0x01 << PWMG_CONFIG_P0RISE_Msk)
#define PWMG_CONFIG_P1RISE_Pos		4
#define PWMG_CONFIG_P1RISE_Msk		(0x01 << PWMG_CONFIG_P1RISE_Msk)

#define PWMG_FORCEO_PWM0A_Pos		0		//0 PWM0A�������    1 PWM0Aǿ������̶���ƽ�������ƽֵ��PWMG->FORCEV.PWM0Aλ����
#define PWMG_FORCEO_PWM0A_Msk		(0x01 << PWMG_FORCEO_PWM0A_Pos)
#define PWMG_FORCEO_PWM1A_Pos		1		
#define PWMG_FORCEO_PWM1A_Msk		(0x01 << PWMG_FORCEO_PWM1A_Pos)
#define PWMG_FORCEO_PWM2A_Pos		2		
#define PWMG_FORCEO_PWM2A_Msk		(0x01 << PWMG_FORCEO_PWM2A_Pos)
#define PWMG_FORCEO_PWM3A_Pos		3		
#define PWMG_FORCEO_PWM3A_Msk		(0x01 << PWMG_FORCEO_PWM3A_Pos)
#define PWMG_FORCEO_PWM0AN_Pos		8		
#define PWMG_FORCEO_PWM0AN_Msk		(0x01 << PWMG_FORCEO_PWM0AN_Pos)
#define PWMG_FORCEO_PWM1AN_Pos		9		
#define PWMG_FORCEO_PWM1AN_Msk		(0x01 << PWMG_FORCEO_PWM1AN_Pos)
#define PWMG_FORCEO_PWM2AN_Pos		10		
#define PWMG_FORCEO_PWM2AN_Msk		(0x01 << PWMG_FORCEO_PWM2AN_Pos)
#define PWMG_FORCEO_PWM3AN_Pos		11		
#define PWMG_FORCEO_PWM3AN_Msk		(0x01 << PWMG_FORCEO_PWM3AN_Pos)
#define PWMG_FORCEO_PWM0B_Pos		16		
#define PWMG_FORCEO_PWM0B_Msk		(0x01 << PWMG_FORCEO_PWM0B_Pos)
#define PWMG_FORCEO_PWM1B_Pos		17		
#define PWMG_FORCEO_PWM1B_Msk		(0x01 << PWMG_FORCEO_PWM1B_Pos)
#define PWMG_FORCEO_PWM2B_Pos		18		
#define PWMG_FORCEO_PWM2B_Msk		(0x01 << PWMG_FORCEO_PWM2B_Pos)
#define PWMG_FORCEO_PWM3B_Pos		19		
#define PWMG_FORCEO_PWM3B_Msk		(0x01 << PWMG_FORCEO_PWM3B_Pos)
#define PWMG_FORCEO_PWM0BN_Pos		24		
#define PWMG_FORCEO_PWM0BN_Msk		(0x01 << PWMG_FORCEO_PWM0BN_Pos)
#define PWMG_FORCEO_PWM1BN_Pos		25		
#define PWMG_FORCEO_PWM1BN_Msk		(0x01 << PWMG_FORCEO_PWM1BN_Pos)
#define PWMG_FORCEO_PWM2BN_Pos		26		
#define PWMG_FORCEO_PWM2BN_Msk		(0x01 << PWMG_FORCEO_PWM2BN_Pos)
#define PWMG_FORCEO_PWM3BN_Pos		27		
#define PWMG_FORCEO_PWM3BN_Msk		(0x01 << PWMG_FORCEO_PWM3BN_Pos)

#define PWMG_BRKCR_STOPCNT_Pos		0       //1 ɲ��ʱ��PWM���������㣬ֹͣ����    0 ɲ��ʱ��PWM�������������� 
#define PWMG_BRKCR_STOPCNT_Msk		(0x01 << PWMG_BRKCR_STOPCNT_Pos)
#define PWMG_BRKCR_S0STATE_Pos		3		//ɲ���ź�0��ǰ״̬
#define PWMG_BRKCR_S0STATE_Msk		(0x01 << PWMG_BRKCR_S0STATE_Pos)
#define PWMG_BRKCR_S1STATE_Pos		4
#define PWMG_BRKCR_S1STATE_Msk		(0x01 << PWMG_BRKCR_S1STATE_Pos)
#define PWMG_BRKCR_S2STATE_Pos		5
#define PWMG_BRKCR_S2STATE_Msk		(0x01 << PWMG_BRKCR_S2STATE_Pos)
#define PWMG_BRKCR_S0STCLR_Pos		6		//ɲ���ź�0״̬���
#define PWMG_BRKCR_S0STCLR_Msk		(0x01 << PWMG_BRKCR_S0STCLR_Pos)
#define PWMG_BRKCR_S1STCLR_Pos		7
#define PWMG_BRKCR_S1STCLR_Msk		(0x01 << PWMG_BRKCR_S1STCLR_Pos)
#define PWMG_BRKCR_S2STCLR_Pos		8
#define PWMG_BRKCR_S2STCLR_Msk		(0x01 << PWMG_BRKCR_S2STCLR_Pos)
#define PWMG_BRKCR_S0INPOL_Pos		9		//ɲ���ź�0������Ч���ԣ�0 �͵�ƽ��Ч   1 �ߵ�ƽ��Ч
#define PWMG_BRKCR_S0INPOL_Msk		(0x01 << PWMG_BRKCR_S0INPOL_Pos)
#define PWMG_BRKCR_S1INPOL_Pos		10		
#define PWMG_BRKCR_S1INPOL_Msk		(0x01 << PWMG_BRKCR_S1INPOL_Pos)
#define PWMG_BRKCR_S2INPOL_Pos		11		
#define PWMG_BRKCR_S2INPOL_Msk		(0x01 << PWMG_BRKCR_S2INPOL_Pos)

#define PWMG_BRKIE_S0_Pos			0		//ɲ���ź�0�ж�ʹ��
#define PWMG_BRKIE_S0_Msk			(0x01 << PWMG_BRKIE_S0_Pos)
#define PWMG_BRKIE_S1_Pos			1
#define PWMG_BRKIE_S1_Msk			(0x01 << PWMG_BRKIE_S1_Pos)
#define PWMG_BRKIE_S2_Pos			2
#define PWMG_BRKIE_S2_Msk			(0x01 << PWMG_BRKIE_S2_Pos)

#define PWMG_BRKIF_S0_Pos			0
#define PWMG_BRKIF_S0_Msk			(0x01 << PWMG_BRKIF_S0_Pos)
#define PWMG_BRKIF_S1_Pos			1
#define PWMG_BRKIF_S1_Msk			(0x01 << PWMG_BRKIF_S1_Pos)
#define PWMG_BRKIF_S2_Pos			2
#define PWMG_BRKIF_S2_Msk			(0x01 << PWMG_BRKIF_S2_Pos)

#define PWMG_BRKIM_S0_Pos			0
#define PWMG_BRKIM_S0_Msk			(0x01 << PWMG_BRKIM_S0_Pos)
#define PWMG_BRKIM_S1_Pos			1
#define PWMG_BRKIM_S1_Msk			(0x01 << PWMG_BRKIM_S1_Pos)
#define PWMG_BRKIM_S2_Pos			2
#define PWMG_BRKIM_S2_Msk			(0x01 << PWMG_BRKIM_S2_Pos)

#define PWMG_BRKIRS_S0_Pos			0		//Raw State, д1����
#define PWMG_BRKIRS_S0_Msk			(0x01 << PWMG_BRKIRS_S0_Pos)
#define PWMG_BRKIRS_S1_Pos			1
#define PWMG_BRKIRS_S1_Msk			(0x01 << PWMG_BRKIRS_S1_Pos)
#define PWMG_BRKIRS_S2_Pos			2
#define PWMG_BRKIRS_S2_Msk			(0x01 << PWMG_BRKIRS_S2_Pos)

#define PWMG_IE_NC0A_Pos			0		//New Cycle		
#define PWMG_IE_NC0A_Msk			(0x01 << PWMG_IE_NC0A_Pos)
#define PWMG_IE_NC0B_Pos			1		
#define PWMG_IE_NC0B_Msk			(0x01 << PWMG_IE_NC0B_Pos)
#define PWMG_IE_NC1A_Pos			2		
#define PWMG_IE_NC1A_Msk			(0x01 << PWMG_IE_NC1A_Pos)
#define PWMG_IE_NC1B_Pos			3		
#define PWMG_IE_NC1B_Msk			(0x01 << PWMG_IE_NC1B_Pos)
#define PWMG_IE_NC2A_Pos			4		
#define PWMG_IE_NC2A_Msk			(0x01 << PWMG_IE_NC2A_Pos)
#define PWMG_IE_NC2B_Pos			5		
#define PWMG_IE_NC2B_Msk			(0x01 << PWMG_IE_NC2B_Pos)
#define PWMG_IE_NC3A_Pos			6		
#define PWMG_IE_NC3A_Msk			(0x01 << PWMG_IE_NC3A_Pos)
#define PWMG_IE_NC3B_Pos			7		
#define PWMG_IE_NC3B_Msk			(0x01 << PWMG_IE_NC3B_Pos)
#define PWMG_IE_HE0A_Pos			16		//High Level End
#define PWMG_IE_HE0A_Msk			(0x01 << PWMG_IE_HE0A_Pos)
#define PWMG_IE_HE0B_Pos			17		
#define PWMG_IE_HE0B_Msk			(0x01 << PWMG_IE_HE0B_Pos)
#define PWMG_IE_HE1A_Pos			18		
#define PWMG_IE_HE1A_Msk			(0x01 << PWMG_IE_HE1A_Pos)
#define PWMG_IE_HE1B_Pos			19		
#define PWMG_IE_HE1B_Msk			(0x01 << PWMG_IE_HE1B_Pos)
#define PWMG_IE_HE2A_Pos			20		
#define PWMG_IE_HE2A_Msk			(0x01 << PWMG_IE_HE2A_Pos)
#define PWMG_IE_HE2B_Pos			21		
#define PWMG_IE_HE2B_Msk			(0x01 << PWMG_IE_HE2B_Pos)
#define PWMG_IE_HE3A_Pos			22		
#define PWMG_IE_HE3A_Msk			(0x01 << PWMG_IE_HE3A_Pos)
#define PWMG_IE_HE3B_Pos			23		
#define PWMG_IE_HE3B_Msk			(0x01 << PWMG_IE_HE3B_Pos)

#define PWMG_CHEN_PWM0A_Pos		    0		
#define PWMG_CHEN_PWM0A_Msk		    (0x01 << PWMG_CHEN_PWM0A_Pos)
#define PWMG_CHEN_PWM1A_Pos		    1		
#define PWMG_CHEN_PWM1A_Msk		    (0x01 << PWMG_CHEN_PWM1A_Pos)
#define PWMG_CHEN_PWM2A_Pos		    2		
#define PWMG_CHEN_PWM2A_Msk		    (0x01 << PWMG_CHEN_PWM2A_Pos)
#define PWMG_CHEN_PWM3A_Pos		    3		
#define PWMG_CHEN_PWM3A_Msk		    (0x01 << PWMG_CHEN_PWM3A_Pos)
#define PWMG_CHEN_PWM0B_Pos		    8		
#define PWMG_CHEN_PWM0B_Msk		    (0x01 << PWMG_CHEN_PWM0B_Pos)
#define PWMG_CHEN_PWM1B_Pos		    9		
#define PWMG_CHEN_PWM1B_Msk		    (0x01 << PWMG_CHEN_PWM1B_Pos)
#define PWMG_CHEN_PWM2B_Pos		    10		
#define PWMG_CHEN_PWM2B_Msk		    (0x01 << PWMG_CHEN_PWM2B_Pos)
#define PWMG_CHEN_PWM3B_Pos		    11		
#define PWMG_CHEN_PWM3B_Msk		    (0x01 << PWMG_CHEN_PWM3B_Pos)

#define PWMG_IM_NC0A_Pos			0		
#define PWMG_IM_NC0A_Msk			(0x01 << PWMG_IM_NC0A_Pos)
#define PWMG_IM_NC0B_Pos			1		
#define PWMG_IM_NC0B_Msk			(0x01 << PWMG_IM_NC0B_Pos)
#define PWMG_IM_NC1A_Pos			2		
#define PWMG_IM_NC1A_Msk			(0x01 << PWMG_IM_NC1A_Pos)
#define PWMG_IM_NC1B_Pos			3		
#define PWMG_IM_NC1B_Msk			(0x01 << PWMG_IM_NC1B_Pos)
#define PWMG_IM_NC2A_Pos			4		
#define PWMG_IM_NC2A_Msk			(0x01 << PWMG_IM_NC2A_Pos)
#define PWMG_IM_NC2B_Pos			5		
#define PWMG_IM_NC2B_Msk			(0x01 << PWMG_IM_NC2B_Pos)
#define PWMG_IM_NC3A_Pos			6		
#define PWMG_IM_NC3A_Msk			(0x01 << PWMG_IM_NC3A_Pos)
#define PWMG_IM_NC3B_Pos			7		
#define PWMG_IM_NC3B_Msk			(0x01 << PWMG_IM_NC3B_Pos)
#define PWMG_IM_HE0A_Pos			16		
#define PWMG_IM_HE0A_Msk			(0x01 << PWMG_IM_HE0A_Pos)
#define PWMG_IM_HE0B_Pos			17		
#define PWMG_IM_HE0B_Msk			(0x01 << PWMG_IM_HE0B_Pos)
#define PWMG_IM_HE1A_Pos			18		
#define PWMG_IM_HE1A_Msk			(0x01 << PWMG_IM_HE1A_Pos)
#define PWMG_IM_HE1B_Pos			19		
#define PWMG_IM_HE1B_Msk			(0x01 << PWMG_IM_HE1B_Pos)
#define PWMG_IM_HE2A_Pos			20		
#define PWMG_IM_HE2A_Msk			(0x01 << PWMG_IM_HE2A_Pos)
#define PWMG_IM_HE2B_Pos			21		
#define PWMG_IM_HE2B_Msk			(0x01 << PWMG_IM_HE2B_Pos)
#define PWMG_IM_HE3A_Pos			22		
#define PWMG_IM_HE3A_Msk			(0x01 << PWMG_IM_HE3A_Pos)
#define PWMG_IM_HE3B_Pos			23		
#define PWMG_IM_HE3B_Msk			(0x01 << PWMG_IM_HE3B_Pos)

#define PWMG_NCIRS_PWM0A_Pos		0       //PWM0A New Cycle Interrupt Raw State��д1����
#define PWMG_NCIRS_PWM0A_Msk		(0x01 << PWMG_NCIRS_PWM0A_Pos)
#define PWMG_NCIRS_PWM0B_Pos		1
#define PWMG_NCIRS_PWM0B_Msk		(0x01 << PWMG_NCIRS_PWM0B_Pos)
#define PWMG_NCIRS_PWM1A_Pos		2
#define PWMG_NCIRS_PWM1A_Msk		(0x01 << PWMG_NCIRS_PWM1A_Pos)
#define PWMG_NCIRS_PWM1B_Pos		3
#define PWMG_NCIRS_PWM1B_Msk		(0x01 << PWMG_NCIRS_PWM1B_Pos)
#define PWMG_NCIRS_PWM2A_Pos		4
#define PWMG_NCIRS_PWM2A_Msk		(0x01 << PWMG_NCIRS_PWM2A_Pos)
#define PWMG_NCIRS_PWM2B_Pos		5
#define PWMG_NCIRS_PWM2B_Msk		(0x01 << PWMG_NCIRS_PWM2B_Pos)
#define PWMG_NCIRS_PWM3A_Pos		6
#define PWMG_NCIRS_PWM3A_Msk		(0x01 << PWMG_NCIRS_PWM3A_Pos)
#define PWMG_NCIRS_PWM3B_Pos		7
#define PWMG_NCIRS_PWM3B_Msk		(0x01 << PWMG_NCIRS_PWM3B_Pos)

#define PWMG_HEIRS_PWM0A_Pos		0       //PWM0A High Level End Interrupt Raw State��д1����
#define PWMG_HEIRS_PWM0A_Msk		(0x01 << PWMG_HEIRS_PWM0A_Pos)
#define PWMG_HEIRS_PWM0B_Pos		1
#define PWMG_HEIRS_PWM0B_Msk		(0x01 << PWMG_HEIRS_PWM0B_Pos)
#define PWMG_HEIRS_PWM1A_Pos		2
#define PWMG_HEIRS_PWM1A_Msk		(0x01 << PWMG_HEIRS_PWM1A_Pos)
#define PWMG_HEIRS_PWM1B_Pos		3
#define PWMG_HEIRS_PWM1B_Msk		(0x01 << PWMG_HEIRS_PWM1B_Pos)
#define PWMG_HEIRS_PWM2A_Pos		4
#define PWMG_HEIRS_PWM2A_Msk		(0x01 << PWMG_HEIRS_PWM2A_Pos)
#define PWMG_HEIRS_PWM2B_Pos		5
#define PWMG_HEIRS_PWM2B_Msk		(0x01 << PWMG_HEIRS_PWM2B_Pos)
#define PWMG_HEIRS_PWM3A_Pos		6
#define PWMG_HEIRS_PWM3A_Msk		(0x01 << PWMG_HEIRS_PWM3A_Pos)
#define PWMG_HEIRS_PWM3B_Pos		7
#define PWMG_HEIRS_PWM3B_Msk		(0x01 << PWMG_HEIRS_PWM3B_Pos)

#define PWMG_NCIF_PWM0A_Pos			0      
#define PWMG_NCIF_PWM0A_Msk			(0x01 << PWMG_NCIF_PWM0A_Pos)
#define PWMG_NCIF_PWM0B_Pos			1
#define PWMG_NCIF_PWM0B_Msk			(0x01 << PWMG_NCIF_PWM0B_Pos)
#define PWMG_NCIF_PWM1A_Pos			2
#define PWMG_NCIF_PWM1A_Msk			(0x01 << PWMG_NCIF_PWM1A_Pos)
#define PWMG_NCIF_PWM1B_Pos			3
#define PWMG_NCIF_PWM1B_Msk			(0x01 << PWMG_NCIF_PWM1B_Pos)
#define PWMG_NCIF_PWM2A_Pos			4
#define PWMG_NCIF_PWM2A_Msk			(0x01 << PWMG_NCIF_PWM2A_Pos)
#define PWMG_NCIF_PWM2B_Pos			5
#define PWMG_NCIF_PWM2B_Msk			(0x01 << PWMG_NCIF_PWM2B_Pos)
#define PWMG_NCIF_PWM3A_Pos			6
#define PWMG_NCIF_PWM3A_Msk			(0x01 << PWMG_NCIF_PWM3A_Pos)
#define PWMG_NCIF_PWM3B_Pos			7
#define PWMG_NCIF_PWM3B_Msk			(0x01 << PWMG_NCIF_PWM3B_Pos)

#define PWMG_HEIF_PWM0A_Pos			0       
#define PWMG_HEIF_PWM0A_Msk			(0x01 << PWMG_HEIF_PWM0A_Pos)
#define PWMG_HEIF_PWM0B_Pos			1
#define PWMG_HEIF_PWM0B_Msk			(0x01 << PWMG_HEIF_PWM0B_Pos)
#define PWMG_HEIF_PWM1A_Pos			2
#define PWMG_HEIF_PWM1A_Msk			(0x01 << PWMG_HEIF_PWM1A_Pos)
#define PWMG_HEIF_PWM1B_Pos			3
#define PWMG_HEIF_PWM1B_Msk			(0x01 << PWMG_HEIF_PWM1B_Pos)
#define PWMG_HEIF_PWM2A_Pos			4
#define PWMG_HEIF_PWM2A_Msk			(0x01 << PWMG_HEIF_PWM2A_Pos)
#define PWMG_HEIF_PWM2B_Pos			5
#define PWMG_HEIF_PWM2B_Msk			(0x01 << PWMG_HEIF_PWM2B_Pos)
#define PWMG_HEIF_PWM3A_Pos			6
#define PWMG_HEIF_PWM3A_Msk			(0x01 << PWMG_HEIF_PWM3A_Pos)
#define PWMG_HEIF_PWM3B_Pos			7
#define PWMG_HEIF_PWM3B_Msk			(0x01 << PWMG_HEIF_PWM3B_Pos)

#define PWMG_HCIE_PWM0A_Pos			0
#define PWMG_HCIE_PWM0A_Msk			(0x01 << PWMG_HCIE_PWM0A_Pos)
#define PWMG_HCIE_PWM0B_Pos			1
#define PWMG_HCIE_PWM0B_Msk			(0x01 << PWMG_HCIE_PWM0B_Pos)
#define PWMG_HCIE_PWM1A_Pos			2
#define PWMG_HCIE_PWM1A_Msk			(0x01 << PWMG_HCIE_PWM1A_Pos)
#define PWMG_HCIE_PWM1B_Pos			3
#define PWMG_HCIE_PWM1B_Msk			(0x01 << PWMG_HCIE_PWM1B_Pos)
#define PWMG_HCIE_PWM2A_Pos			4
#define PWMG_HCIE_PWM2A_Msk			(0x01 << PWMG_HCIE_PWM2A_Pos)
#define PWMG_HCIE_PWM2B_Pos			5
#define PWMG_HCIE_PWM2B_Msk			(0x01 << PWMG_HCIE_PWM2B_Pos)
#define PWMG_HCIE_PWM3A_Pos			6
#define PWMG_HCIE_PWM3A_Msk			(0x01 << PWMG_HCIE_PWM3A_Pos)
#define PWMG_HCIE_PWM3B_Pos			7
#define PWMG_HCIE_PWM3B_Msk			(0x01 << PWMG_HCIE_PWM3B_Pos)

#define PWMG_HCIM_PWM0A_Pos			0      
#define PWMG_HCIM_PWM0A_Msk			(0x01 << PWMG_HCIM_PWM0A_Pos)
#define PWMG_HCIM_PWM0B_Pos			1
#define PWMG_HCIM_PWM0B_Msk			(0x01 << PWMG_HCIM_PWM0B_Pos)
#define PWMG_HCIM_PWM1A_Pos			2
#define PWMG_HCIM_PWM1A_Msk			(0x01 << PWMG_HCIM_PWM1A_Pos)
#define PWMG_HCIM_PWM1B_Pos			3
#define PWMG_HCIM_PWM1B_Msk			(0x01 << PWMG_HCIM_PWM1B_Pos)
#define PWMG_HCIM_PWM2A_Pos			4
#define PWMG_HCIM_PWM2A_Msk			(0x01 << PWMG_HCIM_PWM2A_Pos)
#define PWMG_HCIM_PWM2B_Pos			5
#define PWMG_HCIM_PWM2B_Msk			(0x01 << PWMG_HCIM_PWM2B_Pos)
#define PWMG_HCIM_PWM3A_Pos			6
#define PWMG_HCIM_PWM3A_Msk			(0x01 << PWMG_HCIM_PWM3A_Pos)
#define PWMG_HCIM_PWM3B_Pos			7
#define PWMG_HCIM_PWM3B_Msk			(0x01 << PWMG_HCIM_PWM3B_Pos)

#define PWMG_HCIRS_PWM0A_Pos		0       //д1����
#define PWMG_HCIRS_PWM0A_Msk		(0x01 << PWMG_HCIRS_PWM0A_Pos)
#define PWMG_HCIRS_PWM0B_Pos		1
#define PWMG_HCIRS_PWM0B_Msk		(0x01 << PWMG_HCIRS_PWM0B_Pos)
#define PWMG_HCIRS_PWM1A_Pos		2
#define PWMG_HCIRS_PWM1A_Msk		(0x01 << PWMG_HCIRS_PWM1A_Pos)
#define PWMG_HCIRS_PWM1B_Pos		3
#define PWMG_HCIRS_PWM1B_Msk		(0x01 << PWMG_HCIRS_PWM1B_Pos)
#define PWMG_HCIRS_PWM2A_Pos		4
#define PWMG_HCIRS_PWM2A_Msk		(0x01 << PWMG_HCIRS_PWM2A_Pos)
#define PWMG_HCIRS_PWM2B_Pos		5
#define PWMG_HCIRS_PWM2B_Msk		(0x01 << PWMG_HCIRS_PWM2B_Pos)
#define PWMG_HCIRS_PWM3A_Pos		6
#define PWMG_HCIRS_PWM3A_Msk		(0x01 << PWMG_HCIRS_PWM3A_Pos)
#define PWMG_HCIRS_PWM3B_Pos		7
#define PWMG_HCIRS_PWM3B_Msk		(0x01 << PWMG_HCIRS_PWM3B_Pos)

#define PWMG_HCIF_PWM0A_Pos			0      
#define PWMG_HCIF_PWM0A_Msk			(0x01 << PWMG_HCIF_PWM0A_Pos)
#define PWMG_HCIF_PWM0B_Pos			1
#define PWMG_HCIF_PWM0B_Msk			(0x01 << PWMG_HCIF_PWM0B_Pos)
#define PWMG_HCIF_PWM1A_Pos			2
#define PWMG_HCIF_PWM1A_Msk			(0x01 << PWMG_HCIF_PWM1A_Pos)
#define PWMG_HCIF_PWM1B_Pos			3
#define PWMG_HCIF_PWM1B_Msk			(0x01 << PWMG_HCIF_PWM1B_Pos)
#define PWMG_HCIF_PWM2A_Pos			4
#define PWMG_HCIF_PWM2A_Msk			(0x01 << PWMG_HCIF_PWM2A_Pos)
#define PWMG_HCIF_PWM2B_Pos			5
#define PWMG_HCIF_PWM2B_Msk			(0x01 << PWMG_HCIF_PWM2B_Pos)
#define PWMG_HCIF_PWM3A_Pos			6
#define PWMG_HCIF_PWM3A_Msk			(0x01 << PWMG_HCIF_PWM3A_Pos)
#define PWMG_HCIF_PWM3B_Pos			7
#define PWMG_HCIF_PWM3B_Msk			(0x01 << PWMG_HCIF_PWM3B_Pos)

#define PWMG_FORCEV_PWM0A_Pos		0		//0 ǿ�����Ϊ�͵�ƽ��INVΪ0ʱ��    1 ǿ�����Ϊ�ߵ�ƽ��INVΪ0ʱ��
#define PWMG_FORCEV_PWM0A_Msk		(0x01 << PWMG_FORCEV_PWM0A_Pos)
#define PWMG_FORCEV_PWM1A_Pos		1		
#define PWMG_FORCEV_PWM1A_Msk		(0x01 << PWMG_FORCEV_PWM1A_Pos)
#define PWMG_FORCEV_PWM2A_Pos		2		
#define PWMG_FORCEV_PWM2A_Msk		(0x01 << PWMG_FORCEV_PWM2A_Pos)
#define PWMG_FORCEV_PWM3A_Pos		3		
#define PWMG_FORCEV_PWM3A_Msk		(0x01 << PWMG_FORCEV_PWM3A_Pos)
#define PWMG_FORCEV_PWM0AN_Pos		8		
#define PWMG_FORCEV_PWM0AN_Msk		(0x01 << PWMG_FORCEV_PWM0AN_Pos)
#define PWMG_FORCEV_PWM1AN_Pos		9		
#define PWMG_FORCEV_PWM1AN_Msk		(0x01 << PWMG_FORCEV_PWM1AN_Pos)
#define PWMG_FORCEV_PWM2AN_Pos		10		
#define PWMG_FORCEV_PWM2AN_Msk		(0x01 << PWMG_FORCEV_PWM2AN_Pos)
#define PWMG_FORCEV_PWM3AN_Pos		11		
#define PWMG_FORCEV_PWM3AN_Msk		(0x01 << PWMG_FORCEV_PWM3AN_Pos)
#define PWMG_FORCEV_PWM0B_Pos		16		
#define PWMG_FORCEV_PWM0B_Msk		(0x01 << PWMG_FORCEV_PWM0B_Pos)
#define PWMG_FORCEV_PWM1B_Pos		17		
#define PWMG_FORCEV_PWM1B_Msk		(0x01 << PWMG_FORCEV_PWM1B_Pos)
#define PWMG_FORCEV_PWM2B_Pos		18		
#define PWMG_FORCEV_PWM2B_Msk		(0x01 << PWMG_FORCEV_PWM2B_Pos)
#define PWMG_FORCEV_PWM3B_Pos		19		
#define PWMG_FORCEV_PWM3B_Msk		(0x01 << PWMG_FORCEV_PWM3B_Pos)
#define PWMG_FORCEV_PWM0BN_Pos		24		
#define PWMG_FORCEV_PWM0BN_Msk		(0x01 << PWMG_FORCEV_PWM0BN_Pos)
#define PWMG_FORCEV_PWM1BN_Pos		25		
#define PWMG_FORCEV_PWM1BN_Msk		(0x01 << PWMG_FORCEV_PWM1BN_Pos)
#define PWMG_FORCEV_PWM2BN_Pos		26		
#define PWMG_FORCEV_PWM2BN_Msk		(0x01 << PWMG_FORCEV_PWM2BN_Pos)
#define PWMG_FORCEV_PWM3BN_Pos		27		
#define PWMG_FORCEV_PWM3BN_Msk		(0x01 << PWMG_FORCEV_PWM3BN_Pos)




typedef struct {
	__IO uint32_t EN;                       //[0] ENABLE
    
	__IO uint32_t IE;                       //ֻ��Ϊ1ʱ��IF[CHx]��DMA�������ʱ���ܱ�Ϊ1������һֱ������0
    
	__IO uint32_t IM;                       //��Ϊ1ʱ����ʹIF[CHx]Ϊ1��dma_intҲ������˱�1
    
	__IO uint32_t IF;                       //д1����
	
	__IO uint32_t DSTSGIE;					//ֻ��Scatter Gatherģʽ��ʹ��
	
	__IO uint32_t DSTSGIM;					//ֻ��Scatter Gatherģʽ��ʹ��
	
	__IO uint32_t DSTSGIF;					//ֻ��Scatter Gatherģʽ��ʹ��
	
	__IO uint32_t SRCSGIE;					//ֻ��Scatter Gatherģʽ��ʹ��
	
	__IO uint32_t SRCSGIM;					//ֻ��Scatter Gatherģʽ��ʹ��
	
	__IO uint32_t SRCSGIF;					//ֻ��Scatter Gatherģʽ��ʹ��
	
		 uint32_t RESERVED[5];
	
	__IO uint32_t PRI;						//���ȼ���1 �����ȼ�    0 �����ȼ�
	
	struct {
		__IO uint32_t CR;
		
		__IO uint32_t AM;                   //Adress Mode
		
		__IO uint32_t DST;
		
		__IO uint32_t DSTSGADDR1;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__IO uint32_t DSTSGADDR2;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__IO uint32_t DSTSGADDR3;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__IO uint32_t MUX;
		
		__IO uint32_t SRC;
		
		__IO uint32_t SRCSGADDR1;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__IO uint32_t SRCSGADDR2;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__IO uint32_t SRCSGADDR3;			//ֻ��Scatter Gatherģʽ��ʹ��
		
		__I  uint32_t DSTSR;
		
		__I  uint32_t SRCSR;
		
			 uint32_t RESERVED[3];
	} CH[2];
} DMA_TypeDef;


#define DMA_IE_CH0_Pos			    0		
#define DMA_IE_CH0_Msk			    (0x01 << DMA_IE_CH0_Pos)
#define DMA_IE_CH1_Pos			    1		
#define DMA_IE_CH1_Msk			    (0x01 << DMA_IE_CH1_Pos)

#define DMA_IM_CH0_Pos			    0		
#define DMA_IM_CH0_Msk			    (0x01 << DMA_IM_CH0_Pos)
#define DMA_IM_CH1_Pos			    1		
#define DMA_IM_CH1_Msk			    (0x01 << DMA_IM_CH1_Pos)

#define DMA_IF_CH0_Pos			    0		
#define DMA_IF_CH0_Msk			    (0x01 << DMA_IF_CH0_Pos)
#define DMA_IF_CH1_Pos			    1		
#define DMA_IF_CH1_Msk			    (0x01 << DMA_IF_CH1_Pos)

#define DMA_CR_LEN_Pos				0       //��ͨ�����䵥λ����
#define DMA_CR_LEN_Msk				(0xFFFFF<< DMA_CR_LEN_Pos)
#define DMA_CR_RXEN_Pos				24		//����������䣬���䷽��ΪSRC-->DST
#define DMA_CR_RXEN_Msk				(0x01 << DMA_CR_RXEN_Pos)
#define DMA_CR_TXEN_Pos				25		//����������䣬���䷽��ΪDST-->SRC
#define DMA_CR_TXEN_Msk				(0x01 << DMA_CR_TXEN_Pos)
#define DMA_CR_AUTORE_Pos			26      //Auto Restart, ͨ���ڴ�����ɺ��Ƿ��Զ���������
#define DMA_CR_AUTORE_Msk			(0x01 << DMA_CR_AUTORE_Pos)
#define DMA_CR_STEPOP_Pos			27		//Step Operation, �������䣬����1�δ���1����λ����
#define DMA_CR_STEPOP_Msk			(0x01 << DMA_CR_STEPOP_Pos)

#define DMA_AM_DSTAM_Pos			0       //Address Mode	0 ��ַ�̶�    1 ��ַ����    2 scatter gatherģʽ
#define DMA_AM_DSTAM_Msk			(0x03 << DMA_AM_DSTAM_Pos)
#define DMA_AM_DSTBIT_Pos			2		//����λ��0 �ֽ�    1 ����    2 ��
#define DMA_AM_DSTBIT_Msk			(0x03 << DMA_AM_DSTBIT_Pos)
#define DMA_AM_DSTBURST_Pos			4		//�������ͣ�0 Single    1 Burst��Inc4��
#define DMA_AM_DSTBURST_Msk			(0x01 << DMA_AM_DSTBURST_Pos)
#define DMA_AM_SRCAM_Pos			8
#define DMA_AM_SRCAM_Msk			(0x03 << DMA_AM_SRCAM_Pos)
#define DMA_AM_SRCBIT_Pos			10
#define DMA_AM_SRCBIT_Msk			(0x03 << DMA_AM_SRCBIT_Pos)
#define DMA_AM_SRCBURST_Pos			12
#define DMA_AM_SRCBURST_Msk			(0x01 << DMA_AM_SRCBURST_Pos)

#define DMA_MUX_DSTHSSIG_Pos		0		//Ŀ��������źţ�handshake signal��
#define DMA_MUX_DSTHSSIG_Msk		(0x03 << DMA_MUX_DSTHSSIG_Pos)
#define DMA_MUX_DSTHSEN_Pos			2		//Ŀ�������ʹ�ܣ�handshake enable��
#define DMA_MUX_DSTHSEN_Msk			(0x01 << DMA_MUX_DSTHSEN_Pos)
#define DMA_MUX_SRCHSSIG_Pos		8		//Դ�������ź�
#define DMA_MUX_SRCHSSIG_Msk		(0x03 << DMA_MUX_SRCHSSIG_Pos)
#define DMA_MUX_SRCHSEN_Pos			10		//Դ������ʹ��
#define DMA_MUX_SRCHSEN_Msk			(0x01 << DMA_MUX_SRCHSEN_Pos)
#define DMA_MUX_EXTHSSIG_Pos		16		//�ⲿ�����źţ�0 TIMR0   1 TIMR1   2 TIMR2   3 TIMR3   4 TIMR4
#define DMA_MUX_EXTHSSIG_Msk		(0x07 << DMA_MUX_EXTHSSIG_Pos)
#define DMA_MUX_EXTHSEN_Pos			19		//�ⲿ����ʹ�ܣ�0 �����λCR.RXEN/TXEN��������   1 EXTHSSRCѡ�еĴ���Դ��������
#define DMA_MUX_EXTHSEN_Msk			(0x01 << DMA_MUX_EXTHSEN_Pos)

#define DMA_DSTSR_LEN_Pos			0		//ʣ�ഫ����
#define DMA_DSTSR_LEN_Msk			(0xFFF<< DMA_DSTSR_LEN_Pos)
#define DMA_DSTSR_ERR_Pos			31		//�������ô���
#define DMA_DSTSR_ERR_Msk			(0x01u<< DMA_DSTSR_ERR_Pos)

#define DMA_SRCSR_LEN_Pos			0
#define DMA_SRCSR_LEN_Msk			(0xFFF<< DMA_SRCSR_LEN_Pos)
#define DMA_SRCSR_ERR_Pos			31
#define DMA_SRCSR_ERR_Msk			(0x01u<< DMA_SRCSR_ERR_Pos)




typedef struct {
	__IO uint32_t CR;						//Control Register
	
	__O  uint32_t CMD;						//Command Register
	
	__I  uint32_t SR;						//Status Register
	
	__IO uint32_t IF;						//Interrupt Flag����ȡ����
	
	__IO uint32_t IE;						//Interrupt Enable
	
	__IO uint32_t BT2;
	
	__IO uint32_t BT0;						//Bit Time Register 0
	
	__IO uint32_t BT1;						//Bit Time Register 1
	
	     uint32_t RESERVED;
	
	__IO uint32_t AFM;						//Acceptance Filter Mode
	
	__IO uint32_t AFE;						//Acceptance Filter Enable
	
	__I  uint32_t ALC;						//Arbitration Lost Capture, �ٲö�ʧ��׽
	
	__I  uint32_t ECC;						//Error code capture, ������벶׽
	
	__IO uint32_t EWLIM;					//Error Warning Limit, ���󱨾�����
	
	__IO uint32_t RXERR;					//RX�������
	
	__IO uint32_t TXERR;					//TX�������
	
	struct {
		__IO uint32_t INFO;					//�����ʽ���Buffer��д���ʷ���Buffer
	
		__IO uint32_t DATA[12];
	} FRAME;
	
	__I  uint32_t RMCNT;					//Receive Message Count
	
		 uint32_t RESERVED2[162];
	
	__IO uint32_t ACR[16];					//Acceptance Check Register, ���ռĴ���
	
		 uint32_t RESERVED3[16];
	
	__IO uint32_t AMR[16];					//Acceptance Mask Register, �������μĴ�������Ӧλд0��ID��������ռĴ���ƥ��
} CAN_TypeDef;


#define CAN_CR_RST_Pos				0
#define CAN_CR_RST_Msk				(0x01 << CAN_CR_RST_Pos)
#define CAN_CR_LOM_Pos				1		//Listen Only Mode
#define CAN_CR_LOM_Msk				(0x01 << CAN_CR_LOM_Pos)
#define CAN_CR_STM_Pos				2		//Self Test Mode, ��ģʽ�¼�ʹû��Ӧ��CAN������Ҳ���Գɹ�����
#define CAN_CR_STM_Msk				(0x01 << CAN_CR_STM_Pos)
#define CAN_CR_SLEEP_Pos			4		//д1����˯��ģʽ�������߻���ж�ʱ���Ѳ��Զ������λ
#define CAN_CR_SLEEP_Msk			(0x01 << CAN_CR_SLEEP_Pos)

#define CAN_CMD_TXREQ_Pos			0		//Transmission Request
#define CAN_CMD_TXREQ_Msk			(0x01 << CAN_CMD_TXREQ_Pos)
#define CAN_CMD_ABTTX_Pos			1		//Abort Transmission
#define CAN_CMD_ABTTX_Msk			(0x01 << CAN_CMD_ABTTX_Pos)
#define CAN_CMD_RRB_Pos				2		//Release Receive Buffer
#define CAN_CMD_RRB_Msk				(0x01 << CAN_CMD_RRB_Pos)
#define CAN_CMD_CLROV_Pos			3		//Clear Data Overrun
#define CAN_CMD_CLROV_Msk			(0x01 << CAN_CMD_CLROV_Pos)
#define CAN_CMD_SRR_Pos				4		//Self Reception Request
#define CAN_CMD_SRR_Msk				(0x01 << CAN_CMD_SRR_Pos)

#define CAN_SR_RXDA_Pos				0		//Receive Data Available������FIFO����������Ϣ���Զ�ȡ
#define CAN_SR_RXDA_Msk				(0x01 << CAN_SR_RXDA_Pos)
#define CAN_SR_RXOV_Pos				1		//Receive FIFO Overrun���½��յ���Ϣ���ڽ���FIFO����������
#define CAN_SR_RXOV_Msk				(0x01 << CAN_SR_RXOV_Pos)
#define CAN_SR_TXBR_Pos				2		//Transmit Buffer Release��0 ���ڴ���ǰ��ķ��ͣ����ڲ���д�µ���Ϣ    1 ����д���µ���Ϣ����
#define CAN_SR_TXBR_Msk				(0x01 << CAN_SR_TXBR_Pos)
#define CAN_SR_TXOK_Pos				3		//Transmit OK��successfully completed
#define CAN_SR_TXOK_Msk				(0x01 << CAN_SR_TXOK_Pos)
#define CAN_SR_RXBUSY_Pos			4		//Receive Busy�����ڽ���
#define CAN_SR_RXBUSY_Msk			(0x01 << CAN_SR_RXBUSY_Pos)
#define CAN_SR_TXBUSY_Pos			5		//Transmit Busy�����ڷ���
#define CAN_SR_TXBUSY_Msk			(0x01 << CAN_SR_TXBUSY_Pos)
#define CAN_SR_ERRWARN_Pos			6		//1 ����һ������������ﵽ Warning Limit
#define CAN_SR_ERRWARN_Msk			(0x01 << CAN_SR_ERRWARN_Pos)
#define CAN_SR_BUSOFF_Pos			7		//1 CAN �������������߹ر�״̬��û�в��뵽���߻
#define CAN_SR_BUSOFF_Msk			(0x01 << CAN_SR_BUSOFF_Pos)

#define CAN_IF_RXDA_Pos				0		//IF.RXDA = SR.RXDA & IE.RXDA
#define CAN_IF_RXDA_Msk				(0x01 << CAN_IF_RXDA_Pos)
#define CAN_IF_TXBR_Pos				1		//��IE.TXBR=1ʱ��SR.TXBR��0���1����λ��λ
#define CAN_IF_TXBR_Msk				(0x01 << CAN_IF_TXBR_Pos)
#define CAN_IF_ERRWARN_Pos			2		//��IE.ERRWARN=1ʱ��SR.ERRWARN��SR.BUSOFF 0-to-1 �� 1-to-0����λ��λ
#define CAN_IF_ERRWARN_Msk			(0x01 << CAN_IF_ERRWARN_Pos)
#define CAN_IF_RXOV_Pos				3		//IF.RXOV = SR.RXOV & IE.RXOV
#define CAN_IF_RXOV_Msk				(0x01 << CAN_IF_RXOV_Pos)
#define CAN_IF_WKUP_Pos				4		//��IE.WKUP=1ʱ����˯��ģʽ�µ�CAN��������⵽���߻ʱӲ����λ
#define CAN_IF_WKUP_Msk				(0x01 << CAN_IF_WKUP_Pos)
#define CAN_IF_ERRPASS_Pos			5		//
#define CAN_IF_ERRPASS_Msk			(0x01 << CAN_IF_ERRPASS_Pos)
#define CAN_IF_ARBLOST_Pos			6		//Arbitration Lost����IE.ARBLOST=1ʱ��CAN��������ʧ�ٲñ�ɽ��շ�ʱӲ����λ
#define CAN_IF_ARBLOST_Msk			(0x01 << CAN_IF_ARBLOST_Pos)
#define CAN_IF_BUSERR_Pos			7		//��IE.BUSERR=1ʱ��CAN��������⵽���ߴ���ʱӲ����λ
#define CAN_IF_BUSERR_Msk			(0x01 << CAN_IF_BUSERR_Pos)

#define CAN_IE_RXDA_Pos				0
#define CAN_IE_RXDA_Msk				(0x01 << CAN_IE_RXDA_Pos)
#define CAN_IE_TXBR_Pos				1
#define CAN_IE_TXBR_Msk				(0x01 << CAN_IE_TXBR_Pos)
#define CAN_IE_ERRWARN_Pos			2
#define CAN_IE_ERRWARN_Msk			(0x01 << CAN_IE_ERRWARN_Pos)
#define CAN_IE_RXOV_Pos				3
#define CAN_IE_RXOV_Msk				(0x01 << CAN_IE_RXOV_Pos)
#define CAN_IE_WKUP_Pos				4
#define CAN_IE_WKUP_Msk				(0x01 << CAN_IE_WKUP_Pos)
#define CAN_IE_ERRPASS_Pos			5
#define CAN_IE_ERRPASS_Msk			(0x01 << CAN_IE_ERRPASS_Pos)
#define CAN_IE_ARBLOST_Pos			6
#define CAN_IE_ARBLOST_Msk			(0x01 << CAN_IE_ARBLOST_Pos)
#define CAN_IE_BUSERR_Pos			7
#define CAN_IE_BUSERR_Msk			(0x01 << CAN_IE_BUSERR_Pos)

#define CAN_BT2_BRP_Pos				0
#define CAN_BT2_BRP_Msk				(0x0F << CAN_BT2_BRP_Pos)

#define CAN_BT0_BRP_Pos				0		//Baud Rate Prescaler��CANʱ�䵥λ=2*Tsysclk*((BT2.BRP<<6) + BT0.BRP + 1)
#define CAN_BT0_BRP_Msk				(0x3F << CAN_BT0_BRP_Pos)
#define CAN_BT0_SJW_Pos				6		//Synchronization Jump Width
#define CAN_BT0_SJW_Msk				(0x03 << CAN_BT0_SJW_Pos)

#define CAN_BT1_TSEG1_Pos			0		//t_tseg1 = CANʱ�䵥λ * (TSEG1+1)
#define CAN_BT1_TSEG1_Msk			(0x0F << CAN_BT1_TSEG1_Pos)
#define CAN_BT1_TSEG2_Pos			4		//t_tseg2 = CANʱ�䵥λ * (TSEG2+1)
#define CAN_BT1_TSEG2_Msk			(0x07 << CAN_BT1_TSEG2_Pos)
#define CAN_BT1_SAM_Pos				7		//��������  0: sampled once  1: sampled three times
#define CAN_BT1_SAM_Msk				(0x01 << CAN_BT1_SAM_Pos)

#define CAN_ECC_SEGCODE_Pos			0		//Segment Code
#define CAN_ECC_SEGCODE_Msk			(0x1F << CAN_ECC_SEGCODE_Pos)
#define CAN_ECC_DIR_Pos				5		//0 error occurred during transmission   1 during reception
#define CAN_ECC_DIR_Msk				(0x01 << CAN_ECC_DIR_Pos)
#define CAN_ECC_ERRCODE_Pos			6		//Error Code��0 Bit error   1 Form error   2 Stuff error   3 other error
#define CAN_ECC_ERRCODE_Msk			(0x03 << CAN_ECC_ERRCODE_Pos)

#define CAN_INFO_DLC_Pos			0		//Data Length Control
#define CAN_INFO_DLC_Msk			(0x0F << CAN_INFO_DLC_Pos)
#define CAN_INFO_RTR_Pos			6		//Remote Frame��1 Զ��֡    0 ����֡
#define CAN_INFO_RTR_Msk			(0x01 << CAN_INFO_RTR_Pos)
#define CAN_INFO_FF_Pos				7		//Frame Format��0 ��׼֡��ʽ    1 ��չ֡��ʽ
#define CAN_INFO_FF_Msk				(0x01 << CAN_INFO_FF_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t ADDR;						//programming address
	
	__IO uint32_t ERASE;
	
	__IO uint32_t PROGEN;					//[0] programming enable
	
	__IO uint32_t CFG0;
	
	__IO uint32_t CFG1;
	
	__IO uint32_t STAT;
	
	     uint32_t RESERVED[3];
	
	__IO uint32_t REMAP;
} FMC_TypeDef;


#define FMC_ERASE_ADDR_Pos			0
#define FMC_ERASE_ADDR_Msk			(0x1FFFF<< FMC_ERASE_ADDR_Pos)
#define FMC_ERASE_REQ_Pos			24
#define FMC_ERASE_REQ_Msk			(0xFFu<< FMC_ERASE_REQ_Pos)

#define FMC_STAT_ERASEBUSY_Pos		0
#define FMC_STAT_ERASEBUSY_Msk		(0x01 << FMC_STAT_ERASEBUSY_Pos)
#define FMC_STAT_PROGBUSY_Pos		1
#define FMC_STAT_PROGBUSY_Msk		(0x01 << FMC_STAT_PROGBUSY_Pos)
#define FMC_STAT_READBUSY_Pos		2
#define FMC_STAT_READBUSY_Msk		(0x01 << FMC_STAT_READBUSY_Pos)
#define FMC_STAT_FIFOEMPTY_Pos		3
#define FMC_STAT_FIFOEMPTY_Msk		(0x01 << FMC_STAT_FIFOEMPTY_Pos)
#define FMC_STAT_FIFOFULL_Pos		4
#define FMC_STAT_FIFOFULL_Msk		(0x01 << FMC_STAT_FIFOFULL_Pos)
#define FMC_STAT_FLASHIDLE_Pos		31
#define FMC_STAT_FLASHIDLE_Msk		(0x01u<< FMC_STAT_FLASHIDLE_Pos)

#define FMC_REMAP_ON_Pos			0
#define FMC_REMAP_ON_Msk			(0x01 << FMC_REMAP_ON_Pos)
#define FMC_REMAP_OFFSET_Pos		1		//��0x000-0x800��2K��ַ�ķ���ӳ�䵽2K*OFFSET-2K*(OFFSET+1)��ַ��
#define FMC_REMAP_OFFSET_Msk		(0x3F << FMC_REMAP_OFFSET_Pos)




typedef struct {
	__IO uint32_t RSTVAL;					//��������������ֵʱ������λ
	
	__IO uint32_t INTVAL;					//��������������ֵʱ�����ж�
	
	__IO uint32_t CR;
	
	__IO uint32_t IF;						//[0] �жϱ�־��д1����
	
	__IO uint32_t FEED;						//д0x55ι��
} WDT_TypeDef;


#define WDT_CR_EN_Pos				0
#define WDT_CR_EN_Msk				(0x01 << WDT_CR_EN_Pos)
#define WDT_CR_RSTEN_Pos			1
#define WDT_CR_RSTEN_Msk			(0x01 << WDT_CR_RSTEN_Pos)
#define WDT_CR_INTEN_Pos			2
#define WDT_CR_INTEN_Msk			(0x01 << WDT_CR_INTEN_Pos)
#define WDT_CR_WINEN_Pos			3		//���ڹ���ʹ�ܣ�ʹ�ܺ���WDT����ֵС��INTVALʱι��������λ
#define WDT_CR_WINEN_Msk			(0x01 << WDT_CR_WINEN_Pos)
#define WDT_CR_CLKDIV_Pos			8
#define WDT_CR_CLKDIV_Msk			(0x0F << WDT_CR_CLKDIV_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t SR;
	
	     uint32_t RESERVED[2];
	
	__IO uint32_t DIVIDEND;					//������
	
	__IO uint32_t DIVISOR;					//����
	
	__IO uint32_t QUO;						//��
	
	__IO uint32_t REMAIN;					//����
	
	__IO uint32_t RADICAND;					//��������
	
	__IO uint32_t ROOT;						//ƽ��������16λΪС�����֣���16λΪ��������
} DIV_TypeDef;


#define DIV_CR_DIVGO_Pos			0		//д1�����������㣬������ɺ��Զ�����
#define DIV_CR_DIVGO_Msk			(0x01 << DIV_CR_DIVGO_Pos)
#define DIV_CR_DIVSIGN_Pos			1		//0 �з��ų���    1 �޷��ų���
#define DIV_CR_DIVSIGN_Msk			(0x01 << DIV_CR_DIVSIGN_Pos)
#define DIV_CR_ROOTGO_Pos			8		//д1������ƽ�������㣬������ɺ��Զ�����
#define DIV_CR_ROOTGO_Msk			(0x01 << DIV_CR_ROOTGO_Pos)
#define DIV_CR_ROOTMOD_Pos			9		//��ƽ����ģʽ��0 ���Ϊ����    1 �������������������С������
#define DIV_CR_ROOTMOD_Msk			(0x01 << DIV_CR_ROOTMOD_Pos)

#define DIV_SR_DIVEND_Pos			0		//����������ɱ�־��д1����
#define DIV_SR_DIVEND_Msk			(0x01 << DIV_SR_DIVEND_Pos)
#define DIV_SR_DIVBUSY_Pos			1		//1 �������������
#define DIV_SR_DIVBUSY_Msk			(0x01 << DIV_SR_DIVBUSY_Pos)
#define DIV_SR_ROOTENDI_Pos			8		//��������������ɱ�־��д1����
#define DIV_SR_ROOTENDI_Msk			(0x01 << DIV_SR_ROOTENDI_Pos)
#define DIV_SR_ROOTENDF_Pos			9		//����С��������ɱ�־��д1����
#define DIV_SR_ROOTENDF_Msk			(0x01 << DIV_SR_ROOTENDF_Pos)
#define DIV_SR_ROOTBUSY_Pos			10		//1 �������������
#define DIV_SR_ROOTBUSY_Msk			(0x01 << DIV_SR_ROOTBUSY_Pos)




typedef struct {
    __IO uint32_t CR;
    
	__O  uint32_t DATAIN;
	
    __IO uint32_t INIVAL;
    
    __I  uint32_t RESULT;
} CRC_TypeDef;


#define CRC_CR_EN_Pos			    0
#define CRC_CR_EN_Msk			    (0x01 << CRC_CR_EN_Pos)
#define CRC_CR_IREV_Pos				1       //���������Ƿ�ת��0 bit˳�򲻱�    1 bit˳����ȫ��ת   2 bit˳���ֽ��ڷ�ת   3 ���ֽ�˳��ת
#define CRC_CR_IREV_Msk				(0x03 << CRC_CR_IREV_Pos)
#define CRC_CR_INOT_Pos				3       //���������Ƿ�ȡ��
#define CRC_CR_INOT_Msk				(0x01 << CRC_CR_INOT_Pos)
#define CRC_CR_OREV_Pos				4       //�������Ƿ�ת��0 bit˳�򲻱�    1 bit˳����ȫ��ת   2 bit˳���ֽ��ڷ�ת   3 ���ֽ�˳��ת
#define CRC_CR_OREV_Msk				(0x03 << CRC_CR_OREV_Pos)
#define CRC_CR_ONOT_Pos				6       //�������Ƿ�ȡ��
#define CRC_CR_ONOT_Msk				(0x01 << CRC_CR_ONOT_Pos)
#define CRC_CR_POLY_Pos				7       //����ʽѡ��0 x^16+x^12+x^5+1   1 x^8+x^2+x+1   2 x^16+x^15+x^2+1   3 x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1
#define CRC_CR_POLY_Msk				(0x03 << CRC_CR_POLY_Pos)
#define CRC_CR_IBIT_Pos				9       //����������Чλ�� 0 32λ    1 16λ    2 8λ
#define CRC_CR_IBIT_Msk				(0x03 << CRC_CR_IBIT_Pos)




typedef struct {
    __IO uint32_t MINSEC;                   //�������
    
    __IO uint32_t DATHUR;                   //��ʱ����
    
    __IO uint32_t MONDAY;                   //���ܼ���
    
    __IO uint32_t YEAR;                     //[11:0] �����
    
    __IO uint32_t MINSECAL;                 //������������
    
    __IO uint32_t DAYHURAL;                 //��ʱ��������
    
    __IO uint32_t LOAD;
    
    __IO uint32_t IE;
    
    __IO uint32_t IF;                       //д1����
    
    __IO uint32_t EN;                       //[0] 1 RTCʹ��
    
    __IO uint32_t CFGABLE;                  //[0] 1 RTC������
    
    __IO uint32_t TRIM;                     //ʱ�ӵ���
    
    __IO uint32_t TRIMM;                    //ʱ��΢����
} RTC_TypeDef;


#define RTC_MINSEC_SEC_Pos			0       //�������ȡֵ0--59
#define RTC_MINSEC_SEC_Msk		    (0x3F << RTC_MINSEC_SEC_Pos)
#define RTC_MINSEC_MIN_Pos			6       //���Ӽ�����ȡֵ0--59
#define RTC_MINSEC_MIN_Msk		    (0x3F << RTC_MINSEC_MIN_Pos)

#define RTC_DATHUR_HOUR_Pos			0       //Сʱ������ȡֵ0--23
#define RTC_DATHUR_HOUR_Msk		    (0x1F << RTC_DATHUR_HOUR_Pos)
#define RTC_DATHUR_DATE_Pos			5       //date of month��ȡֵ1--31
#define RTC_DATHUR_DATE_Msk		    (0x1F << RTC_DATHUR_DATE_Pos)

#define RTC_MONDAY_DAY_Pos			0       //day of week��ȡֵ0--6
#define RTC_MONDAY_DAY_Msk		    (0x07 << RTC_MONDAY_DAY_Pos)
#define RTC_MONDAY_MON_Pos			3       //�·ݼ�����ȡֵ1--12
#define RTC_MONDAY_MON_Msk		    (0x0F << RTC_MONDAY_MON_Pos)

#define RTC_MINSECAL_SEC_Pos		0       //����������
#define RTC_MINSECAL_SEC_Msk		(0x3F << RTC_MINSECAL_SEC_Pos)
#define RTC_MINSECAL_MIN_Pos	    6       //���ӷ�������
#define RTC_MINSECAL_MIN_Msk		(0x3F << RTC_MINSECAL_MIN_Pos)

#define RTC_DAYHURAL_HOUR_Pos		0       //����Сʱ����
#define RTC_DAYHURAL_HOUR_Msk		(0x1F << RTC_DAYHURAL_HOUR_Pos)
#define RTC_DAYHURAL_SUN_Pos		5       //����������Ч
#define RTC_DAYHURAL_SUN_Msk		(0x01 << RTC_DAYHURAL_SUN_Pos)
#define RTC_DAYHURAL_MON_Pos		6       //��һ������Ч
#define RTC_DAYHURAL_MON_Msk		(0x01 << RTC_DAYHURAL_MON_Pos)
#define RTC_DAYHURAL_TUE_Pos		7       //�ܶ�������Ч
#define RTC_DAYHURAL_TUE_Msk		(0x01 << RTC_DAYHURAL_TUE_Pos)
#define RTC_DAYHURAL_WED_Pos		8       //����������Ч
#define RTC_DAYHURAL_WED_Msk		(0x01 << RTC_DAYHURAL_WED_Pos)
#define RTC_DAYHURAL_THU_Pos		9       //����������Ч
#define RTC_DAYHURAL_THU_Msk		(0x01 << RTC_DAYHURAL_THU_Pos)
#define RTC_DAYHURAL_FRI_Pos		10      //����������Ч
#define RTC_DAYHURAL_FRI_Msk		(0x01 << RTC_DAYHURAL_FRI_Pos)
#define RTC_DAYHURAL_SAT_Pos		11      //����������Ч
#define RTC_DAYHURAL_SAT_Msk		(0x01 << RTC_DAYHURAL_SAT_Pos)

#define RTC_LOAD_TIME_Pos			0		//��ʱ�䡢��ʱ��У׼��ؼĴ����е�ֵ���ص�RTCʱ����
#define RTC_LOAD_TIME_Msk			(0x01 << RTC_LOAD_TIME_Pos)
#define RTC_LOAD_ALARM_Pos			1		//��������ؼĴ����е�ֵ���ص�RTCʱ����
#define RTC_LOAD_ALARM_Msk			(0x01 << RTC_LOAD_ALARM_Pos)

#define RTC_IE_SEC_Pos		        0       //���ж�ʹ��
#define RTC_IE_SEC_Msk		        (0x01 << RTC_IE_SEC_Pos)
#define RTC_IE_MIN_Pos		        1
#define RTC_IE_MIN_Msk		        (0x01 << RTC_IE_MIN_Pos)
#define RTC_IE_HOUR_Pos		        2
#define RTC_IE_HOUR_Msk		        (0x01 << RTC_IE_HOUR_Pos)
#define RTC_IE_DATE_Pos		        3
#define RTC_IE_DATE_Msk		        (0x01 << RTC_IE_DATE_Pos)
#define RTC_IE_ALARM_Pos		    4
#define RTC_IE_ALARM_Msk		    (0x01 << RTC_IE_ALARM_Pos)
#define RTC_IE_TRIM_Pos				5		//У׼����ж�ʹ��
#define RTC_IE_TRIM_Msk				(0x01 << RTC_IE_TRIM_Pos)
#define RTC_IE_HSEC_Pos				6		//�����ж�ʹ��
#define RTC_IE_HSEC_Msk				(0x01 << RTC_IE_HSEC_Pos)
#define RTC_IE_QSEC_Pos				7		//�ķ�֮һ���ж�ʹ��
#define RTC_IE_QSEC_Msk				(0x01 << RTC_IE_QSEC_Pos)

#define RTC_IF_SEC_Pos		        0       //д1����
#define RTC_IF_SEC_Msk		        (0x01 << RTC_IF_SEC_Pos)
#define RTC_IF_MIN_Pos		        1
#define RTC_IF_MIN_Msk		        (0x01 << RTC_IF_MIN_Pos)
#define RTC_IF_HOUR_Pos		        2
#define RTC_IF_HOUR_Msk		        (0x01 << RTC_IF_HOUR_Pos)
#define RTC_IF_DATE_Pos		        3
#define RTC_IF_DATE_Msk		        (0x01 << RTC_IF_DATE_Pos)
#define RTC_IF_ALARM_Pos		    4
#define RTC_IF_ALARM_Msk		    (0x01 << RTC_IF_ALARM_Pos)
#define RTC_IF_TRIM_Pos				5
#define RTC_IF_TRIM_Msk				(0x01 << RTC_IF_TRIM_Pos)
#define RTC_IF_HSEC_Pos				6
#define RTC_IF_HSEC_Msk				(0x01 << RTC_IF_HSEC_Pos)
#define RTC_IF_QSEC_Pos				7
#define RTC_IF_QSEC_Msk				(0x01 << RTC_IF_QSEC_Pos)

#define RTC_TRIM_ADJ_Pos		    0       //���ڵ���BASECNT�ļ������ڣ�Ĭ��Ϊ32768�����DECΪ1����������ڵ���Ϊ32768-ADJ���������Ϊ32768+ADJ
#define RTC_TRIM_ADJ_Msk		    (0xFF << RTC_TRIM_ADJ_Pos)
#define RTC_TRIM_DEC_Pos		    8
#define RTC_TRIM_DEC_Msk		    (0x01 << RTC_TRIM_DEC_Pos)

#define RTC_TRIMM_CYCLE_Pos		    0       //���ڼ�������΢�������INCΪ1�����n���������ڵ���Ϊ(32768��ADJ)+1,�������Ϊ(32768��ADJ)-1
                                            //cycles=0ʱ��������΢������cycles=1����nΪ2��cycles=7����nΪ8���Դ�����
#define RTC_TRIMM_CYCLE_Msk		    (0x07 << RTC_TRIMM_CYCLE_Pos)
#define RTC_TRIMM_INC_Pos		    3
#define RTC_TRIMM_INC_Msk		    (0x01 << RTC_TRIMM_INC_Pos)




typedef struct {
	__IO uint32_t CR;
	
	     uint32_t RESERVED[3];
	
	__IO uint32_t DATA[4];
} SLCD_TypeDef;


#define SLCD_CR_DRIVEN_Pos			0		//������·ʹ��		
#define SLCD_CR_DRIVEN_Msk			(0x01 << SLCD_CR_DRIVEN_Pos)
#define SLCD_CR_SCANEN_Pos			1		//ɨ���·ʹ��
#define SLCD_CR_SCANEN_Msk			(0x01 << SLCD_CR_SCANEN_Pos)
#define SLCD_CR_DISP_Pos			2		//0 ������ʾ    1 ȫ���ر�    2 ȫ����ʾ	
#define SLCD_CR_DISP_Msk			(0x03 << SLCD_CR_DISP_Pos)
#define SLCD_CR_BIAS_Pos			4		//0 1/3 bias    1 1/2 bias
#define SLCD_CR_BIAS_Msk			(0x01 << SLCD_CR_BIAS_Pos)
#define SLCD_CR_DUTY_Pos			5		//0 1/4 duty    1 1/3 duty
#define SLCD_CR_DUTY_Msk			(0x01 << SLCD_CR_DUTY_Pos)
#define SLCD_CR_SCANFRQ_Pos			6		//֡ɨ��Ƶ�� 0 32Hz   1 64Hz   2 128Hz   3 256Hz
#define SLCD_CR_SCANFRQ_Msk			(0x03 << SLCD_CR_SCANFRQ_Pos)
#define SLCD_CR_DRIVSEL_Pos			8		//�������� 0 8uA   1 25uA   2 50uA   3 100uA
#define SLCD_CR_DRIVSEL_Msk			(0x03 << SLCD_CR_CUR_SEL_Pos)
#define SLCD_CR_KEYSCAN_Pos			10		//����ɨ�蹦��ʹ��
#define SLCD_CR_KEYSCAN_Msk			(0x01 << SLCD_CR_KEYSCAN_Pos)
#define SLCD_CR_CLKDIV_Pos			16
#define SLCD_CR_CLKDIV_Msk			(0x3F << SLCD_CR_CLKDIV_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t CLKDIV;					//[3:0] 2��CLKDIV���ݷ�Ƶ
	
	__IO uint32_t TIM;
	
	     uint32_t RESERVED;
	
	__IO uint32_t DATA[8];
} SLED_TypeDef;


#define SLED_CR_EN_Pos				0
#define SLED_CR_EN_Msk				(0x01 << SLED_CR_EN_Pos)
#define SLED_CR_DUTY_Pos			1		//0 1/4 Duty    1 1/8 Duty
#define SLED_CR_DUTY_Msk			(0x01 << SLED_CR_DUTY_Pos)
#define SLED_CR_COMINV_Pos			31		//1 COM ���η���
#define SLED_CR_COMINV_Msk			(0x01u<< SLED_CR_COMINV_Pos)

#define SLED_TIM_PERIOD_Pos			0		//COM����ʱ�䳤��
#define SLED_TIM_PERIOD_Msk			(0x3FF<< SLED_TIM_PERIOD_Pos)
#define SLED_TIM_HIGH_Pos			16		//COM������COM����ߵ�ƽ��ʱ�䳤��
#define SLED_TIM_HIGH_Msk			(0x3FF<< SLED_TIM_HIGH_Pos)




typedef struct {
	__IO uint32_t PERWP;					//Peripheral Write Protect
	
	__IO uint32_t RAMWP;					//RAM Write Protect
	
	__IO uint32_t IAACR;					//Illegal Address Access Contorl Register
	
	__IO uint32_t IF;
	
	__IO uint32_t IE;
	
		 uint32_t RESERVED[3];
	
	struct {
		__IO uint32_t BADDR;				//Region Bottom Address
		
		__IO uint32_t TADDR;				//Region Top Address
	} REGION[4];
} SAFETY_TypeDef;


#define SAFETY_PERWP_IAADEN_Pos		0		//Illegal Address Access Detection Enable
#define SAFETY_PERWP_IAADEN_Msk		(0x01 << SAFETY_PERWP_IAADEN_Pos)
#define SAFETY_PERWP_IER_Pos		1		//Interrupt Enable Registers Write-Protect Enable
#define SAFETY_PERWP_IER_Msk		(0x01 << SAFETY_PERWP_IER_Pos)
#define SAFETY_PERWP_IOCFGR_Pos		2		//IO Configure Registers Write-Protect Enable
#define SAFETY_PERWP_IOCFGR_Msk		(0x01 << SAFETY_PERWP_IOCFGR_Pos)
#define SAFETY_PERWP_CLKCFGR_Pos	3		//Clock Configure Registers Write-Protect Enable
#define SAFETY_PERWP_CLKCFGR_Msk	(0x01 << SAFETY_PERWP_CLKCFGR_Pos)
#define SAFETY_PERWP_ANACFGR_Pos	4		//Analog Configure Registers Write-Protect Enable
#define SAFETY_PERWP_ANACFGR_Msk	(0x01 << SAFETY_PERWP_ANACFGR_Pos)

#define SAFETY_RAMWP_SIZE_Pos		0		//��Ҫд�����Ŀռ��С��0 0 Byte   1 256 Bytes    2 512 Bytes    3 1024 Bytes
#define SAFETY_RAMWP_SIZE_Msk		(0x03 << SAFETY_RAMWP_SIZE_Pos)
#define SAFETY_RAMWP_ADDR_Pos		8		//��Ҫд�����Ŀռ��ַ����256�ֽ�Ϊ��λ
#define SAFETY_RAMWP_ADDR_Msk		(0x7F << SAFETY_RAMWP_ADDR_Pos)

#define SAFETY_IAACR_R0INT_Pos		0		//1 Region0 �Ƿ����ʲ����ж�    0 Region0 �Ƿ����ʲ�����λ
#define SAFETY_IAACR_R0INT_Msk		(0x01 << SAFETY_IAACR_R0INT_Pos)
#define SAFETY_IAACR_R1INT_Pos		1
#define SAFETY_IAACR_R1INT_Msk		(0x01 << SAFETY_IAACR_R1INT_Pos)
#define SAFETY_IAACR_R2INT_Pos		2
#define SAFETY_IAACR_R2INT_Msk		(0x01 << SAFETY_IAACR_R2INT_Pos)
#define SAFETY_IAACR_R3INT_Pos		3
#define SAFETY_IAACR_R3INT_Msk		(0x01 << SAFETY_IAACR_R3INT_Pos)

#define SAFETY_IF_R0_Pos			0		//Region0 �Ƿ����ʲ��жϱ�־
#define SAFETY_IF_R0_Msk			(0x01 << SAFETY_IF_R0_Pos)
#define SAFETY_IF_R1_Pos			1
#define SAFETY_IF_R1_Msk			(0x01 << SAFETY_IF_R1_Pos)
#define SAFETY_IF_R2_Pos			2
#define SAFETY_IF_R2_Msk			(0x01 << SAFETY_IF_R2_Pos)
#define SAFETY_IF_R3_Pos			3
#define SAFETY_IF_R3_Msk			(0x01 << SAFETY_IF_R3_Pos)

#define SAFETY_IE_R0_Pos			0		//Region0 �Ƿ����ʲ��ж�ʹ��
#define SAFETY_IE_R0_Msk			(0x01 << SAFETY_IE_R0_Pos)
#define SAFETY_IE_R1_Pos			1
#define SAFETY_IE_R1_Msk			(0x01 << SAFETY_IE_R1_Pos)
#define SAFETY_IE_R2_Pos			2
#define SAFETY_IE_R2_Msk			(0x01 << SAFETY_IE_R2_Pos)
#define SAFETY_IE_R3_Pos			3
#define SAFETY_IE_R3_Msk			(0x01 << SAFETY_IE_R3_Pos)




/******************************************************************************/
/*						 Peripheral memory map							  */
/******************************************************************************/
#define RAM_BASE		   	0x20000000
#define AHB_BASE			0x40000000
#define APB1_BASE		 	0x40040000
#define APB2_BASE			0x400A0000

/* AHB Peripheral memory map */
#define SYS_BASE			(AHB_BASE + 0x0000)
#define DMA_BASE			(AHB_BASE + 0x0800)
#define CRC_BASE			(AHB_BASE + 0x2800)
#define DIV_BASE			(AHB_BASE + 0x3800)

/* APB1 Peripheral memory map */
#define GPIOA_BASE			(APB1_BASE + 0x0000)
#define GPIOB_BASE			(APB1_BASE + 0x0800)
#define GPIOC_BASE			(APB1_BASE + 0x1000)
#define GPIOD_BASE			(APB1_BASE + 0x1800)

#define UART0_BASE			(APB1_BASE + 0x2000)
#define UART1_BASE			(APB1_BASE + 0x2800)
#define UART2_BASE			(APB1_BASE + 0x3000)
#define UART3_BASE			(APB1_BASE + 0x3800)

#define SPI0_BASE			(APB1_BASE + 0x4000)
#define SPI1_BASE			(APB1_BASE + 0x4800)

#define PWM0_BASE			(APB1_BASE + 0x6000)
#define PWM1_BASE			(APB1_BASE + 0x6040)
#define PWM2_BASE			(APB1_BASE + 0x6080)
#define PWM3_BASE			(APB1_BASE + 0x60C0)
#define PWMG_BASE			(APB1_BASE + 0x6200)

#define TIMR0_BASE			(APB1_BASE + 0x6800)
#define TIMR1_BASE			(APB1_BASE + 0x6840)
#define TIMR2_BASE			(APB1_BASE + 0x6880)
#define TIMR3_BASE			(APB1_BASE + 0x68C0)
#define TIMR4_BASE			(APB1_BASE + 0x6900)
#define TIMR5_BASE			(APB1_BASE + 0x6940)
#define TIMR6_BASE			(APB1_BASE + 0x6980)
#define TIMR7_BASE			(APB1_BASE + 0x69C0)
#define TIMRG_BASE			(APB1_BASE + 0x6C00)

#define ADC_BASE			(APB1_BASE + 0x9000)

#define FMC_BASE			(APB1_BASE + 0xA000)

#define RTC_BASE			(APB1_BASE + 0xB800)

/* APB2 Peripheral memory map */
#define PORTA_BASE			(APB2_BASE + 0x0000)
#define PORTB_BASE			(APB2_BASE + 0x0010)
#define PORTC_BASE			(APB2_BASE + 0x0020)
#define PORTD_BASE			(APB2_BASE + 0x0030)

#define WDT_BASE			(APB2_BASE + 0x0800)

#define I2C0_BASE			(APB2_BASE + 0x6000)
#define I2C1_BASE			(APB2_BASE + 0x6800)

#define CAN_BASE			(APB2_BASE + 0x8000)

#define SLCD_BASE			(APB2_BASE + 0x9800)

#define SLED_BASE			(APB2_BASE + 0xA800)

#define SAFETY_BASE			(APB2_BASE + 0xB000)



/******************************************************************************/
/*						 Peripheral declaration							 */
/******************************************************************************/
#define SYS					((SYS_TypeDef  *) SYS_BASE)

#define PORTA				((PORT_TypeDef *) PORTA_BASE)
#define PORTB				((PORT_TypeDef *) PORTB_BASE)
#define PORTC				((PORT_TypeDef *) PORTC_BASE)
#define PORTD				((PORT_TypeDef *) PORTD_BASE)

#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB				((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC				((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD				((GPIO_TypeDef *) GPIOD_BASE)

#define TIMR0				((TIMR_TypeDef *) TIMR0_BASE)
#define TIMR1				((TIMR_TypeDef *) TIMR1_BASE)
#define TIMR2				((TIMR_TypeDef *) TIMR2_BASE)
#define TIMR3				((TIMR_TypeDef *) TIMR3_BASE)
#define TIMR4				((TIMR_TypeDef *) TIMR4_BASE)
#define TIMR5				((TIMR_TypeDef *) TIMR5_BASE)
#define TIMR6				((TIMR_TypeDef *) TIMR6_BASE)
#define TIMR7				((TIMR_TypeDef *) TIMR7_BASE)
#define TIMRG				((TIMRG_TypeDef*) TIMRG_BASE)

#define UART0				((UART_TypeDef *) UART0_BASE)
#define UART1				((UART_TypeDef *) UART1_BASE)
#define UART2				((UART_TypeDef *) UART2_BASE)
#define UART3   			((UART_TypeDef *) UART3_BASE)

#define SPI0				((SPI_TypeDef  *) SPI0_BASE)
#define SPI1				((SPI_TypeDef  *) SPI1_BASE)

#define I2C0				((I2C_TypeDef  *) I2C0_BASE)
#define I2C1				((I2C_TypeDef  *) I2C1_BASE)

#define ADC 				((ADC_TypeDef  *) ADC_BASE)

#define PWM0				((PWM_TypeDef  *) PWM0_BASE)
#define PWM1				((PWM_TypeDef  *) PWM1_BASE)
#define PWM2				((PWM_TypeDef  *) PWM2_BASE)
#define PWM3				((PWM_TypeDef  *) PWM3_BASE)
#define PWMG				((PWMG_TypeDef *) PWMG_BASE)

#define DMA 				((DMA_TypeDef  *) DMA_BASE)

#define CAN					((CAN_TypeDef  *) CAN_BASE)

#define FMC					((FMC_TypeDef  *) FMC_BASE)

#define WDT					((WDT_TypeDef  *) WDT_BASE)

#define	DIV					((DIV_TypeDef  *) DIV_BASE)

#define CRC					((CRC_TypeDef  *) CRC_BASE)

#define	RTC					((RTC_TypeDef  *) RTC_BASE)

#define SLCD				((SLCD_TypeDef *) SLCD_BASE)

#define SLED				((SLED_TypeDef *) SLED_BASE)

#define SAFETY				((SAFETY_TypeDef*)SAFETY_BASE)


#include <stdbool.h>

#include "SWM241_port.h"
#include "SWM241_gpio.h"
#include "SWM241_exti.h"
#include "SWM241_timr.h"
#include "SWM241_uart.h"
#include "SWM241_spi.h"
#include "SWM241_i2c.h"
#include "SWM241_pwm.h"
#include "SWM241_adc.h"
#include "SWM241_dma.h"
#include "SWM241_can.h"
#include "SWM241_flash.h"
#include "SWM241_wdt.h"
#include "SWM241_rtc.h"
#include "SWM241_div.h"
#include "SWM241_crc.h"
#include "SWM241_slcd.h"
#include "SWM241_sled.h"
#include "SWM241_sleep.h"
#include "SWM241_safety.h"


#endif //__SWM241_H__
