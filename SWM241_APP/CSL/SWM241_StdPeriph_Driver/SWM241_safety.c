/****************************************************************************************************************************************** 
* �ļ�����:	SWM241_safety.c
* ����˵��:	SWM241��Ƭ���İ�ȫ����������
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����: V1.0.0		2016��1��30��
* ������¼: 
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology  
*******************************************************************************************************************************************/
#include "SWM241.h"
#include "SWM241_safety.h"


/****************************************************************************************************************************************** 
* ��������: SAFETY_PerWP_Open()
* ����˵��:	����д��������
* ��    ��: uint32_t peripherals	SAFETY_WP_IER��SAFETY_WP_IOCFGR��SAFETY_WP_CLKCFGR��SAFETY_WP_ANACFGR ���� ��λ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_PerWP_Open(uint32_t peripherals)
{
	SAFETY->PERWP |= (peripherals << SAFETY_PERWP_IER_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_PerWP_Close()
* ����˵��:	����д�����ر�
* ��    ��: uint32_t peripherals	SAFETY_WP_IER��SAFETY_WP_IOCFGR��SAFETY_WP_CLKCFGR��SAFETY_WP_ANACFGR ���� ��λ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_PerWP_Close(uint32_t peripherals)
{
	SAFETY->PERWP &= ~(peripherals << SAFETY_PERWP_IER_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_RAMWP_Open()
* ����˵��:	RAM д��������
* ��    ��: uint32_t addr	��Ҫд������RAM����ʼ��ַ
*			uint32_t size	��Ҫд������RAM�Ĵ�С����ȡֵ��SAFETY_SIZE_256B��SAFETY_SIZE_256B��SAFETY_SIZE_1024B
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_RAMWP_Open(uint32_t addr, uint32_t size)
{
	addr = (addr - 0x20000000) / 256;
	
	SAFETY->RAMWP = (addr << SAFETY_RAMWP_ADDR_Pos) |
					(size << SAFETY_RAMWP_SIZE_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_RAMWP_Close()
* ����˵��:	RAM д�����ر�
* ��    ��: ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_RAMWP_Close(void)
{
	SAFETY->RAMWP = 0;
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_IAAD_Open()
* ����˵��:	�Ƿ���ַ���ʼ�⣨Illegal Address Access Detection������
* ��    ��: ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAAD_Open(void)
{
	SAFETY->PERWP |= (1 << SAFETY_PERWP_IAADEN_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_IAAD_Close()
* ����˵��:	�Ƿ���ַ���ʼ�⣨Illegal Address Access Detection���ر�
* ��    ��: ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAAD_Close(void)
{
	SAFETY->PERWP &= ~(1 << SAFETY_PERWP_IAADEN_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_IAAD_Close()
* ����˵��:	�Ƿ���ַ���ʼ�⣨Illegal Address Access Detection����������
* ��    ��: uint32_t region		��Ҫ���õ����򣬿�ȡֵ��SAFETY_REGION_0��SAFETY_REGION_1��SAFETY_REGION_2��SAFETY_REGION_3
*			SAFETY_Region_InitStructure * initStruct	���������ֵ
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAADRegion_Config(uint32_t region, SAFETY_Region_InitStructure * initStruct)
{
	SAFETY->REGION[region].BADDR = initStruct->bottom_addr;
	SAFETY->REGION[region].TADDR = initStruct->top_addr;
	
	switch(region)
	{
	case SAFETY_REGION_0:
		SAFETY->IAACR &= ~SAFETY_IAACR_R0INT_Msk;
		SAFETY->IAACR |= (initStruct->action << SAFETY_IAACR_R0INT_Pos);
	
		SAFETY->IF = (1 << SAFETY_IF_R0_Pos);
		if(initStruct->inten)
			SAFETY->IE |=  (1 << SAFETY_IE_R0_Pos);
		else
			SAFETY->IE &= ~(1 << SAFETY_IE_R0_Pos);
		break;
	
	case SAFETY_REGION_1:
		SAFETY->IAACR &= ~SAFETY_IAACR_R1INT_Msk;
		SAFETY->IAACR |= (initStruct->action << SAFETY_IAACR_R1INT_Pos);
	
		SAFETY->IF = (1 << SAFETY_IF_R1_Pos);
		if(initStruct->inten)
			SAFETY->IE |=  (1 << SAFETY_IE_R1_Pos);
		else
			SAFETY->IE &= ~(1 << SAFETY_IE_R1_Pos);
		break;
	
	case SAFETY_REGION_2:
		SAFETY->IAACR &= ~SAFETY_IAACR_R2INT_Msk;
		SAFETY->IAACR |= (initStruct->action << SAFETY_IAACR_R2INT_Pos);
	
		SAFETY->IF = (1 << SAFETY_IF_R2_Pos);
		if(initStruct->inten)
			SAFETY->IE |=  (1 << SAFETY_IE_R2_Pos);
		else
			SAFETY->IE &= ~(1 << SAFETY_IE_R2_Pos);
		break;
	
	case SAFETY_REGION_3:
		SAFETY->IAACR &= ~SAFETY_IAACR_R3INT_Msk;
		SAFETY->IAACR |= (initStruct->action << SAFETY_IAACR_R3INT_Pos);
	
		SAFETY->IF = (1 << SAFETY_IF_R3_Pos);
		if(initStruct->inten)
			SAFETY->IE |=  (1 << SAFETY_IE_R3_Pos);
		else
			SAFETY->IE &= ~(1 << SAFETY_IE_R3_Pos);
		break;
	}
	
	if(initStruct->inten)
		NVIC_EnableIRQ(PWM3_GPIOC_SAFETY_IRQn);
}


/****************************************************************************************************************************************** 
* ��������:	SAFETY_IAAD_INTEn()
* ����˵��:	�Ƿ���ַ���ʼ���ж�ʹ��
* ��    ��: uint32_t region			��Ҫ���õ����򣬿�ȡֵ��SAFETY_REGION_0��SAFETY_REGION_1��SAFETY_REGION_2��SAFETY_REGION_3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAAD_INTEn(uint32_t region)
{
	SAFETY->IE |= (1 << region);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_IAAD_INTDis()
* ����˵��:	�Ƿ���ַ���ʼ���жϽ���
* ��    ��: uint32_t region			��Ҫ���õ����򣬿�ȡֵ��SAFETY_REGION_0��SAFETY_REGION_1��SAFETY_REGION_2��SAFETY_REGION_3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAAD_INTDis(uint32_t region)
{
	SAFETY->IE &= ~(1 << region);
}

/****************************************************************************************************************************************** 
* ��������:	SAFETY_IAAD_INTClr()
* ����˵��:	�Ƿ���ַ���ʼ���жϱ�־���
* ��    ��: uint32_t region			��Ҫ���õ����򣬿�ȡֵ��SAFETY_REGION_0��SAFETY_REGION_1��SAFETY_REGION_2��SAFETY_REGION_3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SAFETY_IAAD_INTClr(uint32_t region)
{
	SAFETY->IF = (1 << region);
}

/****************************************************************************************************************************************** 
* ��������: SAFETY_IAAD_INTStat()
* ����˵��: �Ƿ���ַ���ʼ���ж�״̬
* ��    ��: uint32_t region			��Ҫ���õ����򣬿�ȡֵ��SAFETY_REGION_0��SAFETY_REGION_1��SAFETY_REGION_2��SAFETY_REGION_3
* ��    ��: uint32_t 				0 δ�����ж�    1 �������ж�
* ע������: ��
******************************************************************************************************************************************/
uint32_t SAFETY_IAAD_INTStat(uint32_t region)
{
	return (SAFETY->IF >> region) & 1;
}
