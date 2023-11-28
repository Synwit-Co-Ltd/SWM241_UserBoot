/****************************************************************************************************************************************** 
* �ļ�����:	SWM241_sled.c
* ����˵��:	SWM241��Ƭ����Segment LED����������
* ����֧��: http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����: V1.0.0		2016��1��30��
* ������¼:  
*
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
#include "SWM241_sled.h"


/****************************************************************************************************************************************** 
* ��������: SLED_Init()
* ����˵��:	Segment LEDģ���ʼ��
* ��    ��: SLED_TypeDef * SLEDx			ָ��Ҫ���õ�SLEDģ�飬��Чֵ����SLED
*			SLED_InitStructure * initStruct	����SLED����趨ֵ�Ľṹ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SLED_Init(SLED_TypeDef * SLEDx, SLED_InitStructure * initStruct)
{
	SYS->CLKEN0 |= (0x01u << SYS_CLKEN0_SLED_Pos);
	
	SLEDx->CR &= ~(SLED_CR_DUTY_Msk | SLED_CR_COMINV_Msk);
	SLEDx->CR |= (initStruct->duty   << SLED_CR_DUTY_Pos) |
				 (initStruct->cominv << SLED_CR_COMINV_Pos);
	
	SLEDx->CLKDIV = initStruct->clkdiv;
	
	SLEDx->TIM = (((initStruct->period > 2) ? (initStruct->period - 1) : 2) << SLED_TIM_PERIOD_Pos) |
				 (((initStruct->high > 1) ? (initStruct->high - 1) : 1) << SLED_TIM_HIGH_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SLED_Start()
* ����˵��:	Segment LEDģ�鿪ʼ����
* ��    ��: SLED_TypeDef * SLEDx		ָ��Ҫ���õ�SLEDģ�飬��Чֵ����SLED
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SLED_Start(SLED_TypeDef * SLEDx)
{
	SLEDx->CR |= (1 << SLED_CR_EN_Pos);
}

/****************************************************************************************************************************************** 
* ��������: SLED_Stop()
* ����˵��:	Segment LEDģ��ֹͣ����
* ��    ��: SLED_TypeDef * SLEDx		ָ��Ҫ���õ�SLEDģ�飬��Чֵ����SLED
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void SLED_Stop(SLED_TypeDef * SLEDx)
{
	SLEDx->CR &= ~(1 << SLED_CR_EN_Pos);
}
