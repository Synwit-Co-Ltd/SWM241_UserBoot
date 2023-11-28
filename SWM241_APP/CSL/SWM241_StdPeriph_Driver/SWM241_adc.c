/****************************************************************************************************************************************** 
* �ļ�����:	SWM241_adc.c
* ����˵��:	SWM241��Ƭ����ADC��ģת��������������
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
#include "SWM241_adc.h"


static uint32_t ADC_K, ADC_Offset;


/****************************************************************************************************************************************** 
* ��������: ADC_Init()
* ����˵��:	ADCģ��ת������ʼ��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����Чֵ����ADC
*			ADC_InitStructure * initStruct		����ADC����ض�ֵ�Ľṹ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_Init(ADC_TypeDef * ADCx, ADC_InitStructure * initStruct)
{	
	uint8_t trig_src;
	
	switch((uint32_t)ADCx)
	{
	case ((uint32_t)ADC):
		SYS->CLKEN0 |= (0x01 << SYS_CLKEN0_ADC_Pos);
		break;
	}
	
	ADC_Close(ADCx);		//һЩ�ؼ��Ĵ���ֻ����ADC�ر�ʱ����
	
	ADC->CTRL1 &= ~(0xFF <<  8);
	ADC->CTRL1 |= (0x0F <<  8);
	ADC->CTRL1 &= ~(0x0F << 16);
	ADC->CTRL1 |= (0x0F << 16);
	
	ADC_Offset = (SYS->BACKUP[2] >>  0) & 0xFFFF;
	ADC_K = (SYS->BACKUP[2] >> 16) & 0xFFFF;
	ADC_K = ADC_K * 1.024;
	
	ADCx->CTRL1 &= ~ADC_CTRL1_CLKSRC_Msk;
	ADCx->CTRL1 |= (0 << ADC_CTRL1_CLKSRC_Pos);
	SYS->CLKSEL &= ~(SYS_CLKSEL_ADC_Msk | SYS_CLKSEL_ADCDIV_Msk);
	SYS->CLKSEL |= ((initStruct->clk_src & 0xF) << SYS_CLKSEL_ADC_Pos);
	
	ADCx->CTRL3 &= ~ADC_CTRL3_CLKDIV0_Msk;
	ADCx->CTRL3 |= (((initStruct->clk_src >> 4) & 3) << ADC_CTRL3_CLKDIV0_Pos);
	
	ADCx->CTRL2 &= ~(ADC_CTRL2_ADCEVCM_Msk | ADC_CTRL2_PGAEVCM_Msk | ADC_CTRL2_PGAGAIN_Msk | ADC_CTRL2_VCMSEL_Msk | ADC_CTRL2_CLKDIV1_Msk | ADC_CTRL2_CLKDIV2_Msk);
 	ADCx->CTRL2 |=  (PGA_VCM_EXTERNAL           << ADC_CTRL2_ADCEVCM_Pos) |
					(PGA_VCM_EXTERNAL           << ADC_CTRL2_PGAEVCM_Pos) |
					(7                          << ADC_CTRL2_PGAGAIN_Pos) |
					(7                          << ADC_CTRL2_VCMSEL_Pos)  |
					((initStruct->clk_src >> 6) << ADC_CTRL2_CLKDIV1_Pos) |
					(initStruct->clk_div        << ADC_CTRL2_CLKDIV2_Pos);
	
	ADCx->CTRL3 &= ~ADC_CTRL3_REFPSEL_Msk;
	ADCx->CTRL3 |= (initStruct->ref_src << ADC_CTRL3_REFPSEL_Pos);
	
	if(initStruct->trig_src & 0x1000)
	{
		trig_src = initStruct->trig_src >> 12;
		ADCx->TRGMSK = ~(initStruct->trig_src & 0xFFF);
		
		ADCx->CHSEL &= ~ADC_CHSEL_PWM_Msk;
		ADCx->CHSEL |= (initStruct->channels << ADC_CHSEL_PWM_Pos);
		
		ADCx->CHSEL &= ~ADC_CHSEL_SW_Msk;
		ADCx->CHSEL |= (initStruct->channels << ADC_CHSEL_SW_Pos);
	}
	else
	{
		trig_src = initStruct->trig_src & 0x0FFF;
		
		ADCx->CHSEL &= ~ADC_CHSEL_SW_Msk;
		ADCx->CHSEL |= (initStruct->channels << ADC_CHSEL_SW_Pos);
	}
	
	ADCx->CTRL &= ~(ADC_CTRL_TRIG_Msk | ADC_CTRL_CONT_Msk | ADC_CTRL_AVG_Msk);
	ADCx->CTRL |= (trig_src             << ADC_CTRL_TRIG_Pos) |
				  (initStruct->Continue << ADC_CTRL_CONT_Pos) |
				  (initStruct->samplAvg << ADC_CTRL_AVG_Pos);
	
	ADCx->IF = 0xFFFFFF;	//����жϱ�־
	
	ADCx->IE &= ~(ADC_IE_CH0EOC_Msk | ADC_IE_CH1EOC_Msk | ADC_IE_CH2EOC_Msk | ADC_IE_CH3EOC_Msk |
				  ADC_IE_CH4EOC_Msk | ADC_IE_CH5EOC_Msk | ADC_IE_CH6EOC_Msk | ADC_IE_CH7EOC_Msk |
				  ADC_IE_CH8EOC_Msk | ADC_IE_CH9EOC_Msk | ADC_IE_CH10EOC_Msk| ADC_IE_CH11EOC_Msk);
	ADCx->IE |= (((initStruct->EOC_IEn & ADC_CH0) ? 1 : 0) << ADC_IE_CH0EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH1) ? 1 : 0) << ADC_IE_CH1EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH2) ? 1 : 0) << ADC_IE_CH2EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH3) ? 1 : 0) << ADC_IE_CH3EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH4) ? 1 : 0) << ADC_IE_CH4EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH5) ? 1 : 0) << ADC_IE_CH5EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH6) ? 1 : 0) << ADC_IE_CH6EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH7) ? 1 : 0) << ADC_IE_CH7EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH8) ? 1 : 0) << ADC_IE_CH8EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH9) ? 1 : 0) << ADC_IE_CH9EOC_Pos) |
				(((initStruct->EOC_IEn & ADC_CH10)? 1 : 0) << ADC_IE_CH10EOC_Pos)|
				(((initStruct->EOC_IEn & ADC_CH11)? 1 : 0) << ADC_IE_CH11EOC_Pos);
				
	ADCx->IE &= ~(ADC_IE_CH0OVF_Msk | ADC_IE_CH1OVF_Msk | ADC_IE_CH2OVF_Msk | ADC_IE_CH3OVF_Msk |
				  ADC_IE_CH4OVF_Msk | ADC_IE_CH5OVF_Msk | ADC_IE_CH6OVF_Msk | ADC_IE_CH7OVF_Msk |
				  ADC_IE_CH8OVF_Msk | ADC_IE_CH9OVF_Msk | ADC_IE_CH10OVF_Msk| ADC_IE_CH11OVF_Msk);
	ADCx->IE |= (((initStruct->OVF_IEn & ADC_CH0) ? 1 : 0) << ADC_IE_CH0OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH1) ? 1 : 0) << ADC_IE_CH1OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH2) ? 1 : 0) << ADC_IE_CH2OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH3) ? 1 : 0) << ADC_IE_CH3OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH4) ? 1 : 0) << ADC_IE_CH4OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH5) ? 1 : 0) << ADC_IE_CH5OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH6) ? 1 : 0) << ADC_IE_CH6OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH7) ? 1 : 0) << ADC_IE_CH7OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH8) ? 1 : 0) << ADC_IE_CH8OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH9) ? 1 : 0) << ADC_IE_CH9OVF_Pos) |
				(((initStruct->OVF_IEn & ADC_CH10)? 1 : 0) << ADC_IE_CH10OVF_Pos)|
				(((initStruct->OVF_IEn & ADC_CH11)? 1 : 0) << ADC_IE_CH11OVF_Pos);
	
	switch((uint32_t)ADCx)
	{
	case ((uint32_t)ADC):
		if(initStruct->EOC_IEn | initStruct->OVF_IEn)
		{
			NVIC_EnableIRQ(ADC_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(ADC_IRQn);
		}
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	ADC_Open()
* ����˵��:	ADC���������������������Ӳ������ADCת��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_Open(ADC_TypeDef * ADCx)
{
	ADCx->CTRL |= (0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* ��������:	ADC_Close()
* ����˵��:	ADC�رգ��޷������������Ӳ������ADCת��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_Close(ADC_TypeDef * ADCx)
{
	ADCx->CTRL &= ~(0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* ��������:	ADC_Start()
* ����˵��:	�������ģʽ������ADCת��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_Start(ADC_TypeDef * ADCx)
{
	ADCx->START = (0x01 << ADC_START_GO_Pos);
}

/****************************************************************************************************************************************** 
* ��������:	ADC_Stop()
* ����˵��:	�������ģʽ��ֹͣADCת��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_Stop(ADC_TypeDef * ADCx)
{									 
	ADCx->START &= ~(0x01 << ADC_START_GO_Pos);
}

static uint32_t chn2idx(uint32_t chn)
{
	uint32_t idx = 0;
	
	switch(chn)
	{
		case 0x01:  idx = 0;  break;
		case 0x02:  idx = 1;  break;
		case 0x04:  idx = 2;  break;
		case 0x08:  idx = 3;  break;
		case 0x10:  idx = 4;  break;
		case 0x20:  idx = 5;  break;
		case 0x40:  idx = 6;  break;
		case 0x80:  idx = 7;  break;
		case 0x100: idx = 8;  break;
		case 0x200: idx = 9;  break;
		case 0x400: idx = 10; break;
		case 0x800: idx = 11; break;
	}
	
	return idx;
}

/****************************************************************************************************************************************** 
* ��������:	ADC_Read()
* ����˵��:	��ָ��ͨ����ȡת�����
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ��ȡת�������ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: uint32_t				��ȡ����ת�����
* ע������: ��
******************************************************************************************************************************************/
uint32_t ADC_Read(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t val;
	uint32_t idx = chn2idx(chn);
	
	val = (ADCx->CH[idx].DATA & ADC_DATA_VALUE_Msk);
	
	ADCx->CH[idx].STAT = ADC_STAT_EOC_Msk;	//���EOC��־
	
	if(val < ADC_Offset)
	{
		val = 0;
	}
	else
	{
		val = ((val - ADC_Offset) * ADC_K) >> 10;
		if(val > 4095)
			val = 4095;
	}
	
	return val;
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IsEOC()
* ����˵��:	ָ��ͨ���Ƿ�End Of Conversion
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ��ѯ״̬��ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: uint32_t				1 ��ͨ�������ת��    0 ��ͨ��δ���ת��
* ע������: ��
******************************************************************************************************************************************/
uint32_t ADC_IsEOC(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->CH[idx].STAT & ADC_STAT_EOC_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* ��������:	ADC_ChnSelect()
* ����˵��:	ADCͨ��ѡͨ��ģ��ת������ѡͨ��ͨ�������β���ת��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chns			Ҫѡͨ��ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7������ϣ�������λ�����㣩
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_ChnSelect(ADC_TypeDef * ADCx, uint32_t chns)
{
	ADCx->CHSEL &= ~ADC_CHSEL_SW_Msk;
	ADCx->CHSEL |=  (chns  << ADC_CHSEL_SW_Pos);
	
	ADCx->CTRL = ADCx->CTRL;
}


/****************************************************************************************************************************************** 
* ��������:	ADC_IntEOCEn()
* ����˵��:	ת������ж�ʹ��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntEOCEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntEOCDis()
* ����˵��:	ת������жϽ�ֹ
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntEOCDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntEOCClr()
* ����˵��:	ת������жϱ�־���
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntEOCClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntEOCStat()
* ����˵��:	ת������ж�״̬
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ��ѯ��ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: uint32_t				1 ��ͨ�������ת��    0 ��ͨ��δ���ת��
* ע������: ��
******************************************************************************************************************************************/
uint32_t ADC_IntEOCStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2))) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntOVFEn()
* ����˵��:	��������ж�ʹ��
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntOVFEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntOVFDis()
* ����˵��:	��������жϽ�ֹ
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntOVFDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntOVFClr()
* ����˵��:	��������жϱ�־���
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ���õ�ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void ADC_IntOVFClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* ��������:	ADC_IntOVFStat()
* ����˵��:	��������ж�״̬
* ��    ��: ADC_TypeDef * ADCx		ָ��Ҫ�����õ�ADC����ȡֵ����ADC
*			uint32_t chn			Ҫ��ѯ��ͨ������ЧֵADC_CH0��ADC_CH1��... ... ��ADC_CH7		
* ��    ��: uint32_t				1 ��ͨ�����    0 û��ͨ�����
* ע������: ��
******************************************************************************************************************************************/
uint32_t ADC_IntOVFStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2+1))) ? 1 : 0;
}
