/****************************************************************************************************************************************** 
* 文件名称:	SWM241_adc.c
* 功能说明:	SWM241单片机的ADC数模转换器功能驱动库
* 技术支持:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* 注意事项:
* 版本日期: V1.0.0		2016年1月30日
* 升级记录: 
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
* 函数名称: ADC_Init()
* 功能说明:	ADC模数转换器初始化
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，有效值包括ADC
*			ADC_InitStructure * initStruct		包含ADC各相关定值的结构体
* 输    出: 无
* 注意事项: 无
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
	
	ADC_Close(ADCx);		//一些关键寄存器只能在ADC关闭时设置
	
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
	
	ADCx->IF = 0xFFFFFF;	//清除中断标志
	
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
* 函数名称:	ADC_Open()
* 功能说明:	ADC开启，可以软件启动、或硬件触发ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Open(ADC_TypeDef * ADCx)
{
	ADCx->CTRL |= (0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Close()
* 功能说明:	ADC关闭，无法软件启动、或硬件触发ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Close(ADC_TypeDef * ADCx)
{
	ADCx->CTRL &= ~(0x01 << ADC_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Start()
* 功能说明:	软件触发模式下启动ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_Start(ADC_TypeDef * ADCx)
{
	ADCx->START = (0x01 << ADC_START_GO_Pos);
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_Stop()
* 功能说明:	软件触发模式下停止ADC转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
* 输    出: 无
* 注意事项: 无
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
* 函数名称:	ADC_Read()
* 功能说明:	从指定通道读取转换结果
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要读取转换结果的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				读取到的转换结果
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_Read(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t val;
	uint32_t idx = chn2idx(chn);
	
	val = (ADCx->CH[idx].DATA & ADC_DATA_VALUE_Msk);
	
	ADCx->CH[idx].STAT = ADC_STAT_EOC_Msk;	//清除EOC标志
	
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
* 函数名称:	ADC_IsEOC()
* 功能说明:	指定通道是否End Of Conversion
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询状态的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 该通道完成了转换    0 该通道未完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IsEOC(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->CH[idx].STAT & ADC_STAT_EOC_Msk) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_ChnSelect()
* 功能说明:	ADC通道选通，模数转换会在选通的通道上依次采样转换
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chns			要选通的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7及其组合（即“按位或”运算）
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_ChnSelect(ADC_TypeDef * ADCx, uint32_t chns)
{
	ADCx->CHSEL &= ~ADC_CHSEL_SW_Msk;
	ADCx->CHSEL |=  (chns  << ADC_CHSEL_SW_Pos);
	
	ADCx->CTRL = ADCx->CTRL;
}


/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCEn()
* 功能说明:	转换完成中断使能
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCDis()
* 功能说明:	转换完成中断禁止
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCClr()
* 功能说明:	转换完成中断标志清除
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntEOCClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntEOCStat()
* 功能说明:	转换完成中断状态
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 该通道完成了转换    0 该通道未完成转换
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IntEOCStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2))) ? 1 : 0;
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFEn()
* 功能说明:	数据溢出中断使能
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFEn(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE |= (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFDis()
* 功能说明:	数据溢出中断禁止
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFDis(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IE &= ~(0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFClr()
* 功能说明:	数据溢出中断标志清除
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要设置的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
void ADC_IntOVFClr(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	ADCx->IF = (0x01 << (idx*2+1));
}

/****************************************************************************************************************************************** 
* 函数名称:	ADC_IntOVFStat()
* 功能说明:	数据溢出中断状态
* 输    入: ADC_TypeDef * ADCx		指定要被设置的ADC，可取值包括ADC
*			uint32_t chn			要查询的通道，有效值ADC_CH0、ADC_CH1、... ... 、ADC_CH7		
* 输    出: uint32_t				1 有通道溢出    0 没有通道溢出
* 注意事项: 无
******************************************************************************************************************************************/
uint32_t ADC_IntOVFStat(ADC_TypeDef * ADCx, uint32_t chn)
{
	uint32_t idx = chn2idx(chn);
	
	return (ADCx->IF & (0x01 << (idx*2+1))) ? 1 : 0;
}
