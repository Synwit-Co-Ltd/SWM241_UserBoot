#include "SWM241.h"


/* ע�⣺remap.c�еĺ���������RAM��ִ�У�Keil��ʵ�ַ����У�
   ����һ��Scatter file
   ��������remap.c���Ҽ� =��Options for File "remap.c" =��Properties =��Memory Assignment =��Code/Conts ѡ�� IRAM1
*/

void Flash_remap(uint32_t addr)
{
	/* ֻ����APP��REMAP����UserBoot��REMAP���ܻᵼ�·���UserBoot�Ĵ��뱻�ض���APP�Ĵ��� */
	FMC->REMAP = (1 << FMC_REMAP_ON_Pos) | ((addr / 2048) << FMC_REMAP_OFFSET_Pos);
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	
	__enable_irq();
}
