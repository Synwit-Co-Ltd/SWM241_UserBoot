#include "SWM241.h"


/* 注意：remap.c中的函数必须在RAM中执行，Keil下实现方法有：
   方法一、Scatter file
   方法二、remap.c上右键 =》Options for File "remap.c" =》Properties =》Memory Assignment =》Code/Conts 选择 IRAM1
*/

void Flash_remap(uint32_t addr)
{
	/* 只能在APP中REMAP，在UserBoot中REMAP可能会导致访问UserBoot的代码被重定向到APP的代码 */
	FMC->REMAP = (1 << FMC_REMAP_ON_Pos) | ((addr / 2048) << FMC_REMAP_OFFSET_Pos);
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	
	__enable_irq();
}
