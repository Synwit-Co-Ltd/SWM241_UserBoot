#ifndef __SWM241_SAFETY_H__
#define	__SWM241_SAFETY_H__

typedef struct {
	uint32_t bottom_addr;	//����ײ���ַ
	uint32_t top_addr;		//���򶥲���ַ��ע�⣺REGION_3�Ķ�����ַ�̶�Ϊ0xFFFFFFFF
	uint32_t action;		//������ַ�Ƿ�����ʱ�Ķ�������ȡֵ��SAFETY_ACTION_RESET��SAFETY_ACTION_INT
	uint32_t inten;			//��ַ�Ƿ������ж�ʹ��
} SAFETY_Region_InitStructure;


#define SAFETY_WP_IER		(1 << SAFETY_PERWP_IER_Pos)			//�ж�ʹ�ܼĴ���д����
#define SAFETY_WP_IOCFGR	(1 << SAFETY_PERWP_IOCFGR_Pos)		// IO ���üĴ���д����
#define SAFETY_WP_CLKCFGR	(1 << SAFETY_PERWP_CLKCFGR_Pos)		//ʱ�����üĴ���д����
#define SAFETY_WP_ANACFGR	(1 << SAFETY_PERWP_ANACFGR_Pos)		//ģ�����üĴ���д����

#define SAFETY_SIZE_256B	1
#define SAFETY_SIZE_512B	2
#define SAFETY_SIZE_1024B	3

#define SAFETY_REGION_0		0
#define SAFETY_REGION_1		1
#define SAFETY_REGION_2		2
#define SAFETY_REGION_3		3

#define SAFETY_ACTION_RESET	0
#define SAFETY_ACTION_INT	1


void SAFETY_PerWP_Open(uint32_t peripherals);
void SAFETY_PerWP_Close(uint32_t peripherals);
void SAFETY_RAMWP_Open(uint32_t addr, uint32_t size);
void SAFETY_RAMWP_Close(void);
void SAFETY_IAAD_Open(void);
void SAFETY_IAAD_Close(void);
void SAFETY_IAADRegion_Config(uint32_t region, SAFETY_Region_InitStructure * initStruct);

void SAFETY_IAAD_INTEn(uint32_t region);
void SAFETY_IAAD_INTDis(uint32_t region);
void SAFETY_IAAD_INTClr(uint32_t region);
uint32_t SAFETY_IAAD_INTStat(uint32_t region);


#endif // __SWM241_SAFETY_H__
