#include "isr.h"
#include "Mydefine.h"
#include "oled.h"



void PIT1_IRQHandler(void)
{
	timeFlag = 1;
	PIT_ClearITPendingBit(PIT1,PIT_IT_TIF);
}

void UART0_RX_TX_IRQHandler(void)
{
	if(UART_ReceiveData(UART0,&buff[buffCnt]))
	{
		if(buffCnt == 0 && buff[0] != 0x55)
		{
			return;
		}
		buffCnt++;
		
		if(buffCnt == 11)
		{
			buffCnt = 0;
			receOverFlag = 1;
		}
	}
	
}

/**************************************芯片发生错误*******************************************/
#define  CHIP_DEBUG         ON		//提剐酒⑸阑挪槲侍夥椒ǎ梢栽诘ゲ降魇灾惺褂?
#if(CHIP_DEBUG==ON)
__asm void wait()
{
  BX lr	//返回发生错误的位置
}
#endif

void HardFault_Handler(void)//内存或堆栈溢出错误
{
	OLED_Clear();
	OLED_Write_String(0,0,(uint8_t *)"The memory");
	
	OLED_Write_String(0,2,(uint8_t *)"or Stacks");	
	
	OLED_Write_String(0,4,(uint8_t *)"overflows");
	
	#if(CHIP_DEBUG==ON)
	wait();
	#endif
	while(1);
}

void NMI_Handler(void)//不可屏蔽中断
{
	OLED_Clear();

	OLED_Write_String(0,0,(uint8_t *)"The Chip");
	
	OLED_Write_String(0,2,(uint8_t *)"have big");	
	
	OLED_Write_String(0,4,(uint8_t *)"error");
	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
	while(1);
}

void MemManage_Handler(void)//存储发生错误
{
	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}

void BusFault_Handler(void)//总线发生错误
{

	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}

void UsageFault_Handler(void)//用法发生错误
{

	#if(CHIP_DEBUG==ON)	
	wait();
	#endif
}
