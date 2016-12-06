#include "sys.h"
#include "gpio.h"
#include "delay.h"
#include "led.h"
#include "wdog.h"
#include "pit.h"
#include "dma.h"
#include "ftm.h"
#include "lptm.h"
#include "key.h"
#include "adc.h"
#include "tsi.h"
#include "spi.h"
#include "i2c.h"
#include "pdb.h"
#include "sd.h"
#include "flash.h"
#include "spilcd.h"
#include "stdio.h"
#include "nrf2401.h"
#include "uart.h"
#include "oled.h"
#include "data_sent.h"
#include "math.h"
#include "L3G4200D.h"
#include "menu.h"
#include "dmacnt.h"
#include "Mydefine.h"
#include "MyIIC.h"
#include "MMA8451.h"
#include "SDCar.h"
#include "nrf2401.h"


uint16_t Mode = 2;
int16_t SDData[2];
uint8_t Dummy = 0;           //DMA采集除bug
Data_Type DATA_ALL;
extern Data_Type *Sent_Data;
uint8_t Pixels[14000];
uint8_t timeFlag = 0;
uint16_t k = 5;
uint32_t time;
uint16_t whichSet = 0;
uint8_t buff[11];
uint8_t buffCnt = 0;
uint8_t receOverFlag = 0;
uint16_t acceX;
uint16_t acceY;
uint16_t acceZ;
uint16_t acceXLast;
uint16_t acceYLast;
uint16_t acceZLast;
uint16_t acceSum = 0;
uint16_t temper;
uint8_t countFlag;
uint8_t dateSend[20];
uint8_t hour;
uint8_t min;
uint8_t sec;
uint16_t maxDate = 0;
uint16_t minDate = 0;
uint16_t maxDateTemp = 0;
uint16_t minDateTemp = 65000;
uint16_t acceSumLast = 0;

uint8_t regetMinFlag = 0;
uint8_t becomeBigCnt = 0;
uint8_t regetMaxFlag = 1;;
uint8_t becomeSmallCnt = 0;
uint16_t averValue = 0;

uint16_t belowAverCnt = 0;
uint16_t aboveAverCnt = 0;

uint32_t step = 0;

/****************************************/
void InitPit()
{
	PIT_InitTypeDef PIT_InitStruct1;
	PIT_InitStruct1.PITx = PIT1;          
	PIT_InitStruct1.PIT_Interval = 10;   
	PIT_Init(&PIT_InitStruct1);
	PIT_ITConfig(PIT1,PIT_IT_TIF,ENABLE);
	NVIC_EnableIRQ(PIT1_IRQn);
}
/**************************************/
void writeSD(void)
{
	dateSend[0] = acceSum;
	dateSend[1] = acceSum >> 8;
	dateSend[2] = acceX;
	dateSend[3] = acceX >> 8;
	dateSend[4] = acceY;
	dateSend[5] = acceY >> 8;
	dateSend[6] = acceZ;
	dateSend[7] = acceZ >> 8;
	dateSend[8] = maxDate;
	dateSend[9] = maxDate >> 8;
	dateSend[10] = minDate;
	dateSend[11] = minDate >> 8;
	dateSend[12] = averValue;
	dateSend[13] = averValue >> 8;
	dateSend[14] = aboveAverCnt;
	dateSend[15] = aboveAverCnt >> 8;
	dateSend[16] = belowAverCnt;
	dateSend[17] = belowAverCnt >> 8;
	dateSend[18] = step;
	while(SD_WriteSingleBlock(time,dateSend) != ESDHC_OK);
}
/**************************************/
void readSD(void)
{
	SD_ReadSingleBlock(time,dateSend);
	acceSum = dateSend[1];
	acceSum <<= 8;
	acceSum |= (uint16_t)dateSend[0];
	
	acceX = dateSend[3];
	acceX <<= 8;
	acceX |= (uint16_t)dateSend[2];
	
	acceY = dateSend[5];
	acceY <<= 8;
	acceY |= (uint16_t)dateSend[4];
	
	acceZ = dateSend[7];
	acceZ <<= 8;
	acceZ |= (uint16_t)dateSend[6];
	
	maxDate = dateSend[9];
	maxDate <<= 8;
	maxDate |= (uint16_t)dateSend[8];
	
	minDate = dateSend[11];
	minDate <<= 8;
	minDate |= (uint16_t)dateSend[10];
	
	averValue = dateSend[13];
	averValue <<= 8;
	averValue |= (uint16_t)dateSend[12];
	
	aboveAverCnt = dateSend[15];
	aboveAverCnt <<= 8;
	aboveAverCnt |= (uint16_t)dateSend[14];
	
	belowAverCnt = dateSend[17];
	belowAverCnt <<= 8;
	belowAverCnt |= (uint16_t)dateSend[16];
	
	step = dateSend[18];
}
/****************************************/
float mySqrt(unsigned long x)
{
	float val = x;//最终
	float last;//保存上一个计算的值
	do
	{
		last = val;
		val =(val + x / val) / 2;
	}
	while(fabs(val-last) > 1);   //精度控制
	return val;
}
/******************************/
void Data_Calculate(void)
{
	if(buff[0]==0x55 && buff[1] == 0x51)
	{
			acceXLast = acceX;
			acceYLast = acceY;
			acceZLast = acceZ;
			acceX = buff[2];
			acceX = (acceX | (uint16_t)(buff[3]<<8) + 5000);
			acceY = buff[4];
			acceY = (acceY | (uint16_t)(buff[5]<<8) + 5000);
			acceZ = buff[6];
			acceZ = (acceZ | (uint16_t)(buff[7]<<8) + 5000);
			//acceY = ((buff[5]<<8| buff [4])+5000);
			//acceZ = ((buff[7]<<8| buff [6])+5000);
		
			temper = ((short)(buff[9] << 8| buff[8])) / 340.0 + 36.25;
			
			acceX = acceXLast * 0.2 + acceX * 0.8;
			acceY = acceYLast * 0.2 + acceY * 0.8;
			acceZ = acceZLast * 0.2 + acceZ * 0.8;
			acceSumLast = acceSum;
			acceSum = (acceX * acceX + acceY * acceY + acceZ * acceZ) / 10000;
	}
}


/**************************************/
void GetTime(void)
{
	uint8_t temp_i=0;
	uint8_t a[4]={0};
	OLED_Write_String(0,0,(uint8_t*)"getting date");
//	while(1)
//	if(UART_ReceiveData(UART3,&a[temp_i]))
//	{
//		OLED_Write_String(0,2,a);
//	}
	
	while(temp_i<=3)
	{
		while(UART_ReceiveData(UART3,&a[temp_i]) ==0);
		if(a[0]==128)
			temp_i++;
	}
	hour=a[1];
	min=a[2];
	sec=a[3];
	UART_SendData(UART3,140);
	OLED_Clear();
	OLED_Write_String(0,0,(uint8_t*)"Current time is:");

	OLED_Write_Num2(6,1,hour);
	OLED_Write_Char(8,1,':');
	OLED_Write_Num2(9,1,min);
	OLED_Write_Char(11,1,':');
	OLED_Write_Num2(12,1,sec);
}
/*******************************/
void judge()
{
//	if(time % 1000 == 0)
//	{
//		maxDate = 0;
//		minDate = 65000;
//	}
	if(regetMinFlag == 1)
	{
		if(acceSum < minDateTemp)//更新最小值
		{
			minDateTemp = acceSum;
		}
		
		if(acceSum > acceSumLast)//连续十场变大 则不记录最小值 
		{
			becomeBigCnt++;
		}
		else
		{
			becomeBigCnt = 0;
		}
		
		if(becomeBigCnt >= 10)
		{
			minDate = minDateTemp;//储存最小值
			regetMinFlag = 0;
			regetMaxFlag = 1;//开始记录最大值
			maxDateTemp = 0;//清零
			
			
		}
	}
	
	if(regetMaxFlag == 1)
	{
		if(acceSum > maxDateTemp)
			maxDateTemp = acceSum;
		
		if(acceSum < acceSumLast)
		{
			becomeSmallCnt++;
		}
		else
		{
			becomeSmallCnt = 0;
		}
		
		if(becomeSmallCnt >= 10)
		{
			maxDate = maxDateTemp;
			regetMinFlag = 1;//开始记录最小值
			regetMaxFlag = 0;
			minDateTemp = 65000;
		}
	}
	
	
	if(maxDate - minDate > 1400 && maxDate - minDate < 8500)
		averValue = (maxDate + minDate) >> 1;
	
	if(acceSum >= averValue)
	{
		if(belowAverCnt > 30)
		{
			if(aboveAverCnt < 0xffff)
				aboveAverCnt++;
			if(aboveAverCnt > 30)
			{
				step++;
				aboveAverCnt = 0;
				belowAverCnt = 0;
			}
		}
		else
		{
			belowAverCnt = 0;
			if(aboveAverCnt < 0xffff)
				aboveAverCnt++;
		}
			
	}
	else
	{
		if(aboveAverCnt > 30)
		{
			if(belowAverCnt < 0xffff)
				belowAverCnt++;
			if(belowAverCnt > 30)
			{
				step++;
				aboveAverCnt = 0;
				belowAverCnt = 0;
			}
		}
		else
		{
			aboveAverCnt = 0;
			if(belowAverCnt < 0xffff)
				belowAverCnt++;
		}
			
	}
	
}


/**********************************************/
int main(void)
{
	
	//初始化系统时钟 使用外部50M晶振 PLL倍频到100M
	SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
	OLED_Init();
	DelayInit();
	
	DataInit();
	Data_Uart_Init();
	
	InitPit();
	
	UART_DebugPortInit(UART5_RX_E8_TX_E9,640000);
	UART_DebugPortInit(UART0_RX_PA14_TX_PA15,115200);          //初始化上位机串口
	UART_DebugPortInit(UART3_RX_B10_TX_B11,9600);//用于蓝牙发送数据
	
	UART_ITConfig(UART0,UART_IT_RDRF,ENABLE);	//配置中断
	NVIC_EnableIRQ(UART0_RX_TX_IRQn);	//开启串口发送中断
	
	//GetTime();//蓝牙获得时间基准值
	
	Menu_Init();
	Display_All();

	if(Mode==0)
	{
		OLED_Clear();
		while(1)
		{
			if(receOverFlag)
			{
				receOverFlag = 0;
				time++;
				Data_Calculate();
				judge();
				OLED_Write_Num5(0,0,acceSum);
				OLED_Write_Num5(0,2,temper);
				OLED_Write_Num5(0,4,time);
				OLED_Write_Num5(10,0,step);
			}
		}
		
	}
	if(Mode == 1)
	{
		OLED_Write_String(0,0,(uint8_t*)"SD");
		MySDInit_Write();
		while(1)
		{
			if(receOverFlag)
			{
				receOverFlag = 0;
				time++;
				Data_Calculate();
				judge();
				OLED_Write_Num5(0,2,acceSum);
				OLED_Write_Num5(0,4,temper);
				OLED_Write_Num5(0,6,time);
				OLED_Write_Num5(10,0,step);
				writeSD();
			}
		}
	}
	if(Mode == 2)
	{
		NVIC_DisableIRQ(UART0_RX_TX_IRQn);
		MySDInit_Write();
		while(1)
		{
			if(timeFlag == 1)
			{
				timeFlag = 0;
				time++;
				OLED_Write_Num5(0,0,time);
				readSD();
				UART_Send_Con();
			}
		}
	}
	
	
	
	return 0;
}
void assert_failed(uint8_t* file, uint32_t line)
{
	//断言失败检测
	UART_printf("assert_failed:line:%d %s\r\n",line,file);
	while(1);
}

