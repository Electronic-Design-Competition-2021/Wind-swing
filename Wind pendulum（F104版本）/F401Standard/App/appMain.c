#include "appMain.h"
#include "pid.h"
#include "fifo.h"
#include "usart.h"
#include "bluetooth.h"
#include "bsp_gpio.h"
#include "bsp_delay.h"
#include "bsp_usart.h"
#include "bsp_lcd_init.h"
#include "bsp_lcd.h"
#include "pic.h"
#include "spi.h"
#include "bsp_24l01.h"
#include "menu.h"
#include "appMenu.h"
#include "Oscilloscope.h"
#include "motorcol.h"

void appInit(void)
{
	usart1_init();	//��ʼ������1
	Windpendulum_Init();
	NRF24L01_Init();			//��ʼ��NRF24L01
	__HAL_SPI_ENABLE(&hspi1);	//ʹ��SPI
	
	
	
	SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);	//����spi1��DMA

	//LCD_Init();			//LCD��ʼ��
	User_Delay(100);
	//LCD_ShowString(COLUMN0,LINE0,"GUI DEMO - DOEE",GREEN,BLACK,16,0);	//��ʾ���յ�������
	//LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	//��ʼ��ɫ

	SPI1_NRF24_ReInit(SPI_BAUDRATEPRESCALER_16); //SPI1���³�ʼ��
//	while(NRF24L01_Check() == 1)	//���NRF24L01
//	{
//		HAL_Delay(100);
//		User_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);	//���δ��⵽��led����
//	}

	//menuInit(&GUI,menu1);	//��ʼ���˵�
}


uint32_t ledTick;
uint32_t nrfTick = 0;

uint8_t nrfBuf[32];

void appMain(void)
{
	//menuRefresh(&GUI);			//ˢ��GUI:�˵���appMenu.c
	bt_DataProcess(&u1_Fifo);	//�������ݴ���
	 Windpendulum_Run();
	if(User_GetTick() - ledTick >= 500)
	{
		ledTick = User_GetTick();
		//HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);	//LED��˸
	}
	
	//SPI1_NRF24_ReInit(SPI_BAUDRATEPRESCALER_16); //SPI1���³�ʼ��
//	if(User_GetTick() - nrfTick >= 10)
//	{
//		nrfTick = User_GetTick();
//		if(NRF24L01_RxPacket(nrfBuf)==0)//NRF���յ���Ϣ
//		{
//			SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_2);	//���³�ʼ��SPI1
//			//LCD_ShowString(10,20,nrfBuf,RED,WHITE,16,0);	//��ʾ���յ�������
//		}
//	}

}
