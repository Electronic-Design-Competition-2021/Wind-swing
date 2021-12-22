/**
  ****************************(C) COPYRIGHT 2021 DOEE****************************
  * @file      	appMenu.c/h
  * @brief     	�༶�˵����û��㣩
  * @note      	ÿ���˵�������һ��������������ʽ�����ã��½��˵������½�һ��������
				ͨ������menuSkipʵ�ֲ˵���ת����غ�����menu.c/h
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     July-1-2021     doee            1. ���
  * @blog		https://blog.csdn.net/weixin_44578655?spm=1001.2101.3001.5343
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 DOEE****************************
  */
  
#include "appMenu.h"
#include "main.h"
#include "stdio.h"
#include "bsp_lcd.h"
#include "bsp_lcd_init.h"
#include "bluetooth.h"
#include "menu.h"
#include "Oscilloscope.h"

menu_s_t GUI;	//����༶�˵�

void menu1(void)
{
	SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_2);	//���³�ʼ��SPI1
	if(GUI.init_flag)
	{
		//���������л�
		GUI.init_flag = 0;
		LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	//��ʼ��ɫ
	}
	LCD_ShowString(COLUMN0,LINE0,"GUI DEMO - DOEE",GREEN,BLACK,16,0);	//��ʾ���յ�������
	LCD_ShowString(COLUMN0,LINE1,"FLOAT:",GREEN,BLACK,16,0);	//��ʾ���յ�������
	LCD_ShowFloatNum1(COLUMN1,LINE1,btData.float1,5,GREEN,BLACK,16);	//��ʾС��

	if(btData.buttonRes == 2)
	{
		menuSkip(&GUI,menu2);	//��ת��menu2
	}
	else if(btData.buttonRes == 3)
	{
		menuSkip(&GUI,menu3);	//��ת��menu3
	}
	else if(btData.buttonRes == 4)
	{
		menuReturn(&GUI);		//���ؽ���
	}
	btData.buttonRes = 0;
}

//����ʾ��������
osc_window osc_win;
uint32_t osc_x = 0;
void menu2(void)
{
	SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_2);	//���³�ʼ��SPI1
	
	if(GUI.init_flag)
	{
		//���������л�
		GUI.init_flag = 0;
		LCD_Fill(0,0,LCD_W,LCD_H,BLACK);	//��ʼ��ɫ
		osc_init(&osc_win,0,0,LCD_W,LCD_H,1);		//ʾ������ʼ��,ģʽ0
		osc_y_ratio_bias_set(&osc_win,(osc_win.width/255.0f),-128);
//		osc_init(&osc_win,uint16_t loca_x_set,uint16_t loca_y_set,uint16_t length_set,uint16_t width_set);
	}

	//LCD_ShowString(10,20,"menu2",RED,WHITE,16,0);	//��ʾ���յ�������
	osc_refresh(&osc_win,osc_x++,btData.float1);	//ˢ��ʾ�������ڣ�����Ļ���ص�Ϊ��λ��
	if(btData.buttonRes == 1)
	{
		menuSkip(&GUI,menu1);	//��ת��menu1
	}
	else if(btData.buttonRes == 3)
	{
		menuSkip(&GUI,menu3);	//��ת��menu3		
	}
	else if(btData.buttonRes == 4)
	{
		menuReturn(&GUI);		//���ؽ���
	}
	btData.buttonRes = 0;
}
void menu3(void)
{
	SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_2);	//���³�ʼ��SPI1
	if(GUI.init_flag)
	{
		//���������л�
		GUI.init_flag = 0;
		LCD_Fill(0,0,LCD_W,LCD_H,GREEN);	//��ʼ��ɫ
	}
	LCD_ShowString(10,20,"menu3",RED,WHITE,16,0);	//��ʾ���յ�������
	if(btData.buttonRes == 1)
	{
		menuSkip(&GUI,menu1);	//��ת��menu1
	}
	else if(btData.buttonRes == 2)
	{
		menuSkip(&GUI,menu2);	//��ת��menu2	
	}
	else if(btData.buttonRes == 4)
	{
		menuReturn(&GUI);		//���ؽ���
	}
	btData.buttonRes = 0;
}
void menu4(void)
{
	SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_2);	//���³�ʼ��SPI1
	if(GUI.init_flag)
	{
		//���������л�
		GUI.init_flag = 0;
		LCD_Fill(0,0,LCD_W,LCD_H,YELLOW);	//��ʼ��ɫ
	}
	LCD_ShowString(10,20,"menu4",RED,WHITE,16,0);	//��ʾ���յ�������
}


