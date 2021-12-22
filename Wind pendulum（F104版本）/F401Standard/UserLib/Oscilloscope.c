/**
  ****************************(C) COPYRIGHT 2021 DOEE****************************
  * @file       Oscilloscope.c/h
  * @brief      ʾ����ʵ�֣�����������ʾ��
  * @note       
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
#include "Oscilloscope.h"
#include "bsp_lcd.h"


/**
  * @brief          ʾ�������ڳ�ʼ��
  * @param[in]      osc1: ʾ��������ָ��
  * @param[in]      loca_x_set: ����x����
  * @param[in]      loca_y_set: ����y����
  * @param[in]      length_set: ���ڳ���
  * @param[in]      width_set:  ���ڿ�� 
  * @param[in]      mode��		ģʽ
  *					@arg 0: ����Ӧ��ֵģʽ
  *					@arg 1: �ֶ���ֵģʽ
  */
void osc_init(osc_window* osc1,uint16_t loca_x_set,uint16_t loca_y_set,uint16_t length_set,uint16_t width_set,uint8_t mode)
{
	//����ʾ�������ڵĻ�������
	length_set -= 1;
	width_set -= 1;
	osc1->length = length_set - 1;
	osc1->width =  width_set - 1;
	osc1->loca_x = loca_x_set + 1;
	osc1->loca_y = loca_y_set + 1;
	osc1->y_min = 0;
	osc1->y_max = 0;
	osc1->y_min_last = 0;
	osc1->y_max_last = 0;
	osc1->y_ratio = 0;
	osc1->mode = mode;
	//����ʾ�����߿�
	LCD_DrawLine(loca_x_set,loca_y_set,loca_x_set + length_set,loca_y_set,WHITE);
	LCD_DrawLine(loca_x_set,loca_y_set + width_set,loca_x_set + length_set,loca_y_set + width_set,WHITE);
	LCD_DrawLine(loca_x_set,loca_y_set,loca_x_set,loca_y_set + width_set,WHITE);
	LCD_DrawLine(loca_x_set + length_set,loca_y_set,loca_x_set + length_set,loca_y_set + width_set,WHITE);	
}
/**
  * @brief          ʾ����ģʽ����
  * @param[in]      mode��		ģʽ
  *					@arg 0: ����Ӧ��ֵģʽ
  *					@arg 1: �ֶ���ֵģʽ
  */
void osc_mode_set(osc_window* osc1,uint8_t mode)
{
	osc1->mode = mode;
}
/**
  * @brief          ʾ��������y�����ź�ƫ��
  * @param[in]      osc1: ʾ��������ָ��
  * @param[in]      ratio: ���ű���
  * @param[in]      bias: ƫ����
  */
void osc_y_ratio_bias_set(osc_window* osc1,float ratio,int32_t bias)
{
	osc1->y_ratio = ratio;
	osc1->y_ratio -= osc1->y_ratio*0.2f;	//����������20%�Ŀհ�	
	osc1->y_bias = bias;
}

//ʾ����ˢ����ʾ�������ĵ粨��ˢ���ص��Լ�д�ģ������ο�����һ����������ʵ��ʾ������ʾ
//�ص㣺��ֵ����Ӧ����Բ��ν��з�ֵ���ţ�ʼ���ò��α����ڴ����С�
/**
  * @brief          ʾ����ˢ����ʾ
  * @param[in]      osc1: ʾ��������ָ��
  * @param[in]      x_raw: x������
  * @param[in]      y_raw: y������
  */
void osc_refresh(osc_window* osc1, uint32_t x_raw,int32_t y_raw)
{
	osc1->cursor_x = x_raw % osc1->length + osc1->loca_x;	//���㵱ǰˢ�µ�ĺ����꣺����ֵ%�ܳ��� + �����������
	
	if(osc1->cursor_x < osc1->last_cursor_x)	//�����ʼ�µ�һҳ
	{
		osc1->y_max_last = osc1->y_max;
		osc1->y_min_last = osc1->y_min;
		osc1->y_min = y_raw;	//����y����Сֵ
		osc1->y_max = y_raw;	//����y�����ֵ
		if(osc1->mode == 0)		//����Ӧ��ֵģʽ
		{
			osc1->y_ratio = (float)osc1->width/(float)(osc1->y_max_last-osc1->y_min_last);	//����y�����ţ�����һҳ�����ֵ����Сֵ���
			osc1->y_ratio -= osc1->y_ratio*0.2f;	//����������20%�Ŀհ�
		}
	}
	
	if(y_raw < osc1->y_min)
	{
		osc1->y_min = y_raw;	//����y����ʷ��Сֵ
	}
	if(y_raw > osc1->y_max)
	{
		osc1->y_max = y_raw;	//����y����ʷ���ֵ
	}
	if(osc1->last_cursor_x != 0)//��һ����ˢ��ʱosc1->last_cursor_x = 0�����±߿򱻸��ǣ�����������һ����
	{
		if(osc1->y_ratio > 0)	//ratioֵ��Ч�Ž���ˢ�£���ֵ����Ӧģʽ�£���һҳ��ˢ�»ᱻ���������Լ�����ʵ����ű�����
		{
			if(osc1->mode == 0)		//��ֵ����Ӧģʽ
				osc1->y_bias = osc1->y_min_last;	//y��ƫ���� = ��һҳ��Сֵ
			
			// ��ʽ��һ��±߽���10%�հף��ڶ��(y-y_bias)*y_ratio
			osc1->cursor_y = (osc1->loca_y + (float)osc1->width*0.9f) - (float)(y_raw-osc1->y_bias)*osc1->y_ratio;
			if(osc1->cursor_y < osc1->loca_y)
			{
				osc1->cursor_y = osc1->loca_y;
			}
			if(osc1->cursor_y > osc1->loca_y+osc1->width)
			{
				osc1->cursor_y = osc1->loca_y+osc1->width;
			}
	//		LCD_DrawLine(osc1->cursor_x,osc1->loca_y,osc1->cursor_x,osc1->loca_y+osc1->width,BLACK);//����ϴβ���
			LCD_Fill(osc1->cursor_x,osc1->loca_y,osc1->cursor_x+1,osc1->loca_y+osc1->width,BLACK);	//����ϴβ���(Ч�ʱ�����ĸ�)

			if(osc1->cursor_x > osc1->last_cursor_x)
			{
				if(osc1->last_cursor_y <= osc1->cursor_y)	//���ߺ�����ȱ�ݣ���Ҫ������յ�>=������ʽ
				{
					LCD_DrawLine(osc1->last_cursor_x,osc1->last_cursor_y,osc1->cursor_x,osc1->cursor_y,GREEN);				
				}
				else
				{
					LCD_DrawLine(osc1->last_cursor_x,osc1->cursor_y,osc1->cursor_x,osc1->last_cursor_y,GREEN);				
				}
			}
		}
	}
	osc1->last_cursor_x = osc1->cursor_x;	//�����ϴε�ˢ��x����
	osc1->last_cursor_y = osc1->cursor_y;	//�����ϴε�ˢ��y����
}
