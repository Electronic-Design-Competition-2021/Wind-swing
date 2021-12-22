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

#ifndef __OSCILL_H__
#define __OSCILL_H__
#include "struct_typedef.h"

  
  

/*******************************************************************************
	mode0:��ֵ����Ӧģʽ����ģʽ��ʾ����������һҳ�����ݵ������Сֵȷ����ǰҳ�����ű�����y��ƫ����
	mode1:��ֵ�ֶ�ģʽ���û�����ȷ�����ű�����y��ƫ����
*******************************************************************************/
typedef struct
{
	uint16_t loca_x;	//�������x����
	uint16_t loca_y;	//�������y����
	uint16_t length;	//���ڳ���
	uint16_t width;		//���ڿ��
	uint16_t cursor_x;		//��ǰˢ��λ��x����
	uint16_t last_cursor_x;	//�ϴ�ˢ��λ��x����
	uint16_t cursor_y;		//��ǰˢ��λ��y����
	uint16_t last_cursor_y;	//�ϴ�ˢ��λ��y����
	int32_t y_max;		//yֵ��ʷ���ֵ
	int32_t y_min;		//yֵ��ʷ��Сֵ
	float y_ratio;		//y�����ű���
	int32_t y_max_last;		//��һҳyֵ��ʷ���ֵ
	int32_t y_min_last;		//��һҳyֵ��ʷ���ֵ
	uint8_t mode;			//ģʽ
	int32_t y_bias;		//y��ƫ��
}osc_window;

void osc_init(osc_window* win1,uint16_t loca_x_set,uint16_t loca_y_set,uint16_t length_set,uint16_t width_set,uint8_t mode);
void osc_refresh(osc_window* win1, uint32_t x_raw,int32_t y_raw);
void osc_y_ratio_bias_set(osc_window* win1,float ratio,int32_t bias);
void osc_mode_set(osc_window* win1,uint8_t mode);
#endif

