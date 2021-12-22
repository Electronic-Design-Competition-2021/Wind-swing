/**
  ****************************(C) COPYRIGHT 2021 DOEE****************************
  * @file      	menu.c/h
  * @brief     	�༶�˵����ײ㣩
  * @note      	ÿ���˵�������һ��������������ʽ�����ã��½��˵������½�һ��������
				�˵����л�ͨ���л�����ָ��ʵ�֣��˵�������ʼ����
				��Ҫ���ϵ���menuRefreshˢ��
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

#ifndef __MENU_H__
#define __MENU_H__
#include "struct_typedef.h"

#define MAX_DEGREE 20   //���˵����

typedef struct
{
    void (*menu_func_p)(void);          //�˵���ʾ����ָ��
    uint8_t refresh_flag;               //ˢ�±�־λ
    void (*menu_func_return_list[MAX_DEGREE])(void);    //���ز˵��б�
    uint8_t list_head;      //�б�ͷ��
    uint8_t cursor;         //���λ��
	uint8_t init_flag;		//��ʼ����־:�����˵��л�ʱ����λ�����ֶ����㣩
    //uint8_t list_free_num;  //�б�ʣ��ռ�

} menu_s_t;

void menuInit(menu_s_t* menu_s,void (*init_func)(void));// GUI��ʼ��
void menuRefresh(menu_s_t* menu_s);	// GUIˢ�º���
void menuRefreshFlagSet(menu_s_t* menu_s);// GUIˢ�±�־λ��λ
void menuSkip(menu_s_t* menu_s,void (*new_func)(void));	//������ת
void menuReturn(menu_s_t* menu_s);			//���淵��
#endif


