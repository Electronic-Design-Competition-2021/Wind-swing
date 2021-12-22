#include "bluetooth.h"
#include "string.h"
#include "stdio.h"
#include "fifo.h"
#include "bsp_usart.h"
#include "bsp_gpio.h"
#include "bsp_lcd_init.h"
#include "bsp_lcd.h"
#include "string.h"
#include "menu.h"
#include "appMenu.h"

uint8_t bt_tx_buf[BT_TX_LENGTH];
uint8_t btProcBuf[BT_RX_LENGTH];//接收数据长度
btDataType btData;

btDataType mode;//选择风力摆模式的结构体

//蓝牙上位机数据发送,两个32位整数
void bt_SendData(uint32_t data1 ,uint32_t data2)
{
	uint8_t i = 0;
	memset(bt_tx_buf,0,BT_TX_LENGTH);
	bt_tx_buf[0] = 0xA5;
	for( i = 0;i < 4; i ++ )
	{
		bt_tx_buf[i+1] = *((uint8_t *)(&data1) + i);
	}
	for( i = 0;i < 4; i ++ )
	{
		bt_tx_buf[i+5] = *((uint8_t *)(&data2) + i);
	}
	
	for(i = 1;i<BT_TX_LENGTH-2;i++)
	{
		bt_tx_buf[BT_TX_LENGTH-2] += bt_tx_buf[i];
	}
	
	bt_tx_buf[BT_TX_LENGTH-1] = 0x5A;
	usart1_SendData(bt_tx_buf,BT_TX_LENGTH);	//改成任意串口即可
}
void bt_DataUnPack(uint8_t *data)
{
	mode.usemode = data[0];
	User_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
}




void bt_DataSend(btDataType_tx * data)
{
	uint8_t tx_buf[BT_TX_LENGTH];
	tx_buf[0] = 0xA5;
	memcpy(tx_buf+1,&data->float1,4);	//float复制到数组中
	
//	memcpy(tx_buf+1,&data->float1,4);	//float复制到数组中
}
uint8_t buf[4];
uint8_t openmv[2];
void uartIRQ()
{
	uint8_t cnt = 0,dat = 0;
	if(cnt == 3 && dat == '\n')
	{
		openmv[0] = buf[0];
		openmv[1] = buf[1];
		cnt = 0;
		return;
	}
	buf[cnt++] = dat;
}
void bt_DataProcess(fifo_s_t * p_fifo)
{
	static uint8_t proc_step = 0;
	uint8_t head = 0,end = 0;
	uint8_t i = 0,sum = 0,adj_sum = 0;
	
	if(fifo_s_used(p_fifo))
	{
		if(proc_step == 0)	
		{
			//step1：定位帧头
			while(head != 0xA5)
			{
				if(fifo_s_gets(p_fifo,(char*)&head,1) == -1)	//查找帧头
					return;	//已全部取出,未发现帧头，直接返回
			}
			proc_step = 1;	//找到帧头，进行step2
		}
		if(proc_step == 1)
		{
			//step2：读出数据
			if(fifo_s_used(p_fifo) >= (BT_RX_LENGTH - 1))	//判断fifo中的数据长度
			{
				fifo_s_gets(p_fifo,(char*)&btProcBuf,BT_RX_LENGTH - 3);	//读出数据
				proc_step = 2;	//读出数据完成，进行step3
			}
			else
				return;	//数据还未接收完，退出
		}
		if(proc_step == 2)
		{
			//step3：计算校验和 & 判断帧尾
			if(fifo_s_used(p_fifo) >= 2)
			{
				fifo_s_gets(p_fifo,(char*)&adj_sum,1);	//读出校验和
				i = BT_RX_LENGTH - 3;	// 2021.7.7注释（更改）
				while(i -- )
					sum += btProcBuf[i];	//计算校验和
				fifo_s_gets(p_fifo,(char*)&end,1);	//读出帧尾
				if(sum == adj_sum && end == 0x5A)
					proc_step = 3;	//校验和&帧尾正确，进行step4
				else
				{
					proc_step = 0;	//错误，返回
					return;
				}
			}
			else
				return;
		}
		if(proc_step == 3)
		{
			proc_step = 0;	//处理完成，重新接收
			mode.dataFlag = 1;
			bt_DataUnPack(btProcBuf);//解析数据
		}
	}
}


