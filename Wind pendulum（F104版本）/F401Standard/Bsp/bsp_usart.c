#include "bsp_usart.h"
#include "fifo.h"
#include "usart.h"
#include <stdarg.h>


uint8_t usart1_Buf[2][U1_BUF_LENGHT];
uint8_t u1_Fifo_Buf[U1_FIFO_BUF_LENGTH];
fifo_s_t u1_Fifo;


void usart1_init(void)
{

	fifo_s_init(&u1_Fifo, u1_Fifo_Buf, U1_FIFO_BUF_LENGTH);	//��u1_Fifo_Buf��ʼ��Ϊfifo������
	
    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

	//DMA_LISR �Ĵ������ڹ��������� 0~3���� DMA_HISR ���ڹ��������� 4~7,TCIFΪ������ɱ�־
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_LISR_TCIF2);	//usart1 rx ΪDMA2 stream2

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(usart1_Buf[0]);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(usart1_Buf[1]);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, U1_BUF_LENGHT);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA����
    __HAL_DMA_ENABLE(&hdma_usart1_rx);


	/* ����DMA���� */
	
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);

}

void usart1_SendData(uint8_t *data, uint16_t len)
{
	if(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
		//�ж�DMA�Ƿ����ڴ���
		//�ȴ�DMA�������
		while(__HAL_DMA_GET_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7) == 0){};	//usart1 tx ΪDMA2 stream7
	}
	//disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}


/**
  * @brief          �������ݷ���
  * @note 			�ú���ʵ��DMA��ʽ��printf�����Ұ�������Э�齫���ݴ��������������λ��
  * @param[in]      ��printf���÷���ͬ
  * @retval         none
  */

uint8_t printf_buf[128];
void User_printf(const char *format,...)
{
	uint32_t length;
	va_list args;
	va_start(args, format);
	length = vsnprintf((char*)(printf_buf), sizeof(printf_buf), (char*)format, args);//�ڶ���������������ݳ���
	va_end(args);
	usart1_SendData(printf_buf,length);
}

//�����жϣ������ж� + DMA˫������
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)	//���������ж�
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);	//����жϱ�־λ

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)	//�����ڻ�����0
        {
			//ʧ��DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
			
			//��ȡ���յ������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = U1_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
			
			//�趨DMA�����ֽ�����512�ֽ�
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, U1_BUF_LENGHT);
			
			//���´ν������ݵ�buff��Ϊusart1_buf[1]
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
			
			//ʹ��DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
			
			//�����ݴ���fifo��
            fifo_s_puts(&u1_Fifo, (char*)usart1_Buf[0], this_time_rx_len);
        }
        else		//���������1������
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = U1_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, U1_BUF_LENGHT);
			//���´ν������ݵ�buff��Ϊusart6_buf[0]
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&u1_Fifo, (char*)usart1_Buf[1], this_time_rx_len);
        }
    }
}

//    if(huart1.Instance->SR & UART_FLAG_RXNE)	//���յ�����
//    {
////        __HAL_UART_CLEAR_PEFLAG(&huart4);		//�����־λ
//		data_temp = huart1.Instance->DR;	//�������ݣ�ͬʱ�����־λ
//		if(push_flag == 0)
//			bt_rx_buf[0][this_time_rx_len++] = data_temp;	//��������
//		else
//			push_flag = 0;	//������һ�ֽ�
//		if(this_time_rx_len == BT_FRAME_LEN)	//������յ��˶�������
//		{
//			this_time_rx_len = 0;
//			//����������������
//			if(bt_rx_buf[0][0] == 0XA5 && bt_rx_buf[0][BT_FRAME_LEN - 1] == 0X5A)
//			{
//				for(i = 1;i<BT_FRAME_LEN-2;i++)
//				{
//					frame_sum += bt_rx_buf[0][i];		//�ۼ�У���
//					*bt_frame_p++ = bt_rx_buf[0][i];	//�������ݣ�4��4�ֽ�int����
//				}
//				bt_frame_p = (uint8_t *)bt_frame;				//�������ݵ�ַ
//				if(frame_sum == bt_rx_buf[0][BT_FRAME_LEN - 2])	//�ж�У���
//				{
//					bt_rx_flag = 1;		//������ɱ�־λ��λ
//				}				
//			}
//			else
//			{
//				push_flag = 1;	//��һ�ֽڶ���
//			}
//		}
//    }




