//#include "bsp_lcd_init.h"
//#include "spi.h"




//extern DMA_HandleTypeDef hdma_spi1_tx;
//extern __IO uint8_t DMAsendingFlag;

////SPI1�������ã���ͬ�豸����SPI1ʱ��SPI���ò�����ͬ��ÿ���豸��Ҫ�е����ĳ�ʼ��������
//void SPI1_LCD_ReInit(uint8_t SPI_BaudRatePrescaler)
//{
//	__HAL_SPI_DISABLE(&hspi1);	//ʧ��SPI
//	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
//	
//	SPI1->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
//	SPI1->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
//	
//	SPI1->CR1 &= ~SPI_CR1_CPOL;	//��ռĴ���CPOLλ
//	SPI1->CR1 |= (SPI_POLARITY_HIGH & SPI_CR1_CPOL); // ����CPOL = HIGH
//	SPI1->CR1 &= ~SPI_CR1_CPHA;	//��ռĴ���CPHAλ
//	SPI1->CR1 |= (SPI_PHASE_2EDGE & SPI_CR1_CPHA);// ����CPHA = 2EDAGE
//	
//	__HAL_SPI_ENABLE(&hspi1);	//ʹ��SPI
//}


//void delay(int t)
//{
//	while(t--);
//}
///******************************************************************************
//      ����˵����LCD��������д�뺯��
//      ������ݣ�dat  Ҫд��Ĵ�������
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_Writ_Bus(uint8_t dat)
//{
//	/*** ��CS�õ� ***/
//	LCD_CS_Clr();
//	//�������ݣ����ݿ�ʼ�����BSY����λ������ͨ����λ�Ĵ�����λ�����DRΪ��ʱ������ʾ�����ѷ�����ɡ����ݿ��ܻ�������λ�Ĵ����У�δ��ȫ������ɣ�
//	SPI1->DR = dat;	
//	//delay(2);	//BUSY��־λ����������λ����Ҫ��ʱ
//	while(__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_BSY) != RESET);	//�ж��Ƿ�����ɣ��������ʱ��BUSY����0
//	
//	while (__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte
//		dat = SPI1->DR;		//��DR�Ĵ�����������ձ�־λ������ʹ�õ���ȫ˫��ģʽ����������һ����Ӱ�������豸ʹ��SPI1���ߣ�
//		
//	LCD_CS_Set();
//}

///******************************************************************************
//      ����˵����LCDд������
//      ������ݣ�dat д�������
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_WR_DATA8(uint8_t dat)
//{
//	LCD_Writ_Bus(dat);
//}


///******************************************************************************
//      ����˵����LCDд������
//      ������ݣ�dat д�������
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_WR_DATA(uint16_t dat)
//{
//	LCD_Writ_Bus(dat>>8);
//	LCD_Writ_Bus(dat);
//}


///******************************************************************************
//      ����˵����LCDд������
//      ������ݣ�dat д�������
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_WR_REG(uint8_t dat)
//{
//	LCD_DC_Clr();//д����
//	LCD_Writ_Bus(dat);
//	LCD_DC_Set();//д����
//}


///******************************************************************************
//      ����˵����������ʼ�ͽ�����ַ
//      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
//                y1,y2 �����е���ʼ�ͽ�����ַ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
//{
//#ifdef ST7735S_1_8		//1.8��LCD
//	if(USE_HORIZONTAL==0)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+2);
//		LCD_WR_DATA(x2+2);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+1);
//		LCD_WR_DATA(y2+1);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else if(USE_HORIZONTAL==1)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+2);
//		LCD_WR_DATA(x2+2);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+1);
//		LCD_WR_DATA(y2+1);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else if(USE_HORIZONTAL==2)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+1);
//		LCD_WR_DATA(x2+1);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+2);
//		LCD_WR_DATA(y2+2);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+1);
//		LCD_WR_DATA(x2+1);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+2);
//		LCD_WR_DATA(y2+2);
//		LCD_WR_REG(0x2c);//������д
//	}
//#else		//0.96��LCD
//	if(USE_HORIZONTAL==0)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+26);
//		LCD_WR_DATA(x2+26);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+1);
//		LCD_WR_DATA(y2+1);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else if(USE_HORIZONTAL==1)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+26);
//		LCD_WR_DATA(x2+26);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+1);
//		LCD_WR_DATA(y2+1);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else if(USE_HORIZONTAL==2)
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+1);
//		LCD_WR_DATA(x2+1);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+26);
//		LCD_WR_DATA(y2+26);
//		LCD_WR_REG(0x2c);//������д
//	}
//	else
//	{
//		LCD_WR_REG(0x2a);//�е�ַ����
//		LCD_WR_DATA(x1+1);
//		LCD_WR_DATA(x2+1);
//		LCD_WR_REG(0x2b);//�е�ַ����
//		LCD_WR_DATA(y1+26);
//		LCD_WR_DATA(y2+26);
//		LCD_WR_REG(0x2c);//������д
//	}	
//#endif
//}

//void LCD_Init(void)
//{
//	SPI1_LCD_ReInit(SPI_BAUDRATEPRESCALER_16);	//��ʼ��SPI
//	LCD_RES_Clr();  //��λ
//	User_Delay(100);
//	LCD_RES_Set();
//	User_Delay(100);
//	
//	LCD_BLK_Set();//�򿪱���
//	User_Delay(100);
//	LCD_WR_REG(0x11);     //Sleep out
//	User_Delay(120);      //Delay 120ms
//	
//#ifdef ST7735S_1_8		//1.8��LCD
//	LCD_WR_REG(0xB1); 
//	LCD_WR_DATA8(0x05); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_REG(0xB2); 
//	LCD_WR_DATA8(0x05);
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_REG(0xB3); 
//	LCD_WR_DATA8(0x05); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_DATA8(0x05); 
//	LCD_WR_DATA8(0x3C); 
//	LCD_WR_DATA8(0x3C); 
//	//------------------------------------End ST7735S Frame Rate---------------------------------// 
//	LCD_WR_REG(0xB4); //Dot inversion 
//	LCD_WR_DATA8(0x03); 
//	//------------------------------------ST7735S Power Sequence---------------------------------// 
//	LCD_WR_REG(0xC0); 
//	LCD_WR_DATA8(0x28); 
//	LCD_WR_DATA8(0x08); 
//	LCD_WR_DATA8(0x04); 
//	LCD_WR_REG(0xC1); 
//	LCD_WR_DATA8(0XC0); 
//	LCD_WR_REG(0xC2); 
//	LCD_WR_DATA8(0x0D); 
//	LCD_WR_DATA8(0x00); 
//	LCD_WR_REG(0xC3); 
//	LCD_WR_DATA8(0x8D); 
//	LCD_WR_DATA8(0x2A); 
//	LCD_WR_REG(0xC4); 
//	LCD_WR_DATA8(0x8D); 
//	LCD_WR_DATA8(0xEE); 
//	//---------------------------------End ST7735S Power Sequence-------------------------------------// 
//	LCD_WR_REG(0xC5); //VCOM 
//	LCD_WR_DATA8(0x1A); 
//	LCD_WR_REG(0x36); //MX, MY, RGB mode 
//	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
//	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
//	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
//	else LCD_WR_DATA8(0xA0); 
//	//------------------------------------ST7735S Gamma Sequence---------------------------------// 
//	LCD_WR_REG(0xE0); 
//	LCD_WR_DATA8(0x04); 
//	LCD_WR_DATA8(0x22); 
//	LCD_WR_DATA8(0x07); 
//	LCD_WR_DATA8(0x0A); 
//	LCD_WR_DATA8(0x2E); 
//	LCD_WR_DATA8(0x30); 
//	LCD_WR_DATA8(0x25); 
//	LCD_WR_DATA8(0x2A); 
//	LCD_WR_DATA8(0x28); 
//	LCD_WR_DATA8(0x26); 
//	LCD_WR_DATA8(0x2E); 
//	LCD_WR_DATA8(0x3A); 
//	LCD_WR_DATA8(0x00); 
//	LCD_WR_DATA8(0x01); 
//	LCD_WR_DATA8(0x03); 
//	LCD_WR_DATA8(0x13); 
//	LCD_WR_REG(0xE1); 
//	LCD_WR_DATA8(0x04); 
//	LCD_WR_DATA8(0x16); 
//	LCD_WR_DATA8(0x06); 
//	LCD_WR_DATA8(0x0D); 
//	LCD_WR_DATA8(0x2D); 
//	LCD_WR_DATA8(0x26); 
//	LCD_WR_DATA8(0x23); 
//	LCD_WR_DATA8(0x27); 
//	LCD_WR_DATA8(0x27); 
//	LCD_WR_DATA8(0x25); 
//	LCD_WR_DATA8(0x2D); 
//	LCD_WR_DATA8(0x3B); 
//	LCD_WR_DATA8(0x00); 
//	LCD_WR_DATA8(0x01); 
//	LCD_WR_DATA8(0x04); 
//	LCD_WR_DATA8(0x13); 
//	//------------------------------------End ST7735S Gamma Sequence-----------------------------// 
//	LCD_WR_REG(0x3A); //65k mode 
//	LCD_WR_DATA8(0x05); 
//	LCD_WR_REG(0x29); //Display on 
//#else						//0.96��LCD
//	LCD_WR_REG(0xB1);     //Normal mode
//	LCD_WR_DATA8(0x05);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_REG(0xB2);     //Idle mode
//	LCD_WR_DATA8(0x05);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_REG(0xB3);     //Partial mode
//	LCD_WR_DATA8(0x05);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_DATA8(0x05);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_DATA8(0x3C);   
//	LCD_WR_REG(0xB4);     //Dot inversion
//	LCD_WR_DATA8(0x03);   
//	LCD_WR_REG(0xC0);     //AVDD GVDD
//	LCD_WR_DATA8(0xAB);   
//	LCD_WR_DATA8(0x0B);   
//	LCD_WR_DATA8(0x04);   
//	LCD_WR_REG(0xC1);     //VGH VGL
//	LCD_WR_DATA8(0xC5);   //C0
//	LCD_WR_REG(0xC2);     //Normal Mode
//	LCD_WR_DATA8(0x0D);   
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_REG(0xC3);     //Idle
//	LCD_WR_DATA8(0x8D);   
//	LCD_WR_DATA8(0x6A);   
//	LCD_WR_REG(0xC4);     //Partial+Full
//	LCD_WR_DATA8(0x8D);   
//	LCD_WR_DATA8(0xEE);   
//	LCD_WR_REG(0xC5);     //VCOM
//	LCD_WR_DATA8(0x0F);   
//	LCD_WR_REG(0xE0);     //positive gamma
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x0E);   
//	LCD_WR_DATA8(0x08);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x10);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x02);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x09);   
//	LCD_WR_DATA8(0x0F);   
//	LCD_WR_DATA8(0x25);   
//	LCD_WR_DATA8(0x36);   
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0x08);   
//	LCD_WR_DATA8(0x04);   
//	LCD_WR_DATA8(0x10);   
//	LCD_WR_REG(0xE1);     //negative gamma
//	LCD_WR_DATA8(0x0A);   
//	LCD_WR_DATA8(0x0D);   
//	LCD_WR_DATA8(0x08);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x0F);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x02);   
//	LCD_WR_DATA8(0x07);   
//	LCD_WR_DATA8(0x09);   
//	LCD_WR_DATA8(0x0F);   
//	LCD_WR_DATA8(0x25);   
//	LCD_WR_DATA8(0x35);   
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0x09);   
//	LCD_WR_DATA8(0x04);   
//	LCD_WR_DATA8(0x10);
//		 
//	LCD_WR_REG(0xFC);    
//	LCD_WR_DATA8(0x80);  
//		
//	LCD_WR_REG(0x3A);     
//	LCD_WR_DATA8(0x05);   
//	LCD_WR_REG(0x36);
//	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
//	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
//	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
//	else LCD_WR_DATA8(0xA8);   
//	LCD_WR_REG(0x21);     //Display inversion
//	LCD_WR_REG(0x29);     //Display on
//	LCD_WR_REG(0x2A);     //Set Column Address
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0x1A);  //26  
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0x69);   //105 
//	LCD_WR_REG(0x2B);     //Set Page Address
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0x01);    //1
//	LCD_WR_DATA8(0x00);   
//	LCD_WR_DATA8(0xA0);    //160
//	LCD_WR_REG(0x2C); 
//#endif	
//}








