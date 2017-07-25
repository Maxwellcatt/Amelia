/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	iic.h
 -- DESCRIPTION 	: 	���iic����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/
	#ifndef __IIC_H__
		#define __IIC_H__
		
		#include "sys.h" 	
								 
		//IO��������
		#define SDA_IN()  {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=0<<4*2;}	//PD4����ģʽ
		#define SDA_OUT() {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=1<<4*2;} //PD4���ģʽ
		//IO��������	 
		#define IIC_SCL    PDout(3) //SCL
		#define IIC_SDA    PDout(4) //SDA	 
		#define READ_SDA   PDin(4)  //����SDA 

		//IIC���в�������
		void IIC_Init(void);         //��ʼ��IIC��IO��				 
		void IIC_Start(void);				//����IIC��ʼ�ź�
		void IIC_Stop(void);	  			//����IICֹͣ�ź�
		void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
		u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
		u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
		void IIC_Ack(void);					//IIC����ACK�ź�
		void IIC_NAck(void);				//IIC������ACK�ź�

		void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
		u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
	  
	#endif
