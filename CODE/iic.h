/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	iic.h
 -- DESCRIPTION 	: 	软件iic驱动
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/
	#ifndef __IIC_H__
		#define __IIC_H__
		
		#include "sys.h" 	
								 
		//IO方向设置
		#define SDA_IN()  {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=0<<4*2;}	//PD4输入模式
		#define SDA_OUT() {GPIOD->MODER&=~(3<<(4*2));GPIOD->MODER|=1<<4*2;} //PD4输出模式
		//IO操作函数	 
		#define IIC_SCL    PDout(3) //SCL
		#define IIC_SDA    PDout(4) //SDA	 
		#define READ_SDA   PDin(4)  //输入SDA 

		//IIC所有操作函数
		void IIC_Init(void);         //初始化IIC的IO口				 
		void IIC_Start(void);				//发送IIC开始信号
		void IIC_Stop(void);	  			//发送IIC停止信号
		void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
		u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
		u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
		void IIC_Ack(void);					//IIC发送ACK信号
		void IIC_NAck(void);				//IIC不发送ACK信号

		void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
		u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
	  
	#endif
