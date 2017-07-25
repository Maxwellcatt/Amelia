/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	pwm.c
 -- DESCRIPTION 	: 	PWM输出
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/11    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#include	"pwm.h"
#include	"sys.h"
#include 	"delay.h"
#include	"global.h"
	
long 	temp_CH1	=	0,	temp_CH2	=	0,	temp_CH3	=	0,	
			temp_CH4	=	0,	temp_CH5	=	0,	temp_CH6	=	0;
extern u8  TIM5CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值
extern u8  TIM4CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM4CH1_CAPTURE_VAL;	//输入捕获值
extern u8  TIM3CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM3CH1_CAPTURE_VAL;	//输入捕获值
extern u8  TIM2CH2_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM2CH2_CAPTURE_VAL;	//输入捕获值
extern u8  TIM8CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM8CH1_CAPTURE_VAL;	//输入捕获值
extern u8  TIM9CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u32	TIM9CH1_CAPTURE_VAL;	//输入捕获值
	
//------------------------------------------------------------------------------
//	NAME					:		Set_Motor_Speed
//	DESCRIPTION 	: 	设定电机速度
//------------------------------------------------------------------------------

void	Set_Motor_Speed(unsigned int	speed_1,	unsigned int	speed_2,	
												unsigned int	speed_3,	unsigned int	speed_4)
{
	TIM1	->	CCR1 = speed_1;
	TIM1	->	CCR2 = speed_2;
	TIM1	->	CCR3 = speed_3;
	TIM1	->	CCR4 = speed_4;
}


//------------------------------------------------------------------------------
//	NAME					:		Read_Remote
//	DESCRIPTION 	: 	读取遥控器
//------------------------------------------------------------------------------
	
void	Read_Remote()
{
	if(TIM2CH2_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH1	=	TIM2CH2_CAPTURE_STA	&	0X3F; 
		temp_CH1	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH1	+=	TIM2CH2_CAPTURE_VAL;		   //得到总的高电平时间
		//printf("CH1: %ld  \r",	temp_CH1); //打印总的高点平时间
		TIM2CH2_CAPTURE_STA	=	0;			     //开启下一次捕获
	}	
	
	if(TIM3CH1_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH2	=	TIM3CH1_CAPTURE_STA	&	0X3F; 
		temp_CH2	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH2	+=	TIM3CH1_CAPTURE_VAL;		   //得到总的高电平时间
		//printf("CH2: %ld  \r",	temp_CH2); //打印总的高点平时间
		TIM3CH1_CAPTURE_STA	=	0;			     //开启下一次捕获
	}
		
	if(TIM4CH1_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH3	=	TIM4CH1_CAPTURE_STA	&	0X3F; 
		temp_CH3	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH3	+=	TIM4CH1_CAPTURE_VAL;		   //得到总的高电平时间
		//printf("CH3: %ld  \n",	temp_CH3); //打印总的高点平时间
		TIM4CH1_CAPTURE_STA	=	0;			     //开启下一次捕获
	}
		
	if(TIM5CH1_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH4	=	TIM5CH1_CAPTURE_STA	&	0X3F; 
		temp_CH4	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH4	+=	TIM5CH1_CAPTURE_VAL;		   //得到总的高电平时间
		//printf("CH4: %ld  \n",	temp_CH4); //打印总的高点平时间
		TIM5CH1_CAPTURE_STA	=	0;			     //开启下一次捕获
	}
	
	/*if(TIM8CH1_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH4	=	TIM8CH1_CAPTURE_STA	&	0X3F; 
		temp_CH4	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH4	+=	TIM8CH1_CAPTURE_VAL;		   //得到总的高电平时间
		printf("CH4: %ld  \n",	temp_CH4); //打印总的高点平时间
		TIM8CH1_CAPTURE_STA	=	0;			     //开启下一次捕获
	}*/
		
	/*if(TIM9CH1_CAPTURE_STA	&	0X80)        //成功捕获到了一次高电平
	{
		temp_CH6	=	TIM9CH1_CAPTURE_STA	&	0X3F; 
		temp_CH6	*=	0XFFFF;		 		         //溢出时间总和
		temp_CH6	+=	TIM9CH1_CAPTURE_VAL;		   //得到总的高电平时间
		printf("CH4: %ld  \r",	temp_CH6); //打印总的高点平时间
		TIM9CH1_CAPTURE_STA	=	0;			     //开启下一次捕获
	}*/
}