/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	io.c
 -- DESCRIPTION 	: 	LED和按键
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#include	"sys.h"
#include 	"delay.h"
#include	"global.h"
	
extern	float	pitch_makeup,	roll_makeup;
extern	int 	AngleX, AngleY;

//------------------------------------------------------------------------------
//	NAME					:		LED函数
//	DESCRIPTION 	: 	LED开关	接收参数HIGH	亮	LOW灭
//------------------------------------------------------------------------------
void	led_switch	(unsigned	int	led_switch)	
{
	if	(led_switch	==	HIGH)
	{
		PAout(LED)	=	HIGH;
	}
	else	if	(led_switch	==	LOW)
	{
		PAout(LED)	=	LOW;
	}
}
	
//------------------------------------------------------------------------------
//	NAME					:		KEY函数
//	DESCRIPTION 	: 	按键输入
//------------------------------------------------------------------------------
unsigned	int	key_read	()
{
	unsigned	int	key_flag;
	key_flag	=	MIN;
	if	(PDin(13)	==	LOW)
	{
		delay_ms(10);	//消抖
		if	(PDin(13)	==	LOW)
		{
			key_flag	=	MAX;
		}
	}
	return	key_flag;
}	


void	sys_set	()
{
	while(key_read())
	{
		pitch_makeup	=	AngleX;
		roll_makeup		=	AngleY;
		led_switch(HIGH);
	}
	led_switch(LOW);
}


//------------------------------------------------------------------------------
//	NAME					:		蜂鸣器函数
//	DESCRIPTION 	: 	蜂鸣器控制，HIGH响
//------------------------------------------------------------------------------
void	bibi(unsigned	int	bibi)	
{
	if	(bibi	==	HIGH)
	{
		PAout(BI)	=	HIGH;
	}
	else	if	(bibi	==	LOW)
	{
		PAout(BI)	=	LOW;
	}
}
	
			

	