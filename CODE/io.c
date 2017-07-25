/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	io.c
 -- DESCRIPTION 	: 	LED�Ͱ���
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#include	"sys.h"
#include 	"delay.h"
#include	"global.h"
	
extern	float	pitch_makeup,	roll_makeup;
extern	int 	AngleX, AngleY;

//------------------------------------------------------------------------------
//	NAME					:		LED����
//	DESCRIPTION 	: 	LED����	���ղ���HIGH	��	LOW��
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
//	NAME					:		KEY����
//	DESCRIPTION 	: 	��������
//------------------------------------------------------------------------------
unsigned	int	key_read	()
{
	unsigned	int	key_flag;
	key_flag	=	MIN;
	if	(PDin(13)	==	LOW)
	{
		delay_ms(10);	//����
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
//	NAME					:		����������
//	DESCRIPTION 	: 	���������ƣ�HIGH��
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
	
			

	