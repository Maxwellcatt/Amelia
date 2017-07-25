/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	pwm.c
 -- DESCRIPTION 	: 	PWM���
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/11    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#include	"pwm.h"
#include	"sys.h"
#include 	"delay.h"
#include	"global.h"
	
long 	temp_CH1	=	0,	temp_CH2	=	0,	temp_CH3	=	0,	
			temp_CH4	=	0,	temp_CH5	=	0,	temp_CH6	=	0;
extern u8  TIM5CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ
extern u8  TIM4CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM4CH1_CAPTURE_VAL;	//���벶��ֵ
extern u8  TIM3CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM3CH1_CAPTURE_VAL;	//���벶��ֵ
extern u8  TIM2CH2_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM2CH2_CAPTURE_VAL;	//���벶��ֵ
extern u8  TIM8CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM8CH1_CAPTURE_VAL;	//���벶��ֵ
extern u8  TIM9CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u32	TIM9CH1_CAPTURE_VAL;	//���벶��ֵ
	
//------------------------------------------------------------------------------
//	NAME					:		Set_Motor_Speed
//	DESCRIPTION 	: 	�趨����ٶ�
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
//	DESCRIPTION 	: 	��ȡң����
//------------------------------------------------------------------------------
	
void	Read_Remote()
{
	if(TIM2CH2_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH1	=	TIM2CH2_CAPTURE_STA	&	0X3F; 
		temp_CH1	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH1	+=	TIM2CH2_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		//printf("CH1: %ld  \r",	temp_CH1); //��ӡ�ܵĸߵ�ƽʱ��
		TIM2CH2_CAPTURE_STA	=	0;			     //������һ�β���
	}	
	
	if(TIM3CH1_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH2	=	TIM3CH1_CAPTURE_STA	&	0X3F; 
		temp_CH2	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH2	+=	TIM3CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		//printf("CH2: %ld  \r",	temp_CH2); //��ӡ�ܵĸߵ�ƽʱ��
		TIM3CH1_CAPTURE_STA	=	0;			     //������һ�β���
	}
		
	if(TIM4CH1_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH3	=	TIM4CH1_CAPTURE_STA	&	0X3F; 
		temp_CH3	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH3	+=	TIM4CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		//printf("CH3: %ld  \n",	temp_CH3); //��ӡ�ܵĸߵ�ƽʱ��
		TIM4CH1_CAPTURE_STA	=	0;			     //������һ�β���
	}
		
	if(TIM5CH1_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH4	=	TIM5CH1_CAPTURE_STA	&	0X3F; 
		temp_CH4	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH4	+=	TIM5CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		//printf("CH4: %ld  \n",	temp_CH4); //��ӡ�ܵĸߵ�ƽʱ��
		TIM5CH1_CAPTURE_STA	=	0;			     //������һ�β���
	}
	
	/*if(TIM8CH1_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH4	=	TIM8CH1_CAPTURE_STA	&	0X3F; 
		temp_CH4	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH4	+=	TIM8CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		printf("CH4: %ld  \n",	temp_CH4); //��ӡ�ܵĸߵ�ƽʱ��
		TIM8CH1_CAPTURE_STA	=	0;			     //������һ�β���
	}*/
		
	/*if(TIM9CH1_CAPTURE_STA	&	0X80)        //�ɹ�������һ�θߵ�ƽ
	{
		temp_CH6	=	TIM9CH1_CAPTURE_STA	&	0X3F; 
		temp_CH6	*=	0XFFFF;		 		         //���ʱ���ܺ�
		temp_CH6	+=	TIM9CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
		printf("CH4: %ld  \r",	temp_CH6); //��ӡ�ܵĸߵ�ƽʱ��
		TIM9CH1_CAPTURE_STA	=	0;			     //������һ�β���
	}*/
}