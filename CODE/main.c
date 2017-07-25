/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	main.c
 -- DESCRIPTION 	: 	������
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/
	
#include	"io.h"
#include	"sys.h"
#include	"pwm.h"
#include	"usart.h"
//#include	"dmp.h"
#include	"delay.h"
#include	"global.h"
#include	"control.h"
#include	"initial.h"
#include 	"mpu60xx.h"
#include	"fast_dmp.h"
#include	"stm32f4xx.h"

extern	long temp_CH1,	temp_CH2,	temp_CH3,	temp_CH4;
static	int	state	=	LOCK;
static	int	mode	=	ANGLE_HOLD;
	
//------------------------------------------------------------------------------
//	NAME					:		������
//	DESCRIPTION 	: 	�㶮��
//------------------------------------------------------------------------------

int	main()	
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2		
	delay_init(168);  			//��ʼ����ʱ����
	PWM_Init(4999,	83);
	TIM5_Cap_Init(0XFFFFFFFF,	83);	//ң������ʱ����ʼ��
	TIM4_Cap_Init(0XFFFFFFFF,	83);	//ң������ʱ����ʼ��
	TIM3_Cap_Init(0XFFFFFFFF,	83);	//ң������ʱ����ʼ��
	TIM2_Cap_Init(0XFFFFFFFF,	83);	//ң������ʱ����ʼ��
	//TIM8_Cap_Init(0XFFFFFFFF,	83);
	//TIM9_Cap_Init(0XFFFFFFFF,	83);
	IO_Init();							//io��ʼ��
	usart_init(9600);			//��ʼ�����ڲ�����Ϊ115200
	EXTI_Configuration();	//�жϳ�ʼ��
	led_switch(LOW);
	mpu_init();	//MPU��ʼ��
	
	Set_Motor_Speed(3100, 3100, 3100, 3100);	//�����ʼ��
	delay_ms(5000);	
	
	bibi(HIGH);	//��ʼ�������ʾ��
	led_switch(HIGH);
	delay_ms(2000);
	bibi(LOW);
	led_switch(LOW);
	state	=	LOCK;
	mode	=	ANGLE_HOLD;
	
	while	(1)
	{
		Read_Remote();
		
		if(temp_CH3	>=	1915	&&	temp_CH4	>=	1915	&&	state	==	LOCK)	//����
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH4	>=	1915)
			{
				state	=	UNLOCK;
				bibi(HIGH);	//������ʾ
				led_switch(LOW);
				delay_ms(500);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(500);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(500);
				bibi(LOW);
				led_switch(HIGH);
				//printf("ulock\n");
			}
		}
		
		else	if(temp_CH3	>=	1915	&&	temp_CH4	<=	1095	&&	state	==	UNLOCK)	//����
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH4	<=	1095)
			{
				state	=	LOCK;
				bibi(HIGH);	//������ʾ
				delay_ms(500);
				bibi(LOW);
				led_switch(LOW);
				//printf("lock\n");
			}
		}
		
		if(temp_CH3	>=	1915	&&	temp_CH1	>=	1915	&&	state	==	LOCK	&&	mode	==	ANGLE_HOLD)	//�˶�ģʽ
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH1	>=	1915)
			{
				//printf("sp\n");
				mode	=	SPORT;
				bibi(HIGH);	//�˶�ģʽ��ʾ
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				led_switch(LOW);
			}
		}
		
		else	if(temp_CH3	>=	1915	&&	temp_CH1	<=	1095	&&	state	==	LOCK	&&	mode	==	SPORT)	//����ģʽ
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH1	<=	1095)
			{
				//printf("ah\n");
				mode	=	ANGLE_HOLD;
				bibi(HIGH);	//�˶�ģʽ��ʾ
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				bibi(HIGH);
				led_switch(LOW);
				delay_ms(100);
				bibi(LOW);
				led_switch(HIGH);
				delay_ms(100);
				led_switch(LOW);
			}
		}
		
		if(state	==	LOCK)
		{
			//printf("lock\n");
			Set_Motor_Speed(3100, 3100, 3100, 3100);
		}
		
		else	if(state	==	UNLOCK)	
		{
			if(mode	==	ANGLE_HOLD)	//����ģʽ
			{
				//printf("ulock\n");
				Read_Remote();
				RC_Calcu();
				Get_Angle();
				PID_Angle();
				PID_Rate();
				PID_Output();
			}
			
			else	if(mode	==	SPORT)	//�˶�ģʽ
			{
				Read_Remote();
				RC_Calcu_Sport();
				Get_Angle();
				PID_Sport();
				PID_Output();
			}
			//ESC_Salibration_Mode();
		}

	}
}
