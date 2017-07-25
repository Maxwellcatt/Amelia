/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	main.c
 -- DESCRIPTION 	: 	主函数
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
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
//	NAME					:		主函数
//	DESCRIPTION 	: 	你懂得
//------------------------------------------------------------------------------

int	main()	
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2		
	delay_init(168);  			//初始化延时函数
	PWM_Init(4999,	83);
	TIM5_Cap_Init(0XFFFFFFFF,	83);	//遥控器定时器初始化
	TIM4_Cap_Init(0XFFFFFFFF,	83);	//遥控器定时器初始化
	TIM3_Cap_Init(0XFFFFFFFF,	83);	//遥控器定时器初始化
	TIM2_Cap_Init(0XFFFFFFFF,	83);	//遥控器定时器初始化
	//TIM8_Cap_Init(0XFFFFFFFF,	83);
	//TIM9_Cap_Init(0XFFFFFFFF,	83);
	IO_Init();							//io初始化
	usart_init(9600);			//初始化串口波特率为115200
	EXTI_Configuration();	//中断初始化
	led_switch(LOW);
	mpu_init();	//MPU初始化
	
	Set_Motor_Speed(3100, 3100, 3100, 3100);	//电调初始化
	delay_ms(5000);	
	
	bibi(HIGH);	//初始化完毕提示音
	led_switch(HIGH);
	delay_ms(2000);
	bibi(LOW);
	led_switch(LOW);
	state	=	LOCK;
	mode	=	ANGLE_HOLD;
	
	while	(1)
	{
		Read_Remote();
		
		if(temp_CH3	>=	1915	&&	temp_CH4	>=	1915	&&	state	==	LOCK)	//解锁
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH4	>=	1915)
			{
				state	=	UNLOCK;
				bibi(HIGH);	//解锁提示
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
		
		else	if(temp_CH3	>=	1915	&&	temp_CH4	<=	1095	&&	state	==	UNLOCK)	//上锁
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH4	<=	1095)
			{
				state	=	LOCK;
				bibi(HIGH);	//上锁提示
				delay_ms(500);
				bibi(LOW);
				led_switch(LOW);
				//printf("lock\n");
			}
		}
		
		if(temp_CH3	>=	1915	&&	temp_CH1	>=	1915	&&	state	==	LOCK	&&	mode	==	ANGLE_HOLD)	//运动模式
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH1	>=	1915)
			{
				//printf("sp\n");
				mode	=	SPORT;
				bibi(HIGH);	//运动模式提示
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
		
		else	if(temp_CH3	>=	1915	&&	temp_CH1	<=	1095	&&	state	==	LOCK	&&	mode	==	SPORT)	//自稳模式
		{
			Set_Motor_Speed(3100, 3100, 3100, 3100);
			delay_ms(3000);
			Read_Remote();
			//delay_ms(1);
			if(temp_CH3	>=	1915	&&	temp_CH1	<=	1095)
			{
				//printf("ah\n");
				mode	=	ANGLE_HOLD;
				bibi(HIGH);	//运动模式提示
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
			if(mode	==	ANGLE_HOLD)	//自稳模式
			{
				//printf("ulock\n");
				Read_Remote();
				RC_Calcu();
				Get_Angle();
				PID_Angle();
				PID_Rate();
				PID_Output();
			}
			
			else	if(mode	==	SPORT)	//运动模式
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
