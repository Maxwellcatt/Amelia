/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	initial.c
 -- DESCRIPTION 	: 	初始化
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/5/24   Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#include	"sys.h"
#include 	"usart.h"
#include 	"delay.h"
#include	"global.h"
#include	"initial.h"

//------------------------------------------------------------------------------
//	NAME					:		GPIO初始化函数
//	DESCRIPTION 	: 	初始化
//------------------------------------------------------------------------------
	
void	IO_Init()
{
	//LED引脚
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);//使能GPIOA
	//GPIOA12初始化设置
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA,	&GPIO_InitStructure);//初始化
	GPIO_ResetBits(GPIOA,	GPIO_Pin_12);
	
	//蜂鸣器引脚
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);//使能GPIOA
	//GPIOA12初始化设置
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA,	&GPIO_InitStructure);//初始化
	GPIO_ResetBits(GPIOA,	GPIO_Pin_6);
		
	//KEY引脚初始化
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,	ENABLE);//使能GPIOD
//	GPIO_InitStructure.GPIO_Pin 	=	GPIO_Pin_13; //KEY对应引脚
//	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;//普通输入模式
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//上拉
//	GPIO_Init(GPIOD,	&GPIO_InitStructure);//初始化
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM初始化函数
//	DESCRIPTION 	: 	TIM1初始化
//------------------------------------------------------------------------------

void PWM_Init(u32 arr,	u16	psc)
{		 					 
	//此部分需手动修改IO口设置
	GPIO_InitTypeDef					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef					TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,	ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,	GPIO_AF_TIM1); //GPIOF9复用为定时器1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,	GPIO_AF_TIM1); //GPIOF9复用为定时器1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,	GPIO_AF_TIM1); //GPIOF9复用为定时器1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,	GPIO_AF_TIM1); //GPIOF9复用为定时器1
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//GPIOA
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,	&GPIO_InitStructure);              //初始化PF9
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,	&TIM_TimeBaseStructure);//初始化定时器1
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode 				= TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low; //输出极性:TIM输出比较极性
  TIM_OCInitStructure.TIM_OCNPolarity 	= TIM_OCNPolarity_Low;//输出同相，TIM_OCNPolarity_High时输出反相
  TIM_OCInitStructure.TIM_OCIdleState 	= TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState 	= TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC3
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC4
	TIM_OC1PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //使能TIM1在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //使能TIM1在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //使能TIM1在CCR4上的预装载寄存器
	TIM_ARRPreloadConfig(TIM1,	ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM1,	ENABLE);	
}

//------------------------------------------------------------------------------
//	NAME					:		中断初始化函数
//	DESCRIPTION 	: 	初始化
//------------------------------------------------------------------------------
void EXTI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//注意要打开SYSCFG时钟

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
  EXTI_InitTypeDef EXTI_InitStructure;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);    //407使用的配置函数
  EXTI_InitStructure.EXTI_Line 		= EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
		
  NVIC_InitTypeDef NVIC_InitStructure;


  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM5初始化
//------------------------------------------------------------------------------

void TIM5_Cap_Init(u32 arr,u16 psc)
{
	TIM_ICInitTypeDef  				TIM5_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,	ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE); //使能PORTA时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOA,	&GPIO_InitStructure); //初始化PA

	GPIO_PinAFConfig(GPIOA,	GPIO_PinSource0,	GPIO_AF_TIM5); //PA0复用位定时器5
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM5,	&TIM_TimeBaseStructure);
	
	//初始化TIM5输入捕获参数
		
	TIM5_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI5上
	TIM5_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM5_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI5上
	TIM5_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM5_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE	CC2IE捕获中断
	TIM_ITConfig(TIM5,	TIM_IT_CC1,ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM5,	ENABLE ); 	//使能定时器5

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=	2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM5CH1_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器5中断服务程序	 
void	TIM5_IRQHandler(void)
{ 		    
	if((TIM5CH1_CAPTURE_STA&0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM5CH1_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM5CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM5CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM5CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM5CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM5CH1_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM5CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM5CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM5);//获取当前的捕获值.
				TIM_OC1PolarityConfig(TIM5,	TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM5CH1_CAPTURE_STA	=	0;			//清空
				TIM5CH1_CAPTURE_VAL	=	0;
				TIM5CH1_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				TIM_SetCounter(TIM5,	0);
				TIM_OC1PolarityConfig(TIM5,	TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM5,	ENABLE); 	//使能定时器5
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1	|	TIM_IT_Update); //清除中断标志位	
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM3初始化
//------------------------------------------------------------------------------
	
void	TIM3_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM3_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,	ENABLE);  	//TIM3时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //使能PORTB时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //初始化PB

	GPIO_PinAFConfig(GPIOB,	GPIO_PinSource4,	GPIO_AF_TIM3); //PB4复用位定时器3
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM3,	&TIM_TimeBaseStructure);
	//初始化TIM3输入捕获参数
		
	TIM3_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM3_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI3上
	TIM3_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM3_ICInitStructure.TIM_ICFilter 		= 0x00;//IC3F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
			
	TIM_ITConfig(TIM3,	TIM_IT_CC1,	ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM3,	ENABLE ); 	//使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM3CH1_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM3CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器3中断服务程序	 
void	TIM3_IRQHandler(void)
{
	if((TIM3CH1_CAPTURE_STA	&	0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM3CH1_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM3CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM3CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM3CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM3CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM3CH1_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM3CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM3);//获取当前的捕获值.
				TIM_OC1PolarityConfig(TIM3,	TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH1_CAPTURE_STA	=	0;			//清空
				TIM3CH1_CAPTURE_VAL	=	0;
				TIM3CH1_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				//TIM_Cmd(TIM3,ENABLE ); 	//使能定时器3
				TIM_SetCounter(TIM3,	0);
				TIM_OC1PolarityConfig(TIM3,	TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM3,	ENABLE); 	//使能定时器3
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1	|	TIM_IT_Update); //清除中断标志位	
}
	
	
//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM4初始化
//------------------------------------------------------------------------------
void	TIM4_Cap_Init(u32	arr,	u16	psc)
{
	TIM_ICInitTypeDef  				TIM4_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,	ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //使能PORTB时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6; //GPIOB
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //初始化PB

	GPIO_PinAFConfig(GPIOB,	GPIO_PinSource6,	GPIO_AF_TIM4); //PB6复用位定时器4
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM4,	&TIM_TimeBaseStructure);
	
	//初始化TIM3输入捕获参数
		
	TIM4_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI4上
	TIM4_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI4上
	TIM4_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI4上
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
			
	TIM_ITConfig(TIM4,	TIM_IT_CC1,	ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM4,	ENABLE); 	//使能定时器4

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM4CH1_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM4CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{
	if((TIM4CH1_CAPTURE_STA	&	0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM4CH1_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM4CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM4CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM4CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM4CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM4CH1_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM4CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM4CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM4);//获取当前的捕获值.
				TIM_OC1PolarityConfig(TIM4,	TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM4CH1_CAPTURE_STA	=	0;			//清空
				TIM4CH1_CAPTURE_VAL	=	0;
				TIM4CH1_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				TIM_SetCounter(TIM4,	0);
				TIM_OC1PolarityConfig(TIM4,	TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM4,	ENABLE); 	//使能定时器4
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1	|	TIM_IT_Update); //清除中断标志位	
}

//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM2初始化
//------------------------------------------------------------------------------	
	
void	TIM2_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM2_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,	ENABLE);  	//TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //使能PORTB时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //初始化PB

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2); //PB3复用位定时器2
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM2,	&TIM_TimeBaseStructure);
	
	//初始化TIM3输入捕获参数
		
	TIM2_ICInitStructure.TIM_Channel 			= TIM_Channel_2; //CC2S=01 	选择输入端 IC2映射到TI2上
	TIM2_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI2上
	TIM2_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM2_ICInitStructure.TIM_ICFilter	 		= 0x00;//IC2F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE	CC2IE捕获中断
	TIM_ITConfig(TIM2,	TIM_IT_CC2,	ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM2CH2_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM2CH2_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器2中断服务程序	 
void	TIM2_IRQHandler(void)
{ 		    
	if((TIM2CH2_CAPTURE_STA	&	0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM2CH2_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM2CH2_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM2CH2_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM2CH2_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM2CH2_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH2_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH2_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM2CH2_CAPTURE_VAL	=	TIM_GetCapture2(TIM2);//获取当前的捕获值.
				TIM_OC2PolarityConfig(TIM2,	TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH2_CAPTURE_STA	=	0;			//清空
				TIM2CH2_CAPTURE_VAL	=	0;
				TIM2CH2_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				//TIM_Cmd(TIM3,ENABLE ); 	//使能定时器2
				TIM_SetCounter(TIM2,	0);
				TIM_OC2PolarityConfig(TIM2,	TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
				TIM_Cmd(TIM2,	ENABLE); 	//使能定时器2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2	|	TIM_IT_Update); //清除中断标志位	
}

//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM8初始化
//------------------------------------------------------------------------------	
	
void	TIM8_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM8_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,	ENABLE);  	//TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,	ENABLE); //使能PORTC时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOC,	&GPIO_InitStructure); //初始化PC

	GPIO_PinAFConfig(GPIOC,	GPIO_PinSource6,	GPIO_AF_TIM8); //PC6复用位定时器8
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM8,	&TIM_TimeBaseStructure);
	
	//初始化TIM8输入捕获参数
		
	TIM8_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC4S=01 	选择输入端 IC1映射到TI1上
	TIM8_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM8_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI1上
	TIM8_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM8_ICInitStructure.TIM_ICFilter 		= 0x00;//IC4F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM8, &TIM8_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE	CC2IE捕获中断
	TIM_ITConfig(TIM8,	TIM_IT_CC1,	ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM8,	ENABLE ); 	//使能定时器8

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM8CH1_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM8CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器3中断服务程序	 
void	TIM8_CC_IRQHandler(void)
{ 		    
	if((TIM8CH1_CAPTURE_STA	&	0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM8,	TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM8CH1_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM8CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM8CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM8CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM8CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM8CH1_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM8CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM8CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM8);//获取当前的捕获值.
				TIM_OC1PolarityConfig(TIM8,	TIM_ICPolarity_Rising); //CC4P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM8CH1_CAPTURE_STA	=	0;			//清空
				TIM8CH1_CAPTURE_VAL	=	0;
				TIM8CH1_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				TIM_SetCounter(TIM8,	0);
				TIM_OC1PolarityConfig(TIM8,	TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
				TIM_Cmd(TIM8,	ENABLE); 	//使能定时器2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM8, TIM_IT_CC1	|	TIM_IT_Update); //清除中断标志位	
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM捕获初始化函数
//	DESCRIPTION 	: 	TIM9初始化
//------------------------------------------------------------------------------	
	
void	TIM9_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM9_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,	ENABLE);  	//TIM9时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,	ENABLE); //使能PORTE时钟	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //开漏复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //下拉
	GPIO_Init(GPIOE,	&GPIO_InitStructure); //初始化PE

	GPIO_PinAFConfig(GPIOE,	GPIO_PinSource5,	GPIO_AF_TIM9); //PE5复用位定时器9
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM9,	&TIM_TimeBaseStructure);
	
	//初始化TIM9输入捕获参数
		
	TIM9_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI9上
	TIM9_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//上升沿捕获
	TIM9_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //映射到TI9上
	TIM9_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM9_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM9, &TIM9_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE	CC2IE捕获中断
	TIM_ITConfig(TIM9,	TIM_IT_CC1,ENABLE);//不允许更新中断 ,允许CC1IE	CC2IE等捕获中断
		
	TIM_Cmd(TIM9,	ENABLE); 	//使能定时器2

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM9CH1_CAPTURE_STA	=	0;	//输入捕获状态
u32	TIM9CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
//定时器3中断服务程序	 
void	TIM9_IRQHandler(void)
{ 		    
	if((TIM9CH1_CAPTURE_STA	&	0X80)	==	0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM9CH1_CAPTURE_STA	&	0X40)//已经捕获到高电平了
			{
				if((TIM9CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//高电平太长了
				{
					TIM9CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获了一次
					TIM9CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM9CH1_CAPTURE_STA	++	;
			}	 
		}
		if(TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM9CH1_CAPTURE_STA	&	0X40)		//捕获到一个下降沿 		
			{	  			
				TIM9CH1_CAPTURE_STA	|=	0X80;		//标记成功捕获到一次高电平脉宽
				TIM9CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM9);//获取当前的捕获值.
				TIM_OC1PolarityConfig(TIM9,	TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM9CH1_CAPTURE_STA	=	0;			//清空
				TIM9CH1_CAPTURE_VAL	=	0;
				TIM9CH1_CAPTURE_STA	|=	0X40;		//标记捕获到了上升沿
				//TIM_Cmd(TIM3,ENABLE ); 	//使能定时器2
				TIM_SetCounter(TIM9,	0);
				TIM_OC1PolarityConfig(TIM9,	TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM9,	ENABLE); 	//使能定时器2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC1	|	TIM_IT_Update); //清除中断标志位	
}