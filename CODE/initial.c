/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	initial.c
 -- DESCRIPTION 	: 	��ʼ��
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/5/24   Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#include	"sys.h"
#include 	"usart.h"
#include 	"delay.h"
#include	"global.h"
#include	"initial.h"

//------------------------------------------------------------------------------
//	NAME					:		GPIO��ʼ������
//	DESCRIPTION 	: 	��ʼ��
//------------------------------------------------------------------------------
	
void	IO_Init()
{
	//LED����
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);//ʹ��GPIOA
	//GPIOA12��ʼ������
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA,	&GPIO_InitStructure);//��ʼ��
	GPIO_ResetBits(GPIOA,	GPIO_Pin_12);
	
	//����������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);//ʹ��GPIOA
	//GPIOA12��ʼ������
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA,	&GPIO_InitStructure);//��ʼ��
	GPIO_ResetBits(GPIOA,	GPIO_Pin_6);
		
	//KEY���ų�ʼ��
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,	ENABLE);//ʹ��GPIOD
//	GPIO_InitStructure.GPIO_Pin 	=	GPIO_Pin_13; //KEY��Ӧ����
//	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;//��ͨ����ģʽ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//����
//	GPIO_Init(GPIOD,	&GPIO_InitStructure);//��ʼ��
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM��ʼ������
//	DESCRIPTION 	: 	TIM1��ʼ��
//------------------------------------------------------------------------------

void PWM_Init(u32 arr,	u16	psc)
{		 					 
	//�˲������ֶ��޸�IO������
	GPIO_InitTypeDef					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef					TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,	ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,	GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,	GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,	GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,	GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��1
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//GPIOA
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,	&GPIO_InitStructure);              //��ʼ��PF9
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,	&TIM_TimeBaseStructure);//��ʼ����ʱ��1
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode 				= TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low; //�������:TIM����Ƚϼ���
  TIM_OCInitStructure.TIM_OCNPolarity 	= TIM_OCNPolarity_Low;//���ͬ�࣬TIM_OCNPolarity_Highʱ�������
  TIM_OCInitStructure.TIM_OCIdleState 	= TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState 	= TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC3
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC4
	TIM_OC1PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //ʹ��TIM1��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //ʹ��TIM1��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM1, 	TIM_OCPreload_Enable);  //ʹ��TIM1��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM1,	ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1,	ENABLE);	
}

//------------------------------------------------------------------------------
//	NAME					:		�жϳ�ʼ������
//	DESCRIPTION 	: 	��ʼ��
//------------------------------------------------------------------------------
void EXTI_Configuration(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ע��Ҫ��SYSCFGʱ��

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
		
  EXTI_InitTypeDef EXTI_InitStructure;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);    //407ʹ�õ����ú���
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
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM5��ʼ��
//------------------------------------------------------------------------------

void TIM5_Cap_Init(u32 arr,u16 psc)
{
	TIM_ICInitTypeDef  				TIM5_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,	ENABLE);  	//TIM5ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,	ENABLE); //ʹ��PORTAʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,	&GPIO_InitStructure); //��ʼ��PA

	GPIO_PinAFConfig(GPIOA,	GPIO_PinSource0,	GPIO_AF_TIM5); //PA0����λ��ʱ��5
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM5,	&TIM_TimeBaseStructure);
	
	//��ʼ��TIM5���벶�����
		
	TIM5_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI5��
	TIM5_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM5_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI5��
	TIM5_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM5_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE	CC2IE�����ж�
	TIM_ITConfig(TIM5,	TIM_IT_CC1,ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM5,	ENABLE ); 	//ʹ�ܶ�ʱ��5

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=	2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM5CH1_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��5�жϷ������	 
void	TIM5_IRQHandler(void)
{ 		    
	if((TIM5CH1_CAPTURE_STA&0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM5CH1_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM5CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM5CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM5CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM5CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM5CH1_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM5CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM5CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM5);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC1PolarityConfig(TIM5,	TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM5CH1_CAPTURE_STA	=	0;			//���
				TIM5CH1_CAPTURE_VAL	=	0;
				TIM5CH1_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				TIM_SetCounter(TIM5,	0);
				TIM_OC1PolarityConfig(TIM5,	TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM5,	ENABLE); 	//ʹ�ܶ�ʱ��5
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1	|	TIM_IT_Update); //����жϱ�־λ	
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM3��ʼ��
//------------------------------------------------------------------------------
	
void	TIM3_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM3_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,	ENABLE);  	//TIM3ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //ʹ��PORTBʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_4; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //��ʼ��PB

	GPIO_PinAFConfig(GPIOB,	GPIO_PinSource4,	GPIO_AF_TIM3); //PB4����λ��ʱ��3
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM3,	&TIM_TimeBaseStructure);
	//��ʼ��TIM3���벶�����
		
	TIM3_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM3_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM3_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI3��
	TIM3_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM3_ICInitStructure.TIM_ICFilter 		= 0x00;//IC3F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
			
	TIM_ITConfig(TIM3,	TIM_IT_CC1,	ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM3,	ENABLE ); 	//ʹ�ܶ�ʱ��3

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM3CH1_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM3CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��3�жϷ������	 
void	TIM3_IRQHandler(void)
{
	if((TIM3CH1_CAPTURE_STA	&	0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM3CH1_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM3CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM3CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM3CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM3CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM3CH1_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM3CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM3CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM3);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC1PolarityConfig(TIM3,	TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM3CH1_CAPTURE_STA	=	0;			//���
				TIM3CH1_CAPTURE_VAL	=	0;
				TIM3CH1_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				//TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��3
				TIM_SetCounter(TIM3,	0);
				TIM_OC1PolarityConfig(TIM3,	TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM3,	ENABLE); 	//ʹ�ܶ�ʱ��3
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1	|	TIM_IT_Update); //����жϱ�־λ	
}
	
	
//------------------------------------------------------------------------------
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM4��ʼ��
//------------------------------------------------------------------------------
void	TIM4_Cap_Init(u32	arr,	u16	psc)
{
	TIM_ICInitTypeDef  				TIM4_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,	ENABLE);  	//TIM4ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //ʹ��PORTBʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6; //GPIOB
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //��ʼ��PB

	GPIO_PinAFConfig(GPIOB,	GPIO_PinSource6,	GPIO_AF_TIM4); //PB6����λ��ʱ��4
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM4,	&TIM_TimeBaseStructure);
	
	//��ʼ��TIM3���벶�����
		
	TIM4_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI4��
	TIM4_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI4��
	TIM4_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI4��
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
			
	TIM_ITConfig(TIM4,	TIM_IT_CC1,	ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM4,	ENABLE); 	//ʹ�ܶ�ʱ��4

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM4CH1_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM4CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��4�жϷ������	 
void TIM4_IRQHandler(void)
{
	if((TIM4CH1_CAPTURE_STA	&	0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM4CH1_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM4CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM4CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM4CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM4CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM4CH1_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM4CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM4CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM4);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC1PolarityConfig(TIM4,	TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM4CH1_CAPTURE_STA	=	0;			//���
				TIM4CH1_CAPTURE_VAL	=	0;
				TIM4CH1_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				TIM_SetCounter(TIM4,	0);
				TIM_OC1PolarityConfig(TIM4,	TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM4,	ENABLE); 	//ʹ�ܶ�ʱ��4
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1	|	TIM_IT_Update); //����жϱ�־λ	
}

//------------------------------------------------------------------------------
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM2��ʼ��
//------------------------------------------------------------------------------	
	
void	TIM2_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM2_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,	ENABLE);  	//TIM2ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,	ENABLE); //ʹ��PORTBʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,	&GPIO_InitStructure); //��ʼ��PB

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2); //PB3����λ��ʱ��2
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM2,	&TIM_TimeBaseStructure);
	
	//��ʼ��TIM3���벶�����
		
	TIM2_ICInitStructure.TIM_Channel 			= TIM_Channel_2; //CC2S=01 	ѡ������� IC2ӳ�䵽TI2��
	TIM2_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM2_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI2��
	TIM2_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM2_ICInitStructure.TIM_ICFilter	 		= 0x00;//IC2F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE	CC2IE�����ж�
	TIM_ITConfig(TIM2,	TIM_IT_CC2,	ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��2

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM2CH2_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM2CH2_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��2�жϷ������	 
void	TIM2_IRQHandler(void)
{ 		    
	if((TIM2CH2_CAPTURE_STA	&	0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM2CH2_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH2_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH2_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM2CH2_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM2CH2_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//����1���������¼�
		{	
			if(TIM2CH2_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM2CH2_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM2CH2_CAPTURE_VAL	=	TIM_GetCapture2(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC2PolarityConfig(TIM2,	TIM_ICPolarity_Rising); //CC2P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH2_CAPTURE_STA	=	0;			//���
				TIM2CH2_CAPTURE_VAL	=	0;
				TIM2CH2_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				//TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��2
				TIM_SetCounter(TIM2,	0);
				TIM_OC2PolarityConfig(TIM2,	TIM_ICPolarity_Falling);		//CC2P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM2,	ENABLE); 	//ʹ�ܶ�ʱ��2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2	|	TIM_IT_Update); //����жϱ�־λ	
}

//------------------------------------------------------------------------------
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM8��ʼ��
//------------------------------------------------------------------------------	
	
void	TIM8_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM8_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,	ENABLE);  	//TIM8ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,	ENABLE); //ʹ��PORTCʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,	&GPIO_InitStructure); //��ʼ��PC

	GPIO_PinAFConfig(GPIOC,	GPIO_PinSource6,	GPIO_AF_TIM8); //PC6����λ��ʱ��8
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM8,	&TIM_TimeBaseStructure);
	
	//��ʼ��TIM8���벶�����
		
	TIM8_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC4S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM8_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM8_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM8_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM8_ICInitStructure.TIM_ICFilter 		= 0x00;//IC4F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM8, &TIM8_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE	CC2IE�����ж�
	TIM_ITConfig(TIM8,	TIM_IT_CC1,	ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM8,	ENABLE ); 	//ʹ�ܶ�ʱ��8

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM8CH1_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM8CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��3�жϷ������	 
void	TIM8_CC_IRQHandler(void)
{ 		    
	if((TIM8CH1_CAPTURE_STA	&	0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM8,	TIM_IT_Update) != RESET)//���
		{	     
			if(TIM8CH1_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM8CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM8CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM8CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM8CH1_CAPTURE_STA	++;
			}	 
		}
		if(TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM8CH1_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM8CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM8CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM8);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC1PolarityConfig(TIM8,	TIM_ICPolarity_Rising); //CC4P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM8CH1_CAPTURE_STA	=	0;			//���
				TIM8CH1_CAPTURE_VAL	=	0;
				TIM8CH1_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				TIM_SetCounter(TIM8,	0);
				TIM_OC1PolarityConfig(TIM8,	TIM_ICPolarity_Falling);		//CC4P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM8,	ENABLE); 	//ʹ�ܶ�ʱ��2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM8, TIM_IT_CC1	|	TIM_IT_Update); //����жϱ�־λ	
}
	
//------------------------------------------------------------------------------
//	NAME					:		PWM�����ʼ������
//	DESCRIPTION 	: 	TIM9��ʼ��
//------------------------------------------------------------------------------	
	
void	TIM9_Cap_Init(u32 arr,	u16 psc)
{
	TIM_ICInitTypeDef  				TIM9_ICInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	NVIC_InitTypeDef 					NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,	ENABLE);  	//TIM9ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,	ENABLE); //ʹ��PORTEʱ��	
		
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //��©�������
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP; //����
	GPIO_Init(GPIOE,	&GPIO_InitStructure); //��ʼ��PE

	GPIO_PinAFConfig(GPIOE,	GPIO_PinSource5,	GPIO_AF_TIM9); //PE5����λ��ʱ��9
		
	TIM_TimeBaseStructure.TIM_Prescaler			=	psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode		=	TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period				=	arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision	=	TIM_CKD_DIV1; 
		
	TIM_TimeBaseInit(TIM9,	&TIM_TimeBaseStructure);
	
	//��ʼ��TIM9���벶�����
		
	TIM9_ICInitStructure.TIM_Channel 			= TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI9��
	TIM9_ICInitStructure.TIM_ICPolarity 	= TIM_ICPolarity_Rising;	//�����ز���
	TIM9_ICInitStructure.TIM_ICSelection 	= TIM_ICSelection_DirectTI; //ӳ�䵽TI9��
	TIM9_ICInitStructure.TIM_ICPrescaler 	= TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM9_ICInitStructure.TIM_ICFilter 		= 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM9, &TIM9_ICInitStructure);
			
	//TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE	CC2IE�����ж�
	TIM_ITConfig(TIM9,	TIM_IT_CC1,ENABLE);//����������ж� ,����CC1IE	CC2IE�Ȳ����ж�
		
	TIM_Cmd(TIM9,	ENABLE); 	//ʹ�ܶ�ʱ��2

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =	0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM9CH1_CAPTURE_STA	=	0;	//���벶��״̬
u32	TIM9CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
//��ʱ��3�жϷ������	 
void	TIM9_IRQHandler(void)
{ 		    
	if((TIM9CH1_CAPTURE_STA	&	0X80)	==	0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM9CH1_CAPTURE_STA	&	0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM9CH1_CAPTURE_STA	&	0X3F)	==	0X3F)//�ߵ�ƽ̫����
				{
					TIM9CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�������һ��
					TIM9CH1_CAPTURE_VAL	=	0XFFFFFFFF;
				}
				else TIM9CH1_CAPTURE_STA	++	;
			}	 
		}
		if(TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM9CH1_CAPTURE_STA	&	0X40)		//����һ���½��� 		
			{	  			
				TIM9CH1_CAPTURE_STA	|=	0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM9CH1_CAPTURE_VAL	=	TIM_GetCapture1(TIM9);//��ȡ��ǰ�Ĳ���ֵ.
				TIM_OC1PolarityConfig(TIM9,	TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM9CH1_CAPTURE_STA	=	0;			//���
				TIM9CH1_CAPTURE_VAL	=	0;
				TIM9CH1_CAPTURE_STA	|=	0X40;		//��ǲ�����������
				//TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��2
				TIM_SetCounter(TIM9,	0);
				TIM_OC1PolarityConfig(TIM9,	TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM9,	ENABLE); 	//ʹ�ܶ�ʱ��2
			}		    
		}			     	    					   
	}
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC1	|	TIM_IT_Update); //����жϱ�־λ	
}