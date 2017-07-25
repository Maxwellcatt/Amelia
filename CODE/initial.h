/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	initial.h
 -- DESCRIPTION 	: 	��ʼ��
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#ifndef	__INITIAL_h__
	#define	__INITIAL_h__

	void	IO_Init(void);	//��ʼ������
	void 	PWM_Init(u32 arr,	u16	psc);
	void 	TIM5_Cap_Init(u32 arr,u16 psc);
	void 	TIM4_Cap_Init(u32 arr,u16 psc);
	void 	TIM3_Cap_Init(u32 arr,u16 psc);
	void 	TIM2_Cap_Init(u32 arr,u16 psc);
	void 	TIM8_Cap_Init(u32 arr,u16 psc);
	void 	TIM9_Cap_Init(u32 arr,u16 psc);
	void	EXTI_Configuration(void);
	void	NVIC_Config(void);

#endif
