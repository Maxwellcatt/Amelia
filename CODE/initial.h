/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	initial.h
 -- DESCRIPTION 	: 	初始化
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/9    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#ifndef	__INITIAL_h__
	#define	__INITIAL_h__

	void	IO_Init(void);	//初始化函数
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
