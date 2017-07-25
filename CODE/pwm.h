/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	pwm.h
 -- DESCRIPTION 	: 	PWM输出
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/11    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#ifndef	__PWM_H__
	#define	__PWM_H__
		
	void	Read_Remote(void);
	void	Set_Motor_Speed(unsigned int	speed_1,	unsigned int	speed_2,
													unsigned int	speed_3,	unsigned int	speed_4);	
			
#endif	
