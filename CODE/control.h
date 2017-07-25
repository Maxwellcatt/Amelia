/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	control.h
 -- DESCRIPTION 	: 	双环PID控制
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/14    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

	#ifndef	__CONTROL_H__
		#define	__CONTROL_H__
		
		#define moto_max 1200                           //电机转速限幅
		#define PIDMAX 100                              //PID限幅
		#define Pitch_Makeup -200.0                      	//俯仰零偏补偿
		#define Roll_Makeup 0.0                        	//滚转零偏补偿
		#define Yaw_Makeup 0.0                          //偏航零偏补偿
		
		void	RC_Calcu(void);
		void	RC_Calcu_Sport(void);
		void 	PID_Angle(void);
		void 	PID_Rate(void);
		void	PID_Sport(void);
		void 	PID_Output(void);
		void	ESC_Salibration_Mode(void);
		
	#endif	
	