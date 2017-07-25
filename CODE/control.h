/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	control.h
 -- DESCRIPTION 	: 	˫��PID����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/14    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

	#ifndef	__CONTROL_H__
		#define	__CONTROL_H__
		
		#define moto_max 1200                           //���ת���޷�
		#define PIDMAX 100                              //PID�޷�
		#define Pitch_Makeup -200.0                      	//������ƫ����
		#define Roll_Makeup 0.0                        	//��ת��ƫ����
		#define Yaw_Makeup 0.0                          //ƫ����ƫ����
		
		void	RC_Calcu(void);
		void	RC_Calcu_Sport(void);
		void 	PID_Angle(void);
		void 	PID_Rate(void);
		void	PID_Sport(void);
		void 	PID_Output(void);
		void	ESC_Salibration_Mode(void);
		
	#endif	
	