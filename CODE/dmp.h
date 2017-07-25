/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	dmp.h
 -- DESCRIPTION 	: 	��̬����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/
	
#ifndef	__DMP_H__
	#define	__DMP_H__
		
	#define	pi		3.14159265f                           
	#define	Kp		2.0f                        
	#define	Ki		0.001f  
	#define Kd    0.001f
	#define	halfT	0.005f
		
	#define Q15(X) ((X < 0.0) ? (int)(32768	*	(X) - 0.5) : (int)(32767	*	(X) + 0.5))
	#define KALMAN_Q	Q15(0.20)
	#define KALMAN_R	Q15(0.80)
		
	float invSqrt(float x);
	void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
	void Angel_Calcu(void);
		
#endif