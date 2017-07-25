/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	dmp.h
 -- DESCRIPTION 	: 	姿态解算
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
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