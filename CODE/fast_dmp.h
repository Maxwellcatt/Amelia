/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	fast_dmp.h
 -- DESCRIPTION 	: 	互补滤波姿态解算
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/7/12    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#ifndef	__FAST_DMP_H__
	#define	__FAST_DMP_H__
	
	#define PI	3.14159265f
	#define	K1	0.1	//互补滤波比例系数
	#define	K2	0.3	//互补滤波比例系数
	#define	DT	0.00001	//互补滤波时间系数
	
	float	Fst_Comple_Filter_X(float	angle_m,	float	gyro_m);
	float	Fst_Comple_Filter_Y(float	angle_m,	float	gyro_m);
	float	Sec_Comple_Filter(float	angle_m,	float	gyro_m);
	float	FIR_Filter_X(float input);
	float	FIR_Filter_Y(float input);
	void	Get_Angle(void); 
	
#endif
	