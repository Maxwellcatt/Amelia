/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	fast_dmp.h
 -- DESCRIPTION 	: 	�����˲���̬����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/7/12    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#ifndef	__FAST_DMP_H__
	#define	__FAST_DMP_H__
	
	#define PI	3.14159265f
	#define	K1	0.1	//�����˲�����ϵ��
	#define	K2	0.3	//�����˲�����ϵ��
	#define	DT	0.00001	//�����˲�ʱ��ϵ��
	
	float	Fst_Comple_Filter_X(float	angle_m,	float	gyro_m);
	float	Fst_Comple_Filter_Y(float	angle_m,	float	gyro_m);
	float	Sec_Comple_Filter(float	angle_m,	float	gyro_m);
	float	FIR_Filter_X(float input);
	float	FIR_Filter_Y(float input);
	void	Get_Angle(void); 
	
#endif
	