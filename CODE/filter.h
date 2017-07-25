/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	filter.h
 -- DESCRIPTION 	: 	�������˲�
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#ifndef	__FILTER_H__
	#define	__FILTER_H__
		
	int KalmanFilter_ax(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
	int KalmanFilter_ay(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
	int KalmanFilter_az(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
	//int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
	//int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
	int KalmanFilter_gyroz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R);
			
#endif