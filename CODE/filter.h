/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	filter.h
 -- DESCRIPTION 	: 	卡尔曼滤波
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
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