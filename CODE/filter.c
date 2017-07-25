/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	filter.c
 -- DESCRIPTION 	: 	卡尔曼滤波
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#include "Filter.h"  //卡尔曼滤波算法

int KalmanFilter_ax(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
		int R = MeasureNoise_R;
		int Q = ProcessNiose_Q;
		static int ax_last;
		int ax_mid = ax_last;
		long ax_now;
		static int ax_p_last;
		long p_mid ;
		long p_now;
		int kg;        
		long temp;

		ax_mid = ax_last;
		p_mid = ax_p_last + Q;                                      //系统当前预测值协方差
		temp = p_mid << 15;
		kg = (temp / ((long)p_mid + R));                            //计算卡尔曼增益
		ax_now = ax_mid + (((long)kg * (ResrcData - ax_mid)) >> 15);//计算当前最优估计值
		p_now = ((long)p_mid * (32768 - kg)) >> 15;                 //系统当前最优估计值协方差     
		ax_p_last = p_now;
		ax_last = ax_now;
		return ax_now;                
}

int KalmanFilter_ay(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
		int R = MeasureNoise_R;
		int Q = ProcessNiose_Q;
		static int ay_last;
		int ay_mid = ay_last;
		long ay_now;
		static int ay_p_last;
		long p_mid ;
		long p_now;
		int kg;        
		long temp;

		ay_mid = ay_last;
		p_mid = ay_p_last + Q; 
		temp = p_mid << 15;
		kg = (temp / ((long)p_mid + R)); 

		ay_now = ay_mid + (((long)kg * (ResrcData - ay_mid)) >> 15);
		p_now = ((long)p_mid * (32768 - kg)) >> 15;       
		ay_p_last = p_now;
		ay_last = ay_now;
		return ay_now;                
}

int KalmanFilter_az(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
		int R = MeasureNoise_R;
		int Q = ProcessNiose_Q;
		static int az_last;
		int az_mid = az_last;
		long az_now;
		static int az_p_last;
		long p_mid ;
		long p_now;
		int kg;        
		long temp;

		az_mid = az_last;
		p_mid = az_p_last + Q;
		temp = p_mid << 15;
		kg = (temp / ((long)p_mid + R)); 

	 az_now = az_mid + (((long)kg * (ResrcData - az_mid)) >> 15);
		p_now = ((long)p_mid * (32768 - kg)) >> 15;      
		az_p_last = p_now;
		az_last = az_now;
		return az_now;                
}

int KalmanFilter_gyroz(int ResrcData, int ProcessNiose_Q, int MeasureNoise_R)
{
		int R = MeasureNoise_R;
		int Q = ProcessNiose_Q;
		static int gyroz_last;
		int gyroz_mid = gyroz_last;
		long gyroz_now;
		static int gyroz_p_last;
		long p_mid ;
		long p_now;
		int kg;        
		long temp;
		 
		gyroz_mid = gyroz_last;
		p_mid = gyroz_p_last + Q;
		temp = p_mid << 15;
		kg = (temp / ((long)p_mid + R)); 

		gyroz_now = gyroz_mid + (((long)kg * (ResrcData - gyroz_mid)) >> 15);
		p_now = ((long)p_mid * (32768 - kg)) >> 15;  
		gyroz_p_last = p_now;
		gyroz_last = gyroz_now;
		return gyroz_now;                
}

	/*
	int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R)
	{
		 int R = MeasureNoise_R;
		 int Q = ProcessNiose_Q;
		 static int gyrox_last;
		 int gyrox_mid = gyrox_last;
		 long gyrox_now;
		 static int gyrox_p_last;
		 long gyrox_p_mid ;
		 long gyrox_p_now;
		 int gyrox_kg;        
		long gyrox_temp;
		 gyrox_mid=gyrox_last;
		 gyrox_p_mid=gyrox_p_last+Q; 
		gyrox_temp=gyrox_p_mid<<15;
		gyrox_kg=(gyrox_temp/((long)gyrox_p_mid+R));

		 gyrox_now= gyrox_mid+(((long)gyrox_kg*(ResrcData-gyrox_mid))>>15);
		 gyrox_p_now=((long)gyrox_p_mid*(32768-gyrox_kg))>>15;     
		 gyrox_p_last = gyrox_p_now;
		 gyrox_last = gyrox_now;
		 return gyrox_now;                
	}

	 int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R)
	{
		 int R = MeasureNoise_R;
		 int Q = ProcessNiose_Q;
		 static int gyroy_last;
		 int gyroy_mid = gyroy_last;
		 long gyroy_now;
		 static int gyroy_p_last;
		 long p_mid ;
		 long p_now;
		 int kg;        
		long temp;
		 gyroy_mid=gyroy_last;
		 p_mid=gyroy_p_last+Q; 
		temp=p_mid<<15;
		kg=(temp/((long)p_mid+R)); 

		 gyroy_now= gyroy_mid+(((long)kg*(ResrcData-gyroy_mid))>>15);
		 p_now=((long)p_mid*(32768-kg))>>15;      
		 gyroy_p_last = p_now;
		 gyroy_last = gyroy_now;
		 return gyroy_now;                
	}
	*/