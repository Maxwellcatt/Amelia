/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	dmp.c
 -- DESCRIPTION 	: 	��̬����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/	
	
#include	"dmp.h"
#include	"math.h"
#include 	"Filter.h"  //�������˲��㷨
#include 	"mpu60xx.h"
	
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
float exDif = 0.0, eyDif = 0.0, ezDif = 0.0;
	
double Gyro_y = 0.0, Gyro_x = 0.0, Gyro_z = 0.0;//Y�������������ݴ�
int Angle_ax = 0, Angle_ay = 0, Angle_az = 0;	  //�ɼ��ٶȼ���ļ��ٶ�(������)
int Angle_gy = 0, Angle_gx = 0, Angle_gz = 0;	  //�ɽ��ٶȼ���Ľ�����(�Ƕ���)
int AngleX = 0, AngleY = 0;				              //��Ԫ���������ŷ���� ����AngleZ=0
int16_t aacx = 0, aacy = 0, aacz = 0;           //������ٶ�
int16_t gyrox = 0, gyroy = 0, gyroz = 0;       	//������ٶ�
float IMU_gz;
float Last_Angle_gx=0.0;                          //�⻷PI�����  ��һ������������
float Last_Angle_gy=0.0;

float invSqrt(float x)   //����˿���ƽ��������
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) & y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) & i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
	
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	static float last_ex,last_ey,last_ez;

	norm = invSqrt(ax * ax + ay * ay + az * az) ;//���ٶ�������ģ
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
		//	�����ǰ���Ԫ������ɷ������Ҿ����еĵ����е�����Ԫ�ء� 
		//	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ��
		//	���������vx vy vz����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������
		//	������λ������
	vx = 2 * (q1 * q3 - q0 * q2);		//T31
	vy = 2 * (q0 * q1 + q2 * q3);		//T32
	vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 ;	//T33

	ex = (ay * vz - az * vy) ;	
	ey = (az * vx - ax * vz) ;
	ez = (ax * vy - ay * vx) ;
		
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

		
	exDif = ex - last_ex;
	eyDif = ey - last_ey;
	ezDif = ez - last_ez;
		
	last_ex = ex;
	last_ey = ey;
	last_ez = ez;
		
	//�ü��ٶȶԽ��ٶȽ����������������Ư��
	gx = gx + Kp * ex + exInt + Kd * exDif;		
	gy = gy + Kp * ey + eyInt + Kd * eyDif;		
	gz = gz + Kp * ez + ezInt + Kd * ezDif;

	//�ý��ٶȸ�����Ԫ��
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;
		
	//��Ԫ���淶�������������������
	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
		
	//����ŷ����
	AngleY = atan2(2 * q2 * q3 + 2 * q0 * q1, - 2 * q1 * q1 - 2 * q2 * q2 + 1) * 572.957795f;
	AngleX = asin(2 * (q0 * q2 - q1 * q3 )) * 572.957795f; 
	//AngleZ = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1)* 572.957795f;
}
	
void Angel_Calcu()
{
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������

	Angle_ax = KalmanFilter_ax((float) aacx, KALMAN_Q, KALMAN_R);  //�������˲�
	Angle_ay = KalmanFilter_ay((float) aacy, KALMAN_Q, KALMAN_R);
	Angle_az = KalmanFilter_az((float) aacz, KALMAN_Q, KALMAN_R);

	Angle_gx = ((float) gyrox) / 65.536;	                   //�����Ǵ���	�����λ�� +-��
	Angle_gy = ((float) gyroy) / 65.536;	                   //���������� +-500��/S, 1��/�� ��Ӧ���� 65.536
	Angle_gz = KalmanFilter_gyroz(gyroz, Q15(0.2), Q15(0.8));
	IMU_gz = Angle_gz / 65.536;
	Last_Angle_gx = Angle_gx;		                            //������һ�ν��ٶ�����
	Last_Angle_gy = Angle_gy;
	Gyro_x = gyrox / 131.00;
	Gyro_y = gyroy / 131.00;
	Gyro_z = gyroz / 131.00;
//*********************************** ��Ԫ������ ***********************************

	IMUupdate(Angle_gx * 0.0174533f, Angle_gy * 0.0174533f, 
					IMU_gz * 0.0174533f, Angle_ax, Angle_ay, Angle_az);//��̬���㣬����0.1��
}	