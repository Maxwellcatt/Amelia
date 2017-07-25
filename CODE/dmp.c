/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	dmp.c
 -- DESCRIPTION 	: 	姿态解算
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/13    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/	
	
#include	"dmp.h"
#include	"math.h"
#include 	"Filter.h"  //卡尔曼滤波算法
#include 	"mpu60xx.h"
	
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
float exDif = 0.0, eyDif = 0.0, ezDif = 0.0;
	
double Gyro_y = 0.0, Gyro_x = 0.0, Gyro_z = 0.0;//Y轴陀螺仪数据暂存
int Angle_ax = 0, Angle_ay = 0, Angle_az = 0;	  //由加速度计算的加速度(弧度制)
int Angle_gy = 0, Angle_gx = 0, Angle_gz = 0;	  //由角速度计算的角速率(角度制)
int AngleX = 0, AngleY = 0;				              //四元数解算出的欧拉角 备用AngleZ=0
int16_t aacx = 0, aacy = 0, aacz = 0;           //各轴加速度
int16_t gyrox = 0, gyroy = 0, gyroz = 0;       	//各轴角速度
float IMU_gz;
float Last_Angle_gx=0.0;                          //外环PI输出量  上一次陀螺仪数据
float Last_Angle_gy=0.0;

float invSqrt(float x)   //卡马克快速平方根倒数
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

	norm = invSqrt(ax * ax + ay * ay + az * az) ;//加速度向量的模
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
		//	下面是把四元数换算成方向余弦矩阵中的第三列的三个元素。 
		//	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素
		//	所以这里的vx vy vz，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的
		//	重力单位向量。
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
		
	//用加速度对角速度进行修正，避免积分漂移
	gx = gx + Kp * ex + exInt + Kd * exDif;		
	gy = gy + Kp * ey + eyInt + Kd * eyDif;		
	gz = gz + Kp * ez + ezInt + Kd * ezDif;

	//用角速度更新四元数
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfT;
		
	//四元数规范化，消除浮点舍入误差
	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
		
	//换算欧拉角
	AngleY = atan2(2 * q2 * q3 + 2 * q0 * q1, - 2 * q1 * q1 - 2 * q2 * q2 + 1) * 572.957795f;
	AngleX = asin(2 * (q0 * q2 - q1 * q3 )) * 572.957795f; 
	//AngleZ = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1)* 572.957795f;
}
	
void Angel_Calcu()
{
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据

	Angle_ax = KalmanFilter_ax((float) aacx, KALMAN_Q, KALMAN_R);  //卡尔曼滤波
	Angle_ay = KalmanFilter_ay((float) aacy, KALMAN_Q, KALMAN_R);
	Angle_az = KalmanFilter_az((float) aacz, KALMAN_Q, KALMAN_R);

	Angle_gx = ((float) gyrox) / 65.536;	                   //陀螺仪处理	结果单位是 +-度
	Angle_gy = ((float) gyroy) / 65.536;	                   //陀螺仪量程 +-500度/S, 1度/秒 对应读数 65.536
	Angle_gz = KalmanFilter_gyroz(gyroz, Q15(0.2), Q15(0.8));
	IMU_gz = Angle_gz / 65.536;
	Last_Angle_gx = Angle_gx;		                            //储存上一次角速度数据
	Last_Angle_gy = Angle_gy;
	Gyro_x = gyrox / 131.00;
	Gyro_y = gyroy / 131.00;
	Gyro_z = gyroz / 131.00;
//*********************************** 四元数解算 ***********************************

	IMUupdate(Angle_gx * 0.0174533f, Angle_gy * 0.0174533f, 
					IMU_gz * 0.0174533f, Angle_ax, Angle_ay, Angle_az);//姿态解算，精度0.1度
}	