/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia 飞控
 -- FILE_NAME			: 	fast_dmp.c
 -- DESCRIPTION 	: 	互补滤波姿态解算
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/7/12    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia 使用CC知识共享协议	知识共享署名-相同方式共享 4.0 国际许可协议	发布
 -- ============================================================================
*/

#include 	"sys.h"
#include	"math.h"
#include 	"delay.h"
#include 	"usart.h"
#include	"mpu60xx.h"
#include	"fast_dmp.h"

float	angle_x,	angle_y;	//最终倾角
float	angleAx,	angleAy;	//加速度计算的倾角
float	gyroGx,	gyroGy,	gyroGz;	//角速度
short	ax,	ay,	az;	//加速度原始值
short	gx,	gy,	gz;	//加速度原始值

float	x[10]	=	0,	temp_x;	//FIR序列
float	y[10]	=	0,	temp_y;	//FIR序列

//------------------------------------------------------------------------------
//	NAME					:		Fst_Comple_Filter_X
//	DESCRIPTION 	: 	X轴一阶互补滤波
//------------------------------------------------------------------------------
float	Fst_Comple_Filter_X(float	angle_m,	float	gyro_m)
{
	float	angle;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + gyro_m * DT);
	
	return	angle;
}

//------------------------------------------------------------------------------
//	NAME					:		Fst_Comple_Filter_Y
//	DESCRIPTION 	: 	Y轴一阶互补滤波
//------------------------------------------------------------------------------
float	Fst_Comple_Filter_Y(float	angle_m,	float	gyro_m)
{
	float	angle;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + gyro_m * DT);
	
	return	angle;
}


//------------------------------------------------------------------------------
//	NAME					:		Sec_Comple_Filter
//	DESCRIPTION 	: 	X轴二阶互补滤波
//------------------------------------------------------------------------------

float	Sec_Comple_Filter(float	angle_m,	float	gyro_m)
{
	float	angle	=	0;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + (gyro_m	+	(angle_m	-	angle)	*	K2)	* DT);
	
	return	angle;
}

//------------------------------------------------------------------------------
//	NAME					:		FIR_Filter_X
//	DESCRIPTION 	: 	X轴FIR低通滤波，9阶截止频率80Hz
//------------------------------------------------------------------------------

float	FIR_Filter_X(float input)
{
	float	output;
	
	temp_x	=	x[0];
	
	x[0]	=	input;	//递推序列
	x[9]	=	x[8];
	x[8]	=	x[7];
	x[7]	=	x[6];
	x[6]	=	x[5];
	x[5]	=	x[4];
	x[4]	=	x[3];
	x[3]	=	x[2];
	x[2]	=	x[1];
	x[1]	=	temp_x;
	
	output	=	x[0]	*	0.006749454325693	+	x[1]	*	0.025063820521276	+	x[2]	*	0.081909996095874	//卷积
					+	x[3]	*	0.162947247253628	+	x[4]	*	0.223329481803529	+	x[5]	*	0.223329481803529
					+	x[6]	*	0.162947247253628	+	x[7]	*	0.081909996095874	+	x[8]	*	0.025063820521276	
					+	x[9]	*	0.006749454325693;
	
	return	output;
}

//------------------------------------------------------------------------------
//	NAME					:		FIR_Filter_Y
//	DESCRIPTION 	: 	Y轴FIR低通滤波，9阶截止频率80Hz
//------------------------------------------------------------------------------

float	FIR_Filter_Y(float input)
{
	float	output;
	
	temp_y	=	y[0];
	
	y[0]	=	input;	//递推序列
	y[9]	=	y[8];
	y[8]	=	y[7];
	y[7]	=	y[6];
	y[6]	=	y[5];
	y[5]	=	y[4];
	y[4]	=	y[3];
	y[3]	=	y[2];
	y[2]	=	y[1];
	y[1]	=	temp_y;
	
	output	=	y[0]	*	0.006749454325693	+	y[1]	*	0.025063820521276	+	y[2]	*	0.081909996095874	//卷积
					+	y[3]	*	0.162947247253628	+	y[4]	*	0.223329481803529	+	y[5]	*	0.223329481803529
					+	y[6]	*	0.162947247253628	+	y[7]	*	0.081909996095874	+	y[8]	*	0.025063820521276	
					+	y[9]	*	0.006749454325693;
	
	return	output;
}


//------------------------------------------------------------------------------
//	NAME					:		Get_Angle
//	DESCRIPTION 	: 	计算倾角
//------------------------------------------------------------------------------
void	Get_Angle(void) 
{
	MPU_Get_Gyroscope(&gx,	&gy,	&gz);	//获取角速度
	MPU_Get_Accelerometer(&ax,	&ay,	&az);	//获取加速度
	//printf("%f \n",	(float)(az));
  angleAx	=	atan2(ax,	az)	*	57.29577951;//计算与x轴夹角
	angleAy	=	atan2(ay,	az)	*	57.29577951;//计算与y轴夹角
	//printf("%f \n",	angleAx);
	gyroGx	=	-gx	/	131.00;//计算角速度
  gyroGy	=	-gy	/	131.00;//计算角速度
	gyroGz	=	-gz	/	131.00;//计算角速度
	//printf("gx: %f \r",	gyroGx);
	//printf("gy: %f \r",	gyroGy);
	//printf("gz: %f \n",	gyroGz);
	
  angle_x	=	Fst_Comple_Filter_X(angleAx,	gyroGy);//X阶滤波
	angle_y	=	Fst_Comple_Filter_Y(angleAy,	gyroGx);//X阶滤波
	angle_x	=	FIR_Filter_X(angle_x);
	angle_y	=	FIR_Filter_Y(angle_y);
	//printf("%f \n",	(float)(az));
	//printf("X: %f \r",	angle_x);
	//printf("Y: %f \n",	angle_y);
}



//void Kalman_Filter(double angle_m,double gyro_m)
//{
//angle+=(gyro_m-q_bias) * dt;
//angle_err = angle_m - angle;
//Pdot[0]=Q_angle - P[0][1] - P[1][0];
//Pdot[1]=- P[1][1];
//Pdot[2]=- P[1][1];
//Pdot[3]=Q_gyro;
//P[0][0] += Pdot[0] * dt;
//P[0][1] += Pdot[1] * dt;
//P[1][0] += Pdot[2] * dt;
//P[1][1] += Pdot[3] * dt;
//PCt_0 = C_0 * P[0][0];
//PCt_1 = C_0 * P[1][0];
//E = R_angle + C_0 * PCt_0;
//K_0 = PCt_0 / E;
//K_1 = PCt_1 / E;
//t_0 = PCt_0;
//t_1 = C_0 * P[0][1];
//P[0][0] -= K_0 * t_0;
//P[0][1] -= K_0 * t_1;
//P[1][0] -= K_1 * t_0;
//P[1][1] -= K_1 * t_1;
//angle += K_0 * angle_err; //最优角度
//q_bias += K_1 * angle_err;
//angle_dot = gyro_m-q_bias;//最优角速度
//}
