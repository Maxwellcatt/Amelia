/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	fast_dmp.c
 -- DESCRIPTION 	: 	�����˲���̬����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/7/12    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/

#include 	"sys.h"
#include	"math.h"
#include 	"delay.h"
#include 	"usart.h"
#include	"mpu60xx.h"
#include	"fast_dmp.h"

float	angle_x,	angle_y;	//�������
float	angleAx,	angleAy;	//���ٶȼ�������
float	gyroGx,	gyroGy,	gyroGz;	//���ٶ�
short	ax,	ay,	az;	//���ٶ�ԭʼֵ
short	gx,	gy,	gz;	//���ٶ�ԭʼֵ

float	x[10]	=	0,	temp_x;	//FIR����
float	y[10]	=	0,	temp_y;	//FIR����

//------------------------------------------------------------------------------
//	NAME					:		Fst_Comple_Filter_X
//	DESCRIPTION 	: 	X��һ�׻����˲�
//------------------------------------------------------------------------------
float	Fst_Comple_Filter_X(float	angle_m,	float	gyro_m)
{
	float	angle;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + gyro_m * DT);
	
	return	angle;
}

//------------------------------------------------------------------------------
//	NAME					:		Fst_Comple_Filter_Y
//	DESCRIPTION 	: 	Y��һ�׻����˲�
//------------------------------------------------------------------------------
float	Fst_Comple_Filter_Y(float	angle_m,	float	gyro_m)
{
	float	angle;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + gyro_m * DT);
	
	return	angle;
}


//------------------------------------------------------------------------------
//	NAME					:		Sec_Comple_Filter
//	DESCRIPTION 	: 	X����׻����˲�
//------------------------------------------------------------------------------

float	Sec_Comple_Filter(float	angle_m,	float	gyro_m)
{
	float	angle	=	0;
	
	angle = (1.0	-	K1) * angle_m	+ K1 * (angle + (gyro_m	+	(angle_m	-	angle)	*	K2)	* DT);
	
	return	angle;
}

//------------------------------------------------------------------------------
//	NAME					:		FIR_Filter_X
//	DESCRIPTION 	: 	X��FIR��ͨ�˲���9�׽�ֹƵ��80Hz
//------------------------------------------------------------------------------

float	FIR_Filter_X(float input)
{
	float	output;
	
	temp_x	=	x[0];
	
	x[0]	=	input;	//��������
	x[9]	=	x[8];
	x[8]	=	x[7];
	x[7]	=	x[6];
	x[6]	=	x[5];
	x[5]	=	x[4];
	x[4]	=	x[3];
	x[3]	=	x[2];
	x[2]	=	x[1];
	x[1]	=	temp_x;
	
	output	=	x[0]	*	0.006749454325693	+	x[1]	*	0.025063820521276	+	x[2]	*	0.081909996095874	//���
					+	x[3]	*	0.162947247253628	+	x[4]	*	0.223329481803529	+	x[5]	*	0.223329481803529
					+	x[6]	*	0.162947247253628	+	x[7]	*	0.081909996095874	+	x[8]	*	0.025063820521276	
					+	x[9]	*	0.006749454325693;
	
	return	output;
}

//------------------------------------------------------------------------------
//	NAME					:		FIR_Filter_Y
//	DESCRIPTION 	: 	Y��FIR��ͨ�˲���9�׽�ֹƵ��80Hz
//------------------------------------------------------------------------------

float	FIR_Filter_Y(float input)
{
	float	output;
	
	temp_y	=	y[0];
	
	y[0]	=	input;	//��������
	y[9]	=	y[8];
	y[8]	=	y[7];
	y[7]	=	y[6];
	y[6]	=	y[5];
	y[5]	=	y[4];
	y[4]	=	y[3];
	y[3]	=	y[2];
	y[2]	=	y[1];
	y[1]	=	temp_y;
	
	output	=	y[0]	*	0.006749454325693	+	y[1]	*	0.025063820521276	+	y[2]	*	0.081909996095874	//���
					+	y[3]	*	0.162947247253628	+	y[4]	*	0.223329481803529	+	y[5]	*	0.223329481803529
					+	y[6]	*	0.162947247253628	+	y[7]	*	0.081909996095874	+	y[8]	*	0.025063820521276	
					+	y[9]	*	0.006749454325693;
	
	return	output;
}


//------------------------------------------------------------------------------
//	NAME					:		Get_Angle
//	DESCRIPTION 	: 	�������
//------------------------------------------------------------------------------
void	Get_Angle(void) 
{
	MPU_Get_Gyroscope(&gx,	&gy,	&gz);	//��ȡ���ٶ�
	MPU_Get_Accelerometer(&ax,	&ay,	&az);	//��ȡ���ٶ�
	//printf("%f \n",	(float)(az));
  angleAx	=	atan2(ax,	az)	*	57.29577951;//������x��н�
	angleAy	=	atan2(ay,	az)	*	57.29577951;//������y��н�
	//printf("%f \n",	angleAx);
	gyroGx	=	-gx	/	131.00;//������ٶ�
  gyroGy	=	-gy	/	131.00;//������ٶ�
	gyroGz	=	-gz	/	131.00;//������ٶ�
	//printf("gx: %f \r",	gyroGx);
	//printf("gy: %f \r",	gyroGy);
	//printf("gz: %f \n",	gyroGz);
	
  angle_x	=	Fst_Comple_Filter_X(angleAx,	gyroGy);//X���˲�
	angle_y	=	Fst_Comple_Filter_Y(angleAy,	gyroGx);//X���˲�
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
//angle += K_0 * angle_err; //���ŽǶ�
//q_bias += K_1 * angle_err;
//angle_dot = gyro_m-q_bias;//���Ž��ٶ�
//}
