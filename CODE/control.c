/*
 -- ============================================================================
 -- PROJECT NAME	:		Amelia �ɿ�
 -- FILE_NAME			: 	control.c
 -- DESCRIPTION 	: 	˫��PID����
 -- ----------------------------------------------------------------------------
 -- Revision  Date		  Coding_by	 Comment
 -- 0.0.0	  2017/3/14    Maxwell_W	 Maxwell_W
 --	Copyright:	Amelia ʹ��CC֪ʶ����Э��	֪ʶ��������-��ͬ��ʽ���� 4.0 �������Э��	����
 -- ============================================================================
*/
	
#include	"pwm.h"
#include	"global.h"
#include	"control.h"
#include	"fast_dmp.h"
	
extern	float	angle_x,	angle_y;
				float	angle_erro_x	=	0.0,	angle_erro_y	=	0.0;	//�Ƕ�������
extern	float	gyroGx,	gyroGy,	gyroGz;
				float	gyroGx_erro	=	0.0,	gyroGy_erro	=	0.0;	//���ٶ�������
extern	long temp_CH1,	temp_CH2,	temp_CH3,	temp_CH4;
	
const	float Angle_P_Pitch = 3.8, Angle_P_Roll = 3.8;    //�⻷��������
													//3.8								//3.8
const	float	Angle_I_Pitch	=	0.0,	Angle_I_Roll	=	0.0;		//�⻷���ֲ���
													//										//
													
const float Rate_P_Pitch = 1.4, Rate_P_Roll = 1.4;        //�ڻ���������
													//1.4							//1.4	
const float Rate_I_Pitch = 0.0005, Rate_I_Roll = 0.0023;        //�ڻ����ֲ���
													//								//
const float Rate_D_Pitch =	0.05, Rate_D_Roll = 0.08, Rate_D_Yaw = 3.5;     //�ڻ�΢�ֲ���
													//0.08							//0.08						//3.5
	
float	pitch_makeup	=	0.0,	roll_makeup	=	0.0,	yaw_makeup=	0.0;	//��ƫ����
float	pitch,	roll,	yaw;	//ң�ػ���

float PID_Angle_Pitch_Output, PID_Angle_Roll_Output, PID_Angle_Yaw_Output;  //�⻷�����
float PID_Rate_Pitch_Output, PID_Rate_Roll_Output, PID_Rate_Yaw_Output;     //�ڻ������

int moto1 = 0, moto2 = 0, moto3 = 0, moto4 = 0;
int moto1_last, moto2_last, moto3_last, moto4_last;


//------------------------------------------------------------------------------
//	NAME					:		RC_Calcu
//	DESCRIPTION 	: 	ң�������ݻ���
//------------------------------------------------------------------------------
void	RC_Calcu()
{
	pitch	=	temp_CH2	/	10	-	150;
	roll	=	temp_CH1	/	10	-	150;
	yaw		=	temp_CH4	/	3	-	501;
	//printf("p: %f \r",	pitch);
	//printf("r: %f \n",	roll);
	//printf("y: %f \n",	yaw);
}


//------------------------------------------------------------------------------
//	NAME					:		RC_Calcu_Sport
//	DESCRIPTION 	: 	Sportģʽң�������ݻ���
//------------------------------------------------------------------------------
void	RC_Calcu_Sport()
{
	pitch	=	temp_CH2	/	3	-	500;
	roll	=	temp_CH1	/	3	-	500;
	yaw		=	temp_CH4	/	3	-	501;
	//printf("p: %f \r",	pitch);
	//printf("r: %f \r",	roll);
	//printf("y: %f \n",	yaw);
}


//------------------------------------------------------------------------------
//	NAME					:		PID_Angle
//	DESCRIPTION 	: 	PID�ǶȻ�
//------------------------------------------------------------------------------
void PID_Angle()//PID�Ƕ��⻷
{
	angle_erro_x	+=	angle_x;	//������
	angle_erro_y	+=	angle_y;
	
	if(angle_erro_x	>	20)	angle_erro_x	=	20;
	else	if(angle_erro_x	<	-20)	angle_erro_x	=	-20;	//�����޷�
	
	if(angle_erro_y	>	20)	angle_erro_y	=	20;
	else	if(angle_erro_y	<	-20)	angle_erro_y	=	-20;
	
	//printf("EX	%f	\r",	angle_erro_x);
	//printf("EY	%f	\n",	angle_erro_y);
	
	PID_Angle_Pitch_Output = (-	angle_x - pitch_makeup + pitch) * Angle_P_Pitch		//�ǶȻ�����
												+	angle_erro_x	*	Angle_I_Pitch;
	//printf("%f \n",	PID_Angle_Pitch_Output);
	PID_Angle_Roll_Output = (-	angle_y - roll_makeup + roll) * Angle_P_Roll
												+	angle_erro_y	*	Angle_I_Roll;
	//printf("%f \n",	PID_Angle_Roll_Output);
}


//------------------------------------------------------------------------------
//	NAME					:		PID_Rate
//	DESCRIPTION 	: 	PID���ٶȻ�
//------------------------------------------------------------------------------
void PID_Rate()//PID���ٶ��ڻ�
{
	gyroGx_erro	+=	gyroGx;	//������
	gyroGy_erro	+=	gyroGy;
	
	if(gyroGx_erro	>	90)	gyroGx_erro	=	90;
	else	if(gyroGx_erro	<	-90)	gyroGx_erro	=	-90;	//�����޷�
	
	if(gyroGy_erro	>	90)	gyroGy_erro	=	90;
	else	if(gyroGy_erro	<	-90)	gyroGy_erro	=	-90;
	
	//printf("GX	%f	\r",	gyroGx_erro);
	//printf("GY	%f	\n",	gyroGy_erro);
	
	PID_Rate_Pitch_Output = (PID_Angle_Pitch_Output - gyroGy) * Rate_P_Pitch - gyroGy * Rate_D_Pitch	
												-	gyroGy_erro	*	Rate_I_Pitch;	//���ٶȻ�����
	//printf("%f \n",	PID_Rate_Pitch_Output);
	PID_Rate_Roll_Output = (PID_Angle_Roll_Output + gyroGx) * Rate_P_Roll + gyroGx * Rate_D_Roll
												+	gyroGx_erro	*	Rate_I_Roll;
	//printf("%f \n",	PID_Rate_Roll_Output);
	PID_Rate_Yaw_Output = (-	gyroGz + yaw) * Rate_D_Yaw;
	//printf("%f \n",	PID_Rate_Yaw_Output);
}


//------------------------------------------------------------------------------
//	NAME					:		PID_Sport
//	DESCRIPTION 	: 	�˶�ģʽ
//------------------------------------------------------------------------------
void PID_Sport()//PID���ٶ��ڻ�
{
	gyroGx_erro	+=	gyroGx;	//������
	gyroGy_erro	+=	gyroGy;
	
	if(gyroGx_erro	>	90)	gyroGx_erro	=	90;
	else	if(gyroGx_erro	<	-90)	gyroGx_erro	=	-90;	//�����޷�
	
	if(gyroGy_erro	>	90)	gyroGy_erro	=	90;
	else	if(gyroGy_erro	<	-90)	gyroGy_erro	=	-90;
	
	//printf("GX	%f	\r",	gyroGx_erro);
	//printf("GY	%f	\n",	gyroGy_erro);
	
	PID_Rate_Pitch_Output = (- gyroGy	+	pitch) * Rate_P_Pitch	-	gyroGy_erro	*	Rate_I_Pitch;	//���ٶȻ�����
	//printf("%f \n",	PID_Rate_Pitch_Output);
	PID_Rate_Roll_Output = (gyroGx	+	roll) * Rate_P_Roll	+	gyroGx_erro	*	Rate_I_Roll;
	//printf("%f \n",	PID_Rate_Roll_Output);
	PID_Rate_Yaw_Output = (-	gyroGz + yaw) * Rate_D_Yaw;
	//printf("%f \n",	PID_Rate_Yaw_Output);
}


//------------------------------------------------------------------------------
//	NAME					:		PID_Output
//	DESCRIPTION 	: 	����������
//------------------------------------------------------------------------------
void PID_Output()//������
{
	moto1 = (int)( - PID_Rate_Roll_Output - PID_Rate_Pitch_Output + PID_Rate_Yaw_Output);
	moto2 = (int)( + PID_Rate_Roll_Output + PID_Rate_Pitch_Output + PID_Rate_Yaw_Output);
	moto3 = (int)( + PID_Rate_Roll_Output - PID_Rate_Pitch_Output - PID_Rate_Yaw_Output);
	moto4 = (int)( - PID_Rate_Roll_Output + PID_Rate_Pitch_Output - PID_Rate_Yaw_Output);
	
	if(moto1	<	-225)	moto1	=	-225;
	else	if(moto1	>	225)	moto1	=	225;	//�޷�
	if(moto2	<	-225)	moto2	=	-225;
	else	if(moto2	>	225)	moto2	=	225;
	if(moto3	<	-225)	moto3	=	-225;
	else	if(moto3	>	225)	moto3	=	225;
	if(moto4	<	-225)	moto4	=	-225;
	else	if(moto4	>	225)	moto4	=	225;
	
	//printf("%d \r",	moto1);
	//printf("%d \r",	moto2);
	//printf("%d \r",	moto3);
	//printf("%d \n",	moto4);
		
	moto1_last	= (temp_CH3	*	2.40963855	-	1626.50602)	-	moto1;	//д������ֵ
	moto2_last 	= (temp_CH3	*	2.40963855	-	1626.50602)	-	moto2;     
	moto3_last 	= (temp_CH3	*	2.40963855	-	1626.50602)	-	moto3;
	moto4_last 	=	(temp_CH3	*	2.40963855	-	1626.50602)	-	moto4;
	
	
	if(moto1_last	<	1000)	moto1_last	=	1000;
	else	if(moto1_last	>	3000)	moto1_last	=	3000;	//�޷�
	if(moto2_last	<	1000)	moto2_last	=	1000;
	else	if(moto2_last	>	3000)	moto2_last	=	3000;
	if(moto3_last	<	1000)	moto3_last	=	1000;
	else	if(moto3_last	>	3000)	moto3_last	=	3000;
	if(moto4_last	<	1000)	moto4_last	=	1000;
	else	if(moto4_last	>	3000)	moto4_last	=	3000;
	
	//printf("%d \n",	moto4_last);
	
	if(temp_CH3	>=	1890)	//��������
	{
		//printf("LOW\n");
		Set_Motor_Speed(2900, 2900, 2900, 2900);
	}
	else	//д����
	{
		//printf("M2	%d \r",	moto2_last);
		//printf("M4	%d \r",	moto4_last);
		//printf("M3	%d \r",	moto3_last);
		//printf("M1	%d \n",	moto1_last);
		Set_Motor_Speed(moto1_last, moto2_last, moto3_last, moto4_last);
	}
}

//------------------------------------------------------------------------------
//	NAME					:		ESC_Salibration_Mode
//	DESCRIPTION 	: 	���У׼ģʽ
//------------------------------------------------------------------------------
void	ESC_Salibration_Mode()
{
	moto1_last	=	(temp_CH3	*	2.40963855	-	1626.50602);	//���Ż���
	moto2_last	=	(temp_CH3	*	2.40963855	-	1626.50602);
	moto3_last	=	(temp_CH3	*	2.40963855	-	1626.50602);
	moto4_last	=	(temp_CH3	*	2.40963855	-	1626.50602);
	Set_Motor_Speed(moto1_last, moto2_last, moto3_last, moto4_last);
}

//------------------------------------------------------------------------------
//	NAME					:		MPU_Salibration_Mode
//	DESCRIPTION 	: 	MPUУ׼ģʽ
//------------------------------------------------------------------------------