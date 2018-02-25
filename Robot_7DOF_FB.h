#ifndef ROBOT_7DOF_FB__H
#define ROBOT_7DOF_FB__H


//#include "Matrix.h"
//#include "modbus.h"
#include <opencv2/opencv.hpp>

//using namespace cv;

#include "conio.h"

#define STANLEY_DEBUG 1

#if (STANLEY_DEBUG)
	#define DBGMSG(x)  _cprintf x;
#else
    #define DBGMSG(x)
#endif

//==========
//==MAX AXIS
//==========
#define MAX_AXIS_NUM  7
#define DEF_RIGHT_HAND 1
#define DEF_LEFT_HAND 2


//==========
//==Axis frame
//==========
#define DEF_X 0
#define DEF_Y 1
#define DEF_Z 2

#define DEF_ALPHA 3
#define DEF_BETA 4
#define DEF_GAMMA 5
#define DEF_REDNT_ALPHA 6
#define DEF_ACT_TYPE 7
//===========================
//==Axis index,ID,NO mapping
//==========================
//enum{
//	ID_AXIS1=1,
//	ID_AXIS2,
//	ID_AXIS3,
//	ID_AXIS4,
//	ID_AXIS5,
//	ID_AXIS6,
//	ID_AXIS7
//};

//enum{
//	NO_AXIS1=1,
//	NO_AXIS2,
//	NO_AXIS3,
//	NO_AXIS4,
//	NO_AXIS5,
//	NO_AXIS6,
//	NO_AXIS7
//};
//
//
//static const unsigned char gMapAxisNO[MAX_AXIS_NUM]=
//{
//	NO_AXIS1,
//	NO_AXIS2,
//	NO_AXIS3,
//	NO_AXIS4,
//	NO_AXIS5,
//	NO_AXIS6,
//	NO_AXIS7
//};
//static const unsigned char gMapAxisID[MAX_AXIS_NUM]=
//{
//	ID_AXIS1,
//	ID_AXIS2,
//	ID_AXIS3,
//	ID_AXIS4,
//	ID_AXIS5,
//	ID_AXIS6,
//	ID_AXIS7
//};

enum{
	Index_AXIS1=0,
	Index_AXIS2,
	Index_AXIS3,
	Index_AXIS4,
	Index_AXIS5,
	Index_AXIS6,
	Index_AXIS7,
};

enum{
	ID_RAXIS1=1,
	ID_RAXIS2,
	ID_RAXIS3,
	ID_RAXIS4,
	ID_RAXIS5,
	ID_RAXIS6,
	ID_RAXIS7,
	ID_LAXIS1,
	ID_LAXIS2,
	ID_LAXIS3,
	ID_LAXIS4,
	ID_LAXIS5,
	ID_LAXIS6,
	ID_LAXIS7
};

enum{
	NO_AXIS1=1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};


static const unsigned char gMapAxisNO[MAX_AXIS_NUM]=
{
	NO_AXIS1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};


static const unsigned char gMapRAxisID[MAX_AXIS_NUM]=
{
	ID_RAXIS1,
	ID_RAXIS2,
	ID_RAXIS3,
	ID_RAXIS4,
	ID_RAXIS5,
	ID_RAXIS6,
	ID_RAXIS7
};

static const unsigned char gMapLAxisID[MAX_AXIS_NUM]=
{
	ID_LAXIS1,
	ID_LAXIS2,
	ID_LAXIS3,
	ID_LAXIS4,
	ID_LAXIS5,
	ID_LAXIS6,
	ID_LAXIS7
};

static const unsigned char gMapAllAxisID[MAX_AXIS_NUM*2] =
{
	ID_RAXIS1,
	ID_RAXIS2,
	ID_RAXIS3,
	ID_RAXIS4,
	ID_RAXIS5,
	ID_RAXIS6,
	ID_RAXIS7,
	ID_LAXIS1,
	ID_LAXIS2,
	ID_LAXIS3,
	ID_LAXIS4,
	ID_LAXIS5,
	ID_LAXIS6,
	ID_LAXIS7
};

//================
//==Unit transform
//=================
#define DEF_PI (3.1415926F)
#define DEF_RATIO_PUS_TO_DEG (0.0879F)		//360/4096
#define DEF_RATIO_PUS_TO_RAD (0.0015F)		//2pi/4096   0.00153398078788564122971808758949
#define DEF_RATIO_DEG_TO_PUS (11.3778F)		//4096/360
#define DEF_RATIO_DEG_TO_RAD (DEF_PI/180)		//pi/180	0.01745329251994329576923690768489 (0.0175F)
#define DEF_RATIO_RAD_TO_DEG (180/DEF_PI)	//57.29578
#define DEF_RATIO_RAD_TO_PUS (651.8986F)	//4096/2pi	651.89864690440329530934789477382
//#define DEF_RATIO_VEL_PUS_TO_DEG (0.684F)  //moving speed in register(0~1024) to deg/s 0.114rpm*360/60=0.684deg/s
#define DEF_RATIO_VEL_PUS_TO_DEG_DXL2 (1.374F)  //moving speed in register(0~1024) to deg/s 0.229rpm*360/60=1.374deg/s


//#define DEF_RATIO_VEL_DEG_TO_PUS (1.462F)  
#define DEF_RATIO_VEL_DEG_TO_PUS_DXL2 (0.7278F)  

//#define DEF_RATIO_VEL_RAD_TO_PUS (83.7664F)  //DEF_RATIO_RAD_TO_DEG x DEF_RATIO_VEL_DEG_TO_PUS =57.29578 x  1.462 =83.7664
#define DEF_RATIO_VEL_RAD_TO_PUS_DXL2 (41.6999F)  //DEF_RATIO_RAD_TO_DEG x DEF_RATIO_VEL_DEG_TO_PUS_DXL2 =57.29578 x  0.7278 =41.6999

//#define DEF_RATIO_ACC_PUS_TO_DEG (8.583) //8.583 deg/s^2 / pus
#define DEF_RATIO_ACC_PUS_TO_DEG_DXL2 (21.4577) // 214.577 rev/min^2 / pus =>214.577x360/(60*60)=21.4577 deg/s^2 / pus

//#define DEF_RATIO_ACC_DEG_TO_PUS (0.1165) //0.1165 pus /deg/s^2       range 8.583 deg/s^2 ~ 2180 deg/s^2
#define DEF_RATIO_ACC_DEG_TO_PUS_DXL2 (0.0466)//0.0466 pus /deg/s^2   

//for read pos unit select
enum{
	DEF_UNIT_RAD=1,
	DEF_UNIT_DEG,
	DEF_UNIT_PUS
};

//=====================================
//==robot hard ware dependent parameter
//=====================================
//==robot to Motor offset==//  //robot pos=motor position -M2R_OFFSET
//right
#define AXISR1_R2M_OFFSET_DEG 90
#define AXISR2_R2M_OFFSET_DEG 270
#define AXISR3_R2M_OFFSET_DEG 180
#define AXISR4_R2M_OFFSET_DEG 180
#define AXISR5_R2M_OFFSET_DEG 180
#define AXISR6_R2M_OFFSET_DEG 180
#define AXISR7_R2M_OFFSET_DEG 180
//left
#define AXISL1_R2M_OFFSET_DEG 270
#define AXISL2_R2M_OFFSET_DEG 90
#define AXISL3_R2M_OFFSET_DEG 180
#define AXISL4_R2M_OFFSET_DEG 180
#define AXISL5_R2M_OFFSET_DEG 90
#define AXISL6_R2M_OFFSET_DEG 180
#define AXISL7_R2M_OFFSET_DEG 180

//==robot angle limit==//
//right
#define AXISR1_ROBOT_LIM_DEG_L (-80)
#define AXISR1_ROBOT_LIM_DEG_H 170
#define AXISR2_ROBOT_LIM_DEG_L (-180)
#define AXISR2_ROBOT_LIM_DEG_H 10
#define AXISR3_ROBOT_LIM_DEG_L (-105)
#define AXISR3_ROBOT_LIM_DEG_H 170
#define AXISR4_ROBOT_LIM_DEG_L 0	
#define AXISR4_ROBOT_LIM_DEG_H 170	
#define AXISR5_ROBOT_LIM_DEG_L (-100)
#define AXISR5_ROBOT_LIM_DEG_H 90
#define AXISR6_ROBOT_LIM_DEG_L (-21)
#define AXISR6_ROBOT_LIM_DEG_H 110
#define AXISR7_ROBOT_LIM_DEG_L (-170)
#define AXISR7_ROBOT_LIM_DEG_H 170
//left
#define AXISL1_ROBOT_LIM_DEG_L (-170)
#define AXISL1_ROBOT_LIM_DEG_H 80
#define AXISL2_ROBOT_LIM_DEG_L (-11)
#define AXISL2_ROBOT_LIM_DEG_H 180
#define AXISL3_ROBOT_LIM_DEG_L (-170)
#define AXISL3_ROBOT_LIM_DEG_H 105
#define AXISL4_ROBOT_LIM_DEG_L 0	
#define AXISL4_ROBOT_LIM_DEG_H 170	
#define AXISL5_ROBOT_LIM_DEG_L (-90)
#define AXISL5_ROBOT_LIM_DEG_H 100
#define AXISL6_ROBOT_LIM_DEG_L (-21)
#define AXISL6_ROBOT_LIM_DEG_H 110
#define AXISL7_ROBOT_LIM_DEG_L (-170)
#define AXISL7_ROBOT_LIM_DEG_H 170

//protocol 2.0 has no torque limit
//==robot TORQUE limit==//  0~1023
//right
//#define AXISR1_MAX_TORQUE 928	//90%
//#define AXISR2_MAX_TORQUE 430   //42%	
//#define AXISR3_MAX_TORQUE 923	//90%
//#define AXISR4_MAX_TORQUE 926	//90%
//#define AXISR5_MAX_TORQUE 514	//50%
//#define AXISR6_MAX_TORQUE 519	//50%
//#define AXISR7_MAX_TORQUE 517	//50%
////left
//#define AXISL1_MAX_TORQUE 928	//90%
//#define AXISL2_MAX_TORQUE 430   //42%	
//#define AXISL3_MAX_TORQUE 923	//90%
//#define AXISL4_MAX_TORQUE 926	//90%
//#define AXISL5_MAX_TORQUE 514	//50%
//#define AXISL6_MAX_TORQUE 519	//50%
//#define AXISL7_MAX_TORQUE 517	//50%

//==PID P gain==//
#define AXISR1_POS_P_GAIN	300		
#define AXISR2_POS_P_GAIN	1000
#define AXISR3_POS_P_GAIN	1500
#define AXISR4_POS_P_GAIN	1000
#define AXISR5_POS_P_GAIN	500
#define AXISR6_POS_P_GAIN	1500
#define AXISR7_POS_P_GAIN	60

#define AXISL1_POS_P_GAIN	1500		
#define AXISL2_POS_P_GAIN	1000
#define AXISL3_POS_P_GAIN	3000
#define AXISL4_POS_P_GAIN	3000
#define AXISL5_POS_P_GAIN	1800
#define AXISL6_POS_P_GAIN	100
#define AXISL7_POS_P_GAIN	60

//==PID I gain==//
#define AXISR1_POS_I_GAIN	20		
#define AXISR2_POS_I_GAIN	15
#define AXISR3_POS_I_GAIN	15
#define AXISR4_POS_I_GAIN	60
#define AXISR5_POS_I_GAIN	40
#define AXISR6_POS_I_GAIN	40
#define AXISR7_POS_I_GAIN	15

#define AXISL1_POS_I_GAIN	30	
#define AXISL2_POS_I_GAIN	15
#define AXISL3_POS_I_GAIN	40
#define AXISL4_POS_I_GAIN	60
#define AXISL5_POS_I_GAIN	15
#define AXISL6_POS_I_GAIN	15
#define AXISL7_POS_I_GAIN	15

//==PID D gain==//
#define AXISR1_POS_D_GAIN	30		
#define AXISR2_POS_D_GAIN	10
#define AXISR3_POS_D_GAIN	10
#define AXISR4_POS_D_GAIN	200
#define AXISR5_POS_D_GAIN	10
#define AXISR6_POS_D_GAIN	10
#define AXISR7_POS_D_GAIN	10

#define AXISL1_POS_D_GAIN	10		
#define AXISL2_POS_D_GAIN	10
#define AXISL3_POS_D_GAIN	10
#define AXISL4_POS_D_GAIN	200
#define AXISL5_POS_D_GAIN	10
#define AXISL6_POS_D_GAIN	10
#define AXISL7_POS_D_GAIN	10

//==define right hand or left hand==//
#define DEF_RIGHT_HAND	1
#define DEF_LEFT_HAND	2

static const unsigned short int gr2m_offset_pulse_R[MAX_AXIS_NUM]=
{
	(unsigned short int)(AXISR1_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR2_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR3_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR4_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR5_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR6_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISR7_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS)
};

static const unsigned short int gr2m_offset_pulse_L[MAX_AXIS_NUM]=
{
	(unsigned short int)(AXISL1_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL2_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL3_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL4_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL5_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL6_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXISL7_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS)
};

//right hand
static const float grobot_lim_rad_R_Low[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISR7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_rad_R_High[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISR7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_pus_R_Low[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISR7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS
};

static const float grobot_lim_pus_R_High[MAX_AXIS_NUM]=
{
	AXISR1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISR7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS
};

//left hand
static const float grobot_lim_rad_L_Low[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXISL7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_rad_L_High[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXISL7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_pus_L_Low[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXISL7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS
};

static const float grobot_lim_pus_L_High[MAX_AXIS_NUM]=
{
	AXISL1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXISL7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS
};


//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0
//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0

//===================
//==ROBOT link length
//====================
#define L0 248    	//head to shoulder
#define L1 250    	//L-type linker long side
#define L2 25   	//L-type linker short side
#define L3 25     	//L-type linker short side
#define L4 230     	//L-type linker long side
#define L5 195     	//length from wrist to end effector
#define X_BASE 0  	//基準點只能都先設0
#define Y_BASE 0
#define Z_BASE 0

//===================
//==IK計算使用
//====================
#define DEF_NORM_VERY_SMALL (1.e-3)//norm很小的量判斷為0使用
#define DEF_COSVAL_VERY_SMALL (1.e-7)//cos值很小的量判斷為0使用

//=========================
//====Line move parameter==
//=========================
#define DEF_ROBOT_COOR 1//coordinate of robot 
#define DEF_OBJFRAME_COOR 2//coordinate of object frame


//=====================================
//自己建立的陣列class，為了要陣列相加減
//=====================================
class CStaArray 
{
public:
	int m_ARR_SIZE;
	double m_arr[7];
	
	CStaArray();
	CStaArray(double x, double y, double z, double alpha, double beta, double gamma, double rednt_alpha);

	double at(int index);
	void SetArray(double x,double y,double z,double alpha,double beta,double gamma,double rednt_alpha);
	void arr_round();

	CStaArray operator*(double k);
	CStaArray operator*(CStaArray &otherk);
	CStaArray operator/(double k);
	CStaArray operator+(CStaArray &other);
	CStaArray operator-(CStaArray &other);

};


//==========
//Function
//==========
unsigned char getMapAxisNO(unsigned char index); //index 0~ (MAX_AXIS_NUM-1)
unsigned char getMapRAxisID(unsigned char index);
unsigned char getMapLAxisID(unsigned char index);
int ROM_Setting_Dual();
void PID_Setting_Dual();
int Read_pos(int RLHand,double *pos,unsigned char unit);
int Read_moving(int RLHand, unsigned char *mov);
int Read_vel(int RLHand, double *vel);
void TestRead_pos();

int ReadPresentLoad(int RLHand,double *LoadPercent);
void WaitMotionDoneDual();
void TestSewingAction();
void PickSewingObject();
void MoveToInitailPoint(int Coordinate,CStaArray &R_starP,CStaArray &L_starP);
int TestMoveToSewingHome_Dual();
int Torque_Switch(int sw);
int SetAllAccTo(double deg_s2);
void LineMoveTo(int Coordinate,CStaArray &L_starP,CStaArray &L_endP,CStaArray &R_starP,CStaArray &R_endP,double CostTime);
void RotateMoveTo(int Coordinate, CStaArray &L_starP,CStaArray &L_endP,CStaArray &R_starP,CStaArray &R_endP,CStaArray &arc_cen,double rot_rad, double CostTime);
void IKOutputToArm(CStaArray &PathPlanPoint_R,CStaArray &PathPlanPoint_L);
void TestMotorPID();
void RecordMotor();

int Output_to_Dynamixel(int RLHand,const double *Ang_rad,const unsigned short int *velocity) ;
int Output_to_Dynamixel_Dual(const double *Ang_rad_R,const unsigned short int *velocity_R, const unsigned short int *acc_R,const double *Ang_rad_L,const unsigned short int *velocity_L, const unsigned short int *acc_L);
int Output_to_Dynamixel_pulse(const unsigned short int *Ang_pulse,const unsigned short int *velocity);

//Matrix R_z1x2y3(double alpha,double beta,double gamma);
cv::Mat R_z1x2y3(double alpha,double beta,double gamma);		
//float norm(const Matrix& v);
//Matrix Rogridues(float theta,const Matrix& V_A);
cv::Mat Rogridues(double theta, cv::Mat V_A);
//int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta);
int IK_7DOF_FB7roll(int RLHand,const double linkL[6],const double base[3],const double Pend[3],const double PoseAngle[3],const double Rednt_alpha, double* out_theta);
bool AngleOverConstrain(int RLHand,const double theta[MAX_AXIS_NUM],int *OverIndex);
int MoveToPoint(int RLHand,double Point[7], double vel_deg); //Point[x,y,z,alpha,beta,gamma,redant_alpha]
int MoveToPoint_Dual(double current_theta_deg_R[7], double Point_R[7], double current_theta_deg_L[7], double Point_L[7]);
int IsMoving(int RLHand,bool *stillmoving);
void QPDelay_ms(int t_ms);

//dynamixel use
//int syncWrite_x86(unsigned short int start_addr, unsigned short int data_length, unsigned short int *param, unsigned short int param_length); // WORD(16bit) syncwrite() for DXL
//int setPosition_x86(int ServoID, int Position, int Speed);
int DXL_Initial();
int DXL_Terminate();


//Modbus control gripper
#ifdef MODBUS_GRIPPER
int Initial_Modbus();
void Terminate_Modbus();
int GripperHold(int RLHand,bool Hold);
#endif

//int Gripper_LattePanda_Initial();
//void Gripper_LattePanda_Close();
//int Gripper_LattePanda_Hold(int RLHand,bool Hold,int delay_ms);
#endif    /* ROBOT_7DOF_FB__H */


//F446RE IO
//#define DEF_CLOCK_WISE          true
//#define DEF_COUNTER_CLOCK_WISE  false
//
//int F446RE_Initial();
//void F446RE_Close();
//void F446RE_RotateMotor(bool dir,int deg);
//void F446RE_Gripper_Hold(int RLHand,bool Hold,int delay_ms);
//void F446RE_FootLifter(bool sw);
//void F446RE_Spindle(bool sw);
//void F446RE_Trimmer(bool sw);


//=================================
//==c++ rs232 communicate to f446re
//==================================
#include <Windows.h>
#include <string>
class cF446RE
{
public:
	HANDLE _hserialPort;
	bool _continue;
	bool _readecho;

	//motor
	static const byte DEF_IX_PARA_DIR = 0;
	static const byte DEF_IX_PARA_DEG_HIGH = 1;
	static const byte DEF_IX_PARA_DEG_LOW = 2;

	static const byte DEF_ECHO_ROTATE_MOTOR_DONE = 3;

	static const bool DEF_CLOCK_WISE = true;
	static const bool DEF_COUNTER_CLOCK_WISE = false;

	//gripper
	//static const byte DEF_RIGHT_HAND = 1;
	//static const byte DEF_LEFT_HAND = 2;

	//====================
	//communication struct
	//====================
	static const byte DEF_CMD_ROTATE_MOTOR = 0xa1;
	static const byte DEF_CMD_GRIP = 0xa2;
	static const byte DEF_CMD_FL = 0xa3;
	static const byte DEF_CMD_SPINDLE = 0xa4;
	static const byte DEF_CMD_TRIMMER = 0xa5;
	
	cF446RE();
	bool initial(int com, int baudrate);
	void close();
	HANDLE RSLINK(unsigned long Port, unsigned long BRate);
	DWORD ReadComm(HANDLE hRS, LPVOID lpInBuffer, DWORD dwBytesToRead);
	BOOL WriteComm(HANDLE hRS, LPCVOID lpSndBuffer, DWORD dwBytesToWrite);
	bool RotateMotor(bool dir, int deg);
	void Gripper_Hold(int RLHand, bool Hold, int Delay_ms);
	void FootLifter(bool sw);
	void Spindle(bool sw);
	void Trimmer(bool sw);
};

//========
//==CV_Test
//========
void testcv();
void GetObjCornerCoorFromImage();
void VisionOn();

//dxl2.0 test
int dxl2test();
int dxl2_sync_test();