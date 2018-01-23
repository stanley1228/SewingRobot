

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  //高層GUI
//#include <opencv2/legacy/legacy.hpp>


#include "Robot_7DOF_FB.h"

#include <vector>
//#include "Matrix.h"

//#include "MatrixMath.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>

#include "dynamixel2.h"
#include <windows.h> //QPDelay_ms使用
#include<fstream>//寫檔使用

//#pragma comment(lib,"dynamixel.lib") 
#pragma comment(lib,"dynamixel2_win64.lib") 
#pragma comment(lib,"opencv_world340d.lib")
//#pragma comment(lib,"opencv_core2413d.lib")	//顯示圖片使用
//#pragma comment(lib,"opencv_highgui2413d.lib") 
//#pragma comment(lib,"opencv_imgproc2413d.lib")
//#pragma comment(lib,"opencv_video2413d.lib")
//#pragma comment(lib,"opencv_legacy2413d.lib")
//#pragma comment(lib,"opencv_objdetect2413d.lib")

//#define F446RE_GRIPPER_EN
//#define CHECK_CARTESIAN_PATH 
//#define GRIPPER_ON_LATTE
#define MOVETOPOINT_DUAL
#define CHECK_JOINT_PATH  
//#define CHECK_JOINT_VEL_CMD
//#define MOVE_TO_INITIAL_POINT
//#define RECORD_JOINT_ANGLE
//#define RECORD_JOINT_VEL
//#define RECORD_JOINT_LOAD
//#define RECORD_JOINT_MOVING
//#define DEF_WAIT_ENTER

const double gCycleT=0.5;

std::fstream gfileR;
std::fstream gfileL;
std::fstream gfileJointR;
std::fstream gfileJointL;
std::fstream gfileJointVel_R;
std::fstream gfileJointVel_L;
std::fstream gfileJointLoad_R;
std::fstream gfileJointLoad_L;
std::fstream gfileJointMoving_R;
std::fstream gfileJointMoving_L;
std::fstream gfileJointVelCmd_R;
std::fstream gfileJointVelCmd_L;

static double gstatic_abst = 0;
cF446RE* gpF446RE = NULL;

using namespace cv;
using namespace std;


//#if (DEBUG)
//	#define DBGMSG(x)  printf x;
//#else
//    #define DBGMSG(x)
//#endif

unsigned char getMapAxisNO(unsigned char index)
{
	return gMapAxisNO[index];

}

unsigned char getMapRAxisID(unsigned char index)
{
	return gMapRAxisID[index];
}

unsigned char getMapLAxisID(unsigned char index)
{
	return gMapLAxisID[index];
}


int ROM_Setting_Dual()
{
	//protocol 2.0 has no torque limit
	//==calculate Max torque to set in rom 
	//const short int Max_torque_R[MAX_AXIS_NUM]=
	//{
	//	AXISR1_MAX_TORQUE,
	//	AXISR2_MAX_TORQUE,
	//	AXISR3_MAX_TORQUE,
	//	AXISR4_MAX_TORQUE,
	//	AXISR5_MAX_TORQUE,
	//	AXISR6_MAX_TORQUE,
	//	AXISR7_MAX_TORQUE
	//};

	//const short int Max_torque_L[MAX_AXIS_NUM]=
	//{
	//	AXISL1_MAX_TORQUE,
	//	AXISL2_MAX_TORQUE,
	//	AXISL3_MAX_TORQUE,
	//	AXISL4_MAX_TORQUE,
	//	AXISL5_MAX_TORQUE,
	//	AXISL6_MAX_TORQUE,
	//	AXISL7_MAX_TORQUE
	//};



	//==Calculate angle limit==//
	const short int R2M_OFFSET_DEG_R[MAX_AXIS_NUM]=
	{
		AXISR1_R2M_OFFSET_DEG,
		AXISR2_R2M_OFFSET_DEG,
		AXISR3_R2M_OFFSET_DEG,
		AXISR4_R2M_OFFSET_DEG,
		AXISR5_R2M_OFFSET_DEG,
		AXISR6_R2M_OFFSET_DEG,
		AXISR7_R2M_OFFSET_DEG
	};

	const short int R2M_OFFSET_DEG_L[MAX_AXIS_NUM]=
	{
		AXISL1_R2M_OFFSET_DEG,
		AXISL2_R2M_OFFSET_DEG,
		AXISL3_R2M_OFFSET_DEG,
		AXISL4_R2M_OFFSET_DEG,
		AXISL5_R2M_OFFSET_DEG,
		AXISL6_R2M_OFFSET_DEG,
		AXISL7_R2M_OFFSET_DEG
	};
	
	//right hand
	const short int ROBOT_LIM_DEG_R_LOW[MAX_AXIS_NUM]=
	{
		AXISR1_ROBOT_LIM_DEG_L,
		AXISR2_ROBOT_LIM_DEG_L,
		AXISR3_ROBOT_LIM_DEG_L,
		AXISR4_ROBOT_LIM_DEG_L,
		AXISR5_ROBOT_LIM_DEG_L,
		AXISR6_ROBOT_LIM_DEG_L,
		AXISR7_ROBOT_LIM_DEG_L
	};

	const short int ROBOT_LIM_DEG_R_HIGH[MAX_AXIS_NUM]=
	{
		AXISR1_ROBOT_LIM_DEG_H,
		AXISR2_ROBOT_LIM_DEG_H,
		AXISR3_ROBOT_LIM_DEG_H,
		AXISR4_ROBOT_LIM_DEG_H,
		AXISR5_ROBOT_LIM_DEG_H,
		AXISR6_ROBOT_LIM_DEG_H,
		AXISR7_ROBOT_LIM_DEG_H
	};

	//left hand
	const short int ROBOT_LIM_DEG_L_LOW[MAX_AXIS_NUM]= 
	{
		AXISL1_ROBOT_LIM_DEG_L,
		AXISL2_ROBOT_LIM_DEG_L,
		AXISL3_ROBOT_LIM_DEG_L,
		AXISL4_ROBOT_LIM_DEG_L,
		AXISL5_ROBOT_LIM_DEG_L,
		AXISL6_ROBOT_LIM_DEG_L,
		AXISL7_ROBOT_LIM_DEG_L
	};

	const short int ROBOT_LIM_DEG_L_HIGH[MAX_AXIS_NUM]= 
	{
		AXISL1_ROBOT_LIM_DEG_H,
		AXISL2_ROBOT_LIM_DEG_H,
		AXISL3_ROBOT_LIM_DEG_H,
		AXISL4_ROBOT_LIM_DEG_H,
		AXISL5_ROBOT_LIM_DEG_H,
		AXISL6_ROBOT_LIM_DEG_H,
		AXISL7_ROBOT_LIM_DEG_H
	};



	unsigned long Motor_lim_pulse_R_high[MAX_AXIS_NUM]={0};
	unsigned long Motor_lim_pulse_R_low[MAX_AXIS_NUM]={0};
	unsigned long Motor_lim_pulse_L_high[MAX_AXIS_NUM]={0};
	unsigned long Motor_lim_pulse_L_low[MAX_AXIS_NUM]={0};
	

	int i=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//right hand
		Motor_lim_pulse_R_low[i]=(unsigned long)((ROBOT_LIM_DEG_R_LOW[i]+R2M_OFFSET_DEG_R[i])*DEF_RATIO_DEG_TO_PUS);
		Motor_lim_pulse_R_high[i]=(unsigned long)((ROBOT_LIM_DEG_R_HIGH[i]+R2M_OFFSET_DEG_R[i])*DEF_RATIO_DEG_TO_PUS);

		//left hand
		Motor_lim_pulse_L_low[i]=(unsigned long)((ROBOT_LIM_DEG_L_LOW[i]+R2M_OFFSET_DEG_L[i])*DEF_RATIO_DEG_TO_PUS);
		Motor_lim_pulse_L_high[i]=(unsigned long)((ROBOT_LIM_DEG_L_HIGH[i]+R2M_OFFSET_DEG_L[i])*DEF_RATIO_DEG_TO_PUS);
	}

	//==writing to ROM==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//==Set MAX_torgue==//
		//dxl_write_word(gMapRAxisID[i],MAX_TORQUE,Max_torque_R[i]);//right
		//dxl_write_word(gMapLAxisID[i],MAX_TORQUE,Max_torque_L[i]);//left
	
		//==Set angel limit==//
		dxl2_write_dword(gMapRAxisID[i], MIN_POS_LIMIT,Motor_lim_pulse_R_low[i]);//right
		dxl2_write_dword(gMapRAxisID[i], MAX_POS_LIMIT,Motor_lim_pulse_R_high[i]);//right  

		dxl2_write_dword(gMapLAxisID[i], MIN_POS_LIMIT,Motor_lim_pulse_L_low[i]);//left
		dxl2_write_dword(gMapLAxisID[i], MAX_POS_LIMIT,Motor_lim_pulse_L_high[i]);//left  

		//==Set MULTITURN_OFFSET to 0==//
		//dxl_write_word(gMapRAxisID[i],MULTITURN_OFFSET,0);//right
		//dxl_write_word(gMapLAxisID[i],MULTITURN_OFFSET,0);//left
	}
	

	//==read and check right hand==//
	int	txrx_result=0;
	//short int max_torque=0;
	unsigned long min_pos_lim=0,max_pos_lim=0;
	//short int multi_turn_offset=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		_cprintf("===AXIS_R%d===\n",gMapAxisNO[i]);

		//==MAX_torgue==//
		//max_torque = dxl_read_word(gMapRAxisID[i], MAX_TORQUE);
		//txrx_result = dxl_get_comm_result();
		//if(txrx_result!=COMM_RXSUCCESS)
		//	printf("Failed read MAX_TORQUE error=%d\n",txrx_result);
		//else
		//	printf("MAX_TORQUE=%d\n",max_torque);
	
		//==MIN_POS_LIMIT,MAX_POS_LIMIT==//
		min_pos_lim =dxl2_read_dword(gMapRAxisID[i], MIN_POS_LIMIT);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read MIN_POS_LIMIT error=%d\n",txrx_result);
		else	
			_cprintf("MIN_POS_LIMIT=%d,degree=%f\n", min_pos_lim, min_pos_lim*DEF_RATIO_PUS_TO_DEG);

		max_pos_lim = dxl2_read_dword(gMapRAxisID[i], MAX_POS_LIMIT);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed Read MAX_POS_LIMIT failed error=%d\n",txrx_result);
		else	
			_cprintf("MAX_POS_LIMIT=%d,degree=%f\n", max_pos_lim, max_pos_lim*DEF_RATIO_PUS_TO_DEG);
		

		//==multi turn offset==//
		//multi_turn_offset=dxl_read_word(gMapRAxisID[i],MULTITURN_OFFSET);
		//txrx_result = dxl_get_comm_result();
		//if(txrx_result!=COMM_RXSUCCESS)
		//	printf("Failed Read MULTITURN_OFFSET failed error=%d\n",txrx_result);
		//else	
		//	printf("MULTITURN_OFFSET=%d\n",multi_turn_offset);
	}

	//==read and check left hand==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		_cprintf("===AXIS_L%d===\n",gMapAxisNO[i]);

		//==MAX_torgue==//
		//max_torque = dxl_read_word(gMapLAxisID[i], MAX_TORQUE);
		//txrx_result = dxl_get_comm_result();
		//if(txrx_result!=COMM_RXSUCCESS)
		//	printf("Failed read MAX_TORQUE error=%d\n",txrx_result);
		//else
		//	printf("MAX_TORQUE=%d\n",max_torque);
	
		//==MIN_POS_LIMIT,MAX_POS_LIMIT==//
		min_pos_lim = dxl2_read_dword(gMapLAxisID[i], MIN_POS_LIMIT);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read MIN_POS_LIMIT error=%d\n",txrx_result);
		else	
			_cprintf("MIN_POS_LIMIT=%d,degree=%f\n", min_pos_lim, min_pos_lim*DEF_RATIO_PUS_TO_DEG);

		max_pos_lim = dxl2_read_dword(gMapLAxisID[i], MAX_POS_LIMIT);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed Read MAX_POS_LIMIT failed error=%d\n",txrx_result);
		else	
			_cprintf("MAX_POS_LIMIT=%d,degree=%f\n", max_pos_lim, max_pos_lim*DEF_RATIO_PUS_TO_DEG);
		

		//==multi turn offset==//
		//multi_turn_offset=dxl_read_word(gMapLAxisID[i],MULTITURN_OFFSET);
		//txrx_result = dxl_get_comm_result();
		//if(txrx_result!=COMM_RXSUCCESS)
		//	printf("Failed Read MULTITURN_OFFSET failed error=%d\n",txrx_result);
		//else	
		//	printf("MULTITURN_OFFSET=%d\n",multi_turn_offset);
	}

	return 0;
}

void PID_Setting_Dual()
{
	const short int POS_P_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_POS_P_GAIN,
		AXISR2_POS_P_GAIN,
		AXISR3_POS_P_GAIN,
		AXISR4_POS_P_GAIN,
		AXISR5_POS_P_GAIN,
		AXISR6_POS_P_GAIN,
		AXISR7_POS_P_GAIN
	};

	const short int POS_P_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_POS_P_GAIN,
		AXISL2_POS_P_GAIN,
		AXISL3_POS_P_GAIN,
		AXISL4_POS_P_GAIN,
		AXISL5_POS_P_GAIN,
		AXISL6_POS_P_GAIN,
		AXISL7_POS_P_GAIN
	};
	const short int POS_I_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_POS_I_GAIN,
		AXISR2_POS_I_GAIN,
		AXISR3_POS_I_GAIN,
		AXISR4_POS_I_GAIN,
		AXISR5_POS_I_GAIN,
		AXISR6_POS_I_GAIN,
		AXISR7_POS_I_GAIN
	};
	const short int POS_I_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_POS_I_GAIN,
		AXISL2_POS_I_GAIN,
		AXISL3_POS_I_GAIN,
		AXISL4_POS_I_GAIN,
		AXISL5_POS_I_GAIN,
		AXISL6_POS_I_GAIN,
		AXISL7_POS_I_GAIN
	};
	const short int POS_D_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_POS_D_GAIN,
		AXISR2_POS_D_GAIN,
		AXISR3_POS_D_GAIN,
		AXISR4_POS_D_GAIN,
		AXISR5_POS_D_GAIN,
		AXISR6_POS_D_GAIN,
		AXISR7_POS_D_GAIN
	};
	const short int POS_D_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_POS_D_GAIN,
		AXISL2_POS_D_GAIN,
		AXISL3_POS_D_GAIN,
		AXISL4_POS_D_GAIN,
		AXISL5_POS_D_GAIN,
		AXISL6_POS_D_GAIN,
		AXISL7_POS_D_GAIN
	};

	//==write PID para  ==//
	int i=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//==Set P==//
		dxl2_write_word(gMapRAxisID[i],POS_P_GAIN,POS_P_GAIN_R[i]);//right
		dxl2_write_word(gMapLAxisID[i],POS_P_GAIN,POS_P_GAIN_L[i]);//left
	
		//==Set I==//
		dxl2_write_word(gMapRAxisID[i],POS_I_GAIN,POS_I_GAIN_R[i]);//right
		dxl2_write_word(gMapLAxisID[i],POS_I_GAIN,POS_I_GAIN_L[i]);//left  

		//==Set D==//
		dxl2_write_word(gMapRAxisID[i], POS_D_GAIN,POS_D_GAIN_R[i]);//right
		dxl2_write_word(gMapLAxisID[i], POS_D_GAIN, POS_D_GAIN_L[i]);//left
	}


	//==read and check right hand==//
	int	txrx_result=0;
	unsigned short pos_p_gain=0;
	unsigned short pos_i_gain=0, pos_d_gain=0;
	unsigned short multi_turn_offset=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		_cprintf("===AXIS_R%d===\n",gMapAxisNO[i]);

		//==P GAIN==//
		pos_p_gain = dxl2_read_word(gMapRAxisID[i],POS_P_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read POS_P_GAIN error=%d\n",txrx_result);
		else
			_cprintf("POS_P_GAIN=%d\n", pos_p_gain);
	
		//==I GAIN==//
		pos_i_gain =dxl2_read_word(gMapRAxisID[i],POS_I_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read POS_I_GAIN error=%d\n",txrx_result);
		else	
			_cprintf("POS_I_GAIN=%d\n", pos_i_gain);

		//==D GAIN==//
		pos_d_gain=dxl2_read_word(gMapRAxisID[i],POS_D_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed Read POS_D_GAIN error=%d\n",txrx_result);
		else	
			_cprintf("POS_D_GAIN=%d\n",pos_d_gain);
	}	
	//==read and check left hand==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		_cprintf("===AXIS_L%d===\n",gMapAxisNO[i]);

		//==P GAIN==//
		pos_p_gain = dxl2_read_word(gMapLAxisID[i], POS_P_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read POS_P_GAIN error=%d\n",txrx_result);
		else
			_cprintf("POS_P_GAIN=%d\n",pos_p_gain);
	
		//==I GAIN==//
		pos_i_gain =dxl2_read_word(gMapLAxisID[i], POS_I_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed read POS_I_GAIN error=%d\n",txrx_result);
		else	
			_cprintf("POS_I_GAIN=%d\n", pos_i_gain);

		//==D GAIN==//
		pos_d_gain=dxl2_read_word(gMapLAxisID[i], POS_D_GAIN);
		txrx_result = dxl_get_comm_result();
		if(txrx_result!=COMM_RXSUCCESS)
			_cprintf("Failed Read POS_D_GAIN error=%d\n",txrx_result);
		else	
			_cprintf("POS_D_GAIN=%d\n",pos_d_gain);
	}	

}

//rt=Read_pos(pos_pus,DEF_UNIT_PUS)
int Read_pos(int RLHand, double *pos,unsigned char unit)
{
	int i=0;
	short int pulse=0;
	int rt=0;

	//sync read 
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_READ);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_POS));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_POS));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));

	if (RLHand == DEF_RIGHT_HAND)
	{
		for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
	}
	else if(RLHand == DEF_LEFT_HAND)
	{
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
	}

	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;
	else
	{
		long pulse = 0;

		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
		{
			if (RLHand == DEF_RIGHT_HAND)
			{
				pulse = dxl2_get_sync_read_data_dword(gMapRAxisID[i], PRESENT_POS);
				pulse -= gr2m_offset_pulse_R[i]; //motor to robot offset =>minus offset
			}
			else if (RLHand == DEF_LEFT_HAND)
			{
				pulse = dxl2_get_sync_read_data_dword(gMapLAxisID[i], PRESENT_POS);
				pulse -= gr2m_offset_pulse_L[i];
			}

			if (unit == DEF_UNIT_RAD)
				pos[i] = pulse*DEF_RATIO_PUS_TO_RAD;
			else if (unit == DEF_UNIT_DEG)
				pos[i] = pulse*DEF_RATIO_PUS_TO_DEG;
			else if (unit == DEF_UNIT_PUS)
				pos[i] = pulse;
			else//non offset pulse
			{
				if (RLHand == DEF_RIGHT_HAND)
					pulse += gr2m_offset_pulse_R[i];
				else if (RLHand == DEF_LEFT_HAND)
					pulse += gr2m_offset_pulse_L[i];
				pos[i] = pulse;
			}
		}
	}
	return rt;
}
int Read_moving(int RLHand, unsigned char *mov)
{
	int i = 0;
	int rt = 0;

	//sync read 
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_READ);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING_LENGTH));

	if (RLHand == DEF_RIGHT_HAND)
	{
		for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
	}
	else if (RLHand == DEF_LEFT_HAND)
	{
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
	}

	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;
	else
	{
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
		{
			if (RLHand == DEF_RIGHT_HAND)
			{
				mov[i] = dxl2_get_sync_read_data_byte(gMapRAxisID[i], STILL_MOVING);
				
			}
			else if (RLHand == DEF_LEFT_HAND)
			{
				mov[i] = dxl2_get_sync_read_data_byte(gMapLAxisID[i], STILL_MOVING);
			}
		}
	}
	return rt;

}
int Read_vel(int RLHand, double *vel)
{
	int i = 0;
	short int pulse = 0;
	int rt = 0;

	//sync read 
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_READ);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_VEL_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_VEL_LENGTH));

	if (RLHand == DEF_RIGHT_HAND)
	{
		for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
	}
	else if (RLHand == DEF_LEFT_HAND)
	{
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
	}

	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;
	else
	{
		long ulvel = 0;
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++) //stanley test
		{
			if (RLHand == DEF_RIGHT_HAND)
			{
				ulvel = dxl2_get_sync_read_data_dword(gMapRAxisID[i], PRESENT_VEL);
				vel[i] = (double)abs(ulvel);
			}
			else if (RLHand == DEF_LEFT_HAND)
			{
				ulvel = dxl2_get_sync_read_data_dword(gMapLAxisID[i], PRESENT_VEL);
				vel[i] = (double)abs(ulvel);
			}
		}
	}
	return rt;

}
void TestRead_pos()
{

	//==record para==//
	double pos_deg_R[MAX_AXIS_NUM] = { 0 };
	double pos_deg_L[MAX_AXIS_NUM] = { 0 };
	double pos_deg_last_ok_R[MAX_AXIS_NUM] = { 0 };
	double pos_deg_last_ok_L[MAX_AXIS_NUM] = { 0 };
	int n = 0;
	int rt = 0;
	char buffer[100];

	gfileR.open("D://GetDrinkJointAngle_R.csv", ios::out | ios::trunc);
	gfileL.open("D://GetDrinkJointAngle_L.csv", ios::out | ios::trunc);

	while (true)
	{
		////==Read right hand
		rt = Read_pos(DEF_RIGHT_HAND, pos_deg_R, DEF_UNIT_DEG);

		if (rt == 0)
		{
			for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			{
				printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
			}
			printf("\n");


			n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, pos_deg_R[Index_AXIS1], pos_deg_R[Index_AXIS2], pos_deg_R[Index_AXIS3], pos_deg_R[Index_AXIS4], pos_deg_R[Index_AXIS5], pos_deg_R[Index_AXIS6], pos_deg_R[Index_AXIS7]);
			gfileR.write(buffer, n);

			memcpy(pos_deg_last_ok_R, pos_deg_R, sizeof(pos_deg_last_ok_R));
		}
		else //讀取失敗時，拿前一筆來補
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
			//}
			printf("\n");
			
			n = sprintf_s(buffer, sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, pos_deg_last_ok_R[Index_AXIS1], pos_deg_last_ok_R[Index_AXIS2], pos_deg_last_ok_R[Index_AXIS3], pos_deg_last_ok_R[Index_AXIS4], pos_deg_last_ok_R[Index_AXIS5], pos_deg_last_ok_R[Index_AXIS6], pos_deg_last_ok_R[Index_AXIS7]);
			gfileR.write(buffer, n);
		}

		//==Read left hand
		rt = Read_pos(DEF_LEFT_HAND, pos_deg_L, DEF_UNIT_DEG);

		if (rt == 0)
		{
			for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			{
				_cprintf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
			}
			_cprintf("\n");

			n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, pos_deg_L[Index_AXIS1], pos_deg_L[Index_AXIS2], pos_deg_L[Index_AXIS3], pos_deg_L[Index_AXIS4], pos_deg_L[Index_AXIS5], pos_deg_L[Index_AXIS6], pos_deg_L[Index_AXIS7]);
			gfileL.write(buffer, n);

			memcpy(pos_deg_last_ok_L, pos_deg_L, sizeof(pos_deg_last_ok_L));
		}
		else //讀取失敗時，拿前一筆來補
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
			//}
			printf("\n");

			n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, pos_deg_last_ok_L[Index_AXIS1], pos_deg_last_ok_L[Index_AXIS2], pos_deg_last_ok_L[Index_AXIS3], pos_deg_last_ok_L[Index_AXIS4], pos_deg_last_ok_L[Index_AXIS5], pos_deg_last_ok_L[Index_AXIS6], pos_deg_last_ok_L[Index_AXIS7]);
			gfileL.write(buffer, n);
		}

		//Sleep(10);
	}

}
int ReadPresentLoad(int RLHand,double *LoadPercent)
{
	int i=0;
	short int LoadValue=0;
	int rt=0;

	//sync read 
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_READ);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_CURRENT));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_CURRENT));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_CURRENT_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_CURRENT_LENGTH));

	if (RLHand == DEF_RIGHT_HAND)
	{
		for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
	}
	else if (RLHand == DEF_LEFT_HAND)
	{
		for (int i = Index_AXIS4; i < MAX_AXIS_NUM; i++)//stanley test
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
	}

	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;
	else
	{
		short load = 0;
		for (i = Index_AXIS4; i<MAX_AXIS_NUM; i++)
		{
			if (RLHand == DEF_RIGHT_HAND)
			{
				load=dxl2_get_sync_read_data_word(gMapRAxisID[i], PRESENT_CURRENT);
				if(i<=Index_AXIS4)
					LoadPercent[i] = (double)abs(load)/1941*100;//dxl2
				else
					LoadPercent[i] = (double)abs(load) /1000 * 100;//mx28 dxl2
				//LoadPercent[i] = (LoadValue & 0x3ff)*0.097;//read low 10 bit 0~1024 and calcualte percetage dxl1
			}
			else if (RLHand == DEF_LEFT_HAND)
			{
				load=dxl2_get_sync_read_data_word(gMapLAxisID[i], PRESENT_CURRENT);
				if (i<=Index_AXIS4)
					LoadPercent[i] = (double)abs(load) / 1941 * 100;//dxl2
				else
					LoadPercent[i] = (double)abs(load) / 1000 * 100;//mx28 dxl2
			}
		}
	}
	
	return rt;
}

void WaitMotionDoneDual()
{
	int rt=0;
	bool stillmoving=true;
	bool btemp=true;
	int cnt_err=0;

	//wait it
	while(stillmoving)
	{
		rt=IsMoving(DEF_LEFT_HAND,&btemp);
		if(rt==0)
		{
			stillmoving=btemp;
		}
		else
		{
			cnt_err++;//communication err
			if(cnt_err==10)
			{
				printf("get moving status error\n");	
				return;
			}
		}
		Sleep(500);
		printf("stillmoving..\n");	
	}
	
}

//=====================================
//自己建立的陣列class，為了要陣列相加減
//=====================================
CStaArray::CStaArray()
{
	m_ARR_SIZE=7;
	SetArray( 0, 0, 0, 0, 0, 0, 0);
}
CStaArray::CStaArray(double x,double y,double z,double alpha,double beta,double gamma,double rednt_alpha)
{
	m_ARR_SIZE=7;
	SetArray( x, y, z, alpha, beta, gamma, rednt_alpha);
}

double CStaArray::at(int index)
{
	return m_arr[index];
}

void CStaArray::SetArray(double x,double y,double z,double alpha,double beta,double gamma,double rednt_alpha)
{
	m_arr[DEF_X]=x;
	m_arr[DEF_Y]=y;;
	m_arr[DEF_Z]=z;
	m_arr[DEF_ALPHA]=alpha;
	m_arr[DEF_BETA]=beta;
	m_arr[DEF_GAMMA]=gamma;
	m_arr[DEF_REDNT_ALPHA]=rednt_alpha;
}


CStaArray CStaArray::operator*(double k)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]*k;
    
	return temp;
}


CStaArray CStaArray::operator/(double k)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]/k;
    
	return temp;
}

CStaArray CStaArray::operator+(CStaArray &other)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]+other.m_arr[i];
    
	return temp;
}

CStaArray CStaArray::operator-(CStaArray &other)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]-other.m_arr[i];
    
	return temp;
}


CStaArray gNeedle_RobotF(350,-300,30,0,0,0,0);//針點在手臂坐標系位置   
CStaArray gNeedle_ini_Plate(30,-30,0,0,0,0,0);//下針點在架子plate座標系上的初始點
CStaArray gTranFrameToRobot=gNeedle_RobotF-gNeedle_ini_Plate;//利用兩個的差值去做比較




void IKOutputToArm(CStaArray &PathPlanPoint_R,CStaArray &PathPlanPoint_L)
{
	
	//==Output to arm ==//
	//float vel_deg_R=30;
	//float vel_deg_L=30;

	//float vel_deg_R[MAX_AXIS_NUM]={0};
	//float vel_deg_L[MAX_AXIS_NUM]={0};

	

		
#ifdef MOVETOPOINT_DUAL
	static int cnt = 0;
	if(cnt%10==0)
		MoveToPoint_Dual(PathPlanPoint_R.m_arr,PathPlanPoint_L.m_arr);  //使用原本matrix大約20ms    改為opencv matri後平均2.5ms 因此cycle time想抓10ms  
#endif
	
	_cprintf("Pend_R=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f]\n", PathPlanPoint_R.at(DEF_X), PathPlanPoint_R.at(DEF_Y), PathPlanPoint_R.at(DEF_Z), PathPlanPoint_R.at(DEF_ALPHA), PathPlanPoint_R.at(DEF_BETA), PathPlanPoint_R.at(DEF_GAMMA), PathPlanPoint_R.at(DEF_REDNT_ALPHA), PathPlanPoint_L.at(DEF_X), PathPlanPoint_L.at(DEF_Y), PathPlanPoint_L.at(DEF_Z), PathPlanPoint_L.at(DEF_ALPHA), PathPlanPoint_L.at(DEF_BETA), PathPlanPoint_L.at(DEF_GAMMA), PathPlanPoint_L.at(DEF_REDNT_ALPHA));
		
	

		//==確認軌跡點==//
#ifdef CHECK_CARTESIAN_PATH
	char buffer[100];
	int k=0;
	k=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,PathPlanPoint_R.at(DEF_X),PathPlanPoint_R.at(DEF_Y),PathPlanPoint_R.at(DEF_Z),PathPlanPoint_R.at(DEF_ALPHA),PathPlanPoint_R.at(DEF_BETA),PathPlanPoint_R.at(DEF_GAMMA),PathPlanPoint_R.at(DEF_REDNT_ALPHA));
	gfileR.write(buffer,k);

	k=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,PathPlanPoint_L.at(DEF_X),PathPlanPoint_L.at(DEF_Y),PathPlanPoint_L.at(DEF_Z),PathPlanPoint_L.at(DEF_ALPHA),PathPlanPoint_L.at(DEF_BETA),PathPlanPoint_L.at(DEF_GAMMA),PathPlanPoint_L.at(DEF_REDNT_ALPHA));
	gfileL.write(buffer,k);
#endif 

#ifdef RECORD_JOINT_ANGLE
	//==record para==//
	double pos_deg_R[MAX_AXIS_NUM]={0};
	double pos_deg_L[MAX_AXIS_NUM]={0};
	double pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
	double pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
	int n=0;
	int rt=0;
	char buffer[100];
	////==Read right hand
	//rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);

	//if(rt==0)
	//{
	//	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//	//{
	//	//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
	//	//}
	//	printf("\n");

	//	
	//	n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst,pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
	//	gfileR.write(buffer,n);
	//	
	//	memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
	//}
	//else //讀取失敗時，拿前一筆來補
	//{
	//	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//	//{
	//	//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
	//	//}
	//	printf("\n");

	//	n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst,pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
	//	gfileR.write(buffer,n);
	//}

	//==Read left hand
	rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

	if(rt==0)
	{
		//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//{
		//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
		//}
		printf("\n");

		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst,pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
		gfileL.write(buffer,n);
			
		memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
	}
	else //讀取失敗時，拿前一筆來補
	{
		//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//{
		//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
		//}
		printf("\n");

		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst,pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
		gfileL.write(buffer,n);
	}
#endif

#ifdef RECORD_JOINT_VEL
	//int rt=0,n=0;
	//char buffer[100];
	double Feedback_vel_L[MAX_AXIS_NUM] = { 0 };
	rt = Read_vel(DEF_LEFT_HAND, Feedback_vel_L);

	if (rt == 0)
	{
		//for (int i = Index_AXIS4; i <= Index_AXIS7; i++)
		//{
		//	printf("L%d:%3.0f, ", gMapAxisNO[i], LoadPercent_L[i]);
		//}
		//printf("\n");


		n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, Feedback_vel_L[Index_AXIS1], Feedback_vel_L[Index_AXIS2], Feedback_vel_L[Index_AXIS3], Feedback_vel_L[Index_AXIS4], Feedback_vel_L[Index_AXIS5], Feedback_vel_L[Index_AXIS6], Feedback_vel_L[Index_AXIS7]);
		gfileJointVel_L.write(buffer, n);

		//memcpy(LoadPercent_last_ok_L, LoadPercent_L, sizeof(LoadPercent_last_ok_L));
	}
	else //讀取失敗時，拿前一筆來補
	{
		printf("read failed");
		printf("\n");

		n = sprintf_s(buffer, sizeof(buffer), "%4.3f,-1,-1,-1,-1,-1,-1\n", gstatic_abst);
		gfileJointVel_L.write(buffer, n);
	}
#endif

#ifdef RECORD_JOINT_LOAD
		
	//int rt=0,n=0;
	//char buffer[100];
	double LoadPercent_R[MAX_AXIS_NUM]={0};
	double LoadPercent_L[MAX_AXIS_NUM]={0};
	double LoadPercent_last_ok_R[MAX_AXIS_NUM]={0};
	double LoadPercent_last_ok_L[MAX_AXIS_NUM]={0};

	//rt=ReadPresentLoad(DEF_RIGHT_HAND,LoadPercent_R);

	//if(rt==0)
	//{
	//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//	{
	//		printf("R%d:%3.0f, ",gMapAxisNO[i],LoadPercent_R[i]);
	//	}
	//	printf("\n");

	//		
	//	n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,LoadPercent_R[Index_AXIS1],LoadPercent_R[Index_AXIS2],LoadPercent_R[Index_AXIS3],LoadPercent_R[Index_AXIS4],LoadPercent_R[Index_AXIS5],LoadPercent_R[Index_AXIS6],LoadPercent_R[Index_AXIS7]);
	//	gfileR.write(buffer,n);
	//		
	//	memcpy(LoadPercent_last_ok_R,LoadPercent_R,sizeof(LoadPercent_last_ok_R));
	//}
	//else //讀取失敗時，拿前一筆來補
	//{
	//	printf("read failed");
	//	printf("\n");

	//	n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,LoadPercent_last_ok_R[Index_AXIS1],LoadPercent_last_ok_R[Index_AXIS2],LoadPercent_last_ok_R[Index_AXIS3],LoadPercent_last_ok_R[Index_AXIS4],LoadPercent_last_ok_R[Index_AXIS5],LoadPercent_last_ok_R[Index_AXIS6],LoadPercent_last_ok_R[Index_AXIS7]);
	//	gfileR.write(buffer,n);
	//}

	rt=ReadPresentLoad(DEF_LEFT_HAND,LoadPercent_L);

	if(rt==0)
	{
		for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		{
			printf("L%d:%3.0f, ",gMapAxisNO[i],LoadPercent_L[i]);
		}
		printf("\n");

			
		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,LoadPercent_L[Index_AXIS1],LoadPercent_L[Index_AXIS2],LoadPercent_L[Index_AXIS3],LoadPercent_L[Index_AXIS4],LoadPercent_L[Index_AXIS5],LoadPercent_L[Index_AXIS6],LoadPercent_L[Index_AXIS7]);
		gfileJointLoad_L.write(buffer,n);
			
		memcpy(LoadPercent_last_ok_L,LoadPercent_L,sizeof(LoadPercent_last_ok_L));
	}
	else //讀取失敗時，拿前一筆來補
	{
		printf("read failed");
		printf("\n");

		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,LoadPercent_last_ok_L[Index_AXIS1],LoadPercent_last_ok_L[Index_AXIS2],LoadPercent_last_ok_L[Index_AXIS3],LoadPercent_last_ok_L[Index_AXIS4],LoadPercent_last_ok_L[Index_AXIS5],LoadPercent_last_ok_L[Index_AXIS6],LoadPercent_last_ok_L[Index_AXIS7]);
		gfileJointLoad_L.write(buffer,n);
	}
		
#endif

#ifdef RECORD_JOINT_MOVING
		//int n = 0;
		//char buffer[100];
		unsigned char mov[MAX_AXIS_NUM] = { 0 };
		rt = Read_moving(DEF_LEFT_HAND,mov);
		if (rt == 0)
		{
			n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%d,%d,%d,%d,%d,%d,%d\n", gstatic_abst, mov[Index_AXIS1], mov[Index_AXIS2], mov[Index_AXIS3], mov[Index_AXIS4], mov[Index_AXIS5], mov[Index_AXIS6], mov[Index_AXIS7]);
			gfileJointMoving_L.write(buffer, n);
		}
		else //讀取失敗時，拿前一筆來補
		{
			n = sprintf_s(buffer, sizeof(buffer), "%4.3f,3,3,3,3,3,3\n", gstatic_abst);
			gfileJointMoving_L.write(buffer, n);
		}

#endif
		gstatic_abst+=gCycleT;
}

void LineMoveTo(int Coordinate,CStaArray &L_starP,CStaArray &L_endP,CStaArray &R_starP,CStaArray &R_endP,double CostTime)
{
	//==transfer frame coordinate to robot coordinate==//
	if (Coordinate == DEF_OBJFRAME_COOR)
	{
		for (int i = 0; i < 3; i++)
		{
			L_starP.m_arr[i] = L_starP.at(i) + gTranFrameToRobot.at(i);
			L_endP.m_arr[i] = L_endP.at(i) + gTranFrameToRobot.at(i);
			R_starP.m_arr[i] = R_starP.at(i) + gTranFrameToRobot.at(i);
			R_endP.m_arr[i] = R_endP.at(i) + gTranFrameToRobot.at(i);
		}
	}

	double const DEF_ACC_L[MAX_AXIS_NUM]={30,30,30,30,30,30,30}; //len/s^2
	double const DEF_ACC_R[MAX_AXIS_NUM]={30,30,30,30,30,30,30};

	//==calculate if costtime is ok
	double acc_L_min[MAX_AXIS_NUM]={0};
	double acc_R_min[MAX_AXIS_NUM]={0};
	double tb_L[MAX_AXIS_NUM]={0}; //parabolic time
	double tb_R[MAX_AXIS_NUM]={0};

	for(int i=0;i<7;i++)//x,y,z,alpha,beta,gamma,rednt_alpha
	{
		acc_L_min[i]=4*(L_endP.at(i)-L_starP.at(i))/pow(CostTime,2);

		if (DEF_ACC_L[i] < abs(acc_L_min[i]))
			printf("L cost time too short'");
	
		tb_L[i]=(DEF_ACC_L[i]*CostTime-sqrt(pow(DEF_ACC_L[i],2)*pow(CostTime,2)-4*DEF_ACC_L[i]*(L_endP.at(i)-L_starP.at(i))))/(2*DEF_ACC_L[i]);
	
		
		acc_R_min[i]=4*(R_endP.at(i)-R_starP.at(i))/pow(CostTime,2);

		if (DEF_ACC_R[i] < abs(acc_R_min[i]))
			printf("R cost time too short'");
	
		tb_R[i]=(DEF_ACC_R[i]*CostTime-sqrt(pow(DEF_ACC_R[i],2)*pow(CostTime,2)-4*DEF_ACC_R[i]*(R_endP.at(i)-R_starP.at(i))))/(2*DEF_ACC_R[i]);	
	}




	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);

	CStaArray PathPlanPoint_R(0,0,0,0,0,0,0);
	CStaArray PathPlanPoint_L(0,0,0,0,0,0,0);
	
	for(double t=0;t<=CostTime;t+=gCycleT)
	{
		QueryPerformanceCounter(&nBeginTime); //Record cycle start time

		//==calculate cartisian point==//
		PathPlanPoint_R=R_starP+(R_endP-R_starP)*(t/CostTime);
		PathPlanPoint_L=L_starP+(L_endP-L_starP)*(t/CostTime);
		
		//==calculate cartisian point with parabolic blend==//
		for (int i=0;i<7;i++)//x,y,z,alpha,beta,gamma,rednt_alpha
		{
			if (L_starP.at(i) == L_endP.at(i))
				PathPlanPoint_L.m_arr[i]=L_endP.at(i);
			else 
			{
				if(t<tb_L[i])
					PathPlanPoint_L.m_arr[i]=L_starP.at(i)+0.5*DEF_ACC_L[i]*pow(t,2);
				else if (t<CostTime-tb_L[i])
					PathPlanPoint_L.m_arr[i]=L_starP.at(i)+0.5*DEF_ACC_L[i]*pow(tb_L[i],2)+DEF_ACC_L[i]*tb_L[i]*(t-tb_L[i]);   
				else 
					PathPlanPoint_L.m_arr[i]=L_endP.at(i)-0.5*DEF_ACC_L[i]*pow(CostTime-t,2);
			}
			
			if (R_starP.at(i) == R_endP.at(i))
				PathPlanPoint_R.m_arr[i]=R_endP.at(i);
			else 
			{
				if(t<tb_R[i])
					PathPlanPoint_R.m_arr[i]=R_starP.at(i)+0.5*DEF_ACC_R[i]*pow(t,2);
				else if (t<CostTime-tb_R[i])
					PathPlanPoint_R.m_arr[i]=R_starP.at(i)+0.5*DEF_ACC_R[i]*pow(tb_R[i],2)+DEF_ACC_R[i]*tb_R[i]*(t-tb_R[i]); 
				else
					PathPlanPoint_R.m_arr[i]=R_endP.at(i)-0.5*DEF_ACC_R[i]*pow(CostTime-t,2);
			}
		}

		//==calculate IK and Output to arm==//
		IKOutputToArm(PathPlanPoint_R,PathPlanPoint_L);

		do
		{
			Sleep(0);
			QueryPerformanceCounter(&nEndTime);
			//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
		}
		while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart < gCycleT);
	}
}
	
void RotateMoveTo(int Coordinate,
	CStaArray &L_starP,
	CStaArray &L_endP,
	CStaArray &R_starP,
	CStaArray &R_endP,
	CStaArray &arc_cen,
	double rot_rad,
	double CostTime)
{
	if (Coordinate == DEF_OBJFRAME_COOR)
	{
		for (int i = 0; i < 3; i++)
		{
			arc_cen.m_arr[i] = arc_cen.at(i) + gTranFrameToRobot.at(i);
			L_starP.m_arr[i] = L_starP.at(i) + gTranFrameToRobot.at(i);
			L_endP.m_arr[i] = L_endP.at(i) + gTranFrameToRobot.at(i);
			R_starP.m_arr[i] = R_starP.at(i) + gTranFrameToRobot.at(i);
			R_endP.m_arr[i] = R_endP.at(i) + gTranFrameToRobot.at(i);
		}
	}
	//右手圓周路徑
	double rR=sqrt(pow(R_starP.at(DEF_X)-arc_cen.at(DEF_X),2)+pow(R_starP.at(DEF_Y)-arc_cen.at(DEF_Y),2));//右手旋轉半徑
	double ini_rad_R=DEF_PI+atan((R_starP.at(DEF_Y)-arc_cen.at(DEF_Y))/(R_starP.at(DEF_X)-arc_cen.at(DEF_X)));//旋轉時的起始旋轉角度

	//左手圓周路徑
	double rL=sqrt(pow(L_starP.at(DEF_X)-arc_cen.at(DEF_X),2)+pow(L_starP.at(DEF_Y)-arc_cen.at(DEF_Y),2));
	double ini_rad_L=atan((L_starP.at(DEF_Y)-arc_cen.at(DEF_Y))/(L_starP.at(DEF_X)-arc_cen.at(DEF_X)));
	
	//
	double acc_deg_L=5; //cartesian space旋轉的角度的角速度
	double acc_deg_R=5;
	double DEF_ACC_L[MAX_AXIS_NUM]={acc_deg_L*DEF_RATIO_DEG_TO_RAD,acc_deg_L*DEF_RATIO_DEG_TO_RAD,acc_deg_L*DEF_RATIO_DEG_TO_RAD,30,30,30,30}; //item x,y,z use the same compenet to interpolate unit is rad/s^2   the rest of item's unit is len/s^2
	double DEF_ACC_R[MAX_AXIS_NUM]={acc_deg_R*DEF_RATIO_DEG_TO_RAD,acc_deg_R*DEF_RATIO_DEG_TO_RAD,acc_deg_R*DEF_RATIO_DEG_TO_RAD,30,30,30,30}; 
	double acc_L_min[MAX_AXIS_NUM]={0};
	double acc_R_min[MAX_AXIS_NUM]={0};
	double tb_L[MAX_AXIS_NUM]={0}; //parabolic time
	double tb_R[MAX_AXIS_NUM]={0};

	//==calculate tb abd check time in this segment is enough==//
	for (int i=0;i<7;i++)//x,y,z,alpha,beta,gamma,rednt_alpha
	{
		//==left hand==//
		if(i<3)//前三項 xyz共用同一個差值元素
			acc_L_min[i]=4*rot_rad/(pow(CostTime,2));
		else
			acc_L_min[i]=4*(L_endP.at(i)-L_starP.at(i))/(pow(CostTime,2));
		
		if (DEF_ACC_L[i] < abs(acc_L_min[i]))
			printf("L cost time too short");
		
		if(i<3)
			tb_L[i]=(DEF_ACC_L[i]*CostTime-sqrt(pow(DEF_ACC_L[i],2)*pow(CostTime,2)-4*DEF_ACC_L[i]*rot_rad))/(2*DEF_ACC_L[i]);
		else
			tb_L[i]=(DEF_ACC_L[i]*CostTime-sqrt(pow(DEF_ACC_L[i],2)*pow(CostTime,2)-4*DEF_ACC_L[i]*(L_endP.at(i)-L_starP.at(i))))/(2*DEF_ACC_L[i]);
		
		//==right hand==//
		if(i<3)
			acc_R_min[i]=4*rot_rad/(pow(CostTime,2));
		else
			acc_R_min[i]=4*(R_endP.at(i)-R_starP.at(i))/(pow(CostTime,2));
		
    
		if (DEF_ACC_R[i] < abs(acc_R_min[i]))
			printf("R cost time too short");
		
		if(i<3)
			tb_R[i]=(DEF_ACC_R[i]*CostTime-sqrt(pow(DEF_ACC_R[i],2)*pow(CostTime,2)-4*DEF_ACC_R[i]*rot_rad))/(2*DEF_ACC_R[i]);    
		else
			tb_R[i]=(DEF_ACC_R[i]*CostTime-sqrt(pow(DEF_ACC_R[i],2)*pow(CostTime,2)-4*DEF_ACC_R[i]*(R_endP.at(i)-R_starP.at(i))))/(2*DEF_ACC_R[i]);    

	}

	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);

	CStaArray PathPlanPoint_R(0,0,0,0,0,0,0);
	CStaArray PathPlanPoint_L(0,0,0,0,0,0,0);

	double current_rad_R=0.0;
	double current_rad_L=0.0;

	for(double t=0;t<=CostTime;t+=gCycleT)
	{
		QueryPerformanceCounter(&nBeginTime); //Record cycle start time

		for (int i=0;i<7;i++) //x,y,z,alpha,beta,gamma,rednt_alpha
		{
			//==right hand
			if(i<3)//x,y,z
			{
				if(t<tb_R[i])
					current_rad_R=ini_rad_R+0.5*DEF_ACC_R[i]*pow(t,2);
				else if (t<CostTime-tb_R[i])
					current_rad_R=ini_rad_R+0.5*DEF_ACC_R[i]*pow(tb_R[i],2)+DEF_ACC_R[i]*tb_R[i]*(t-tb_R[i]); 
				else
					current_rad_R=(ini_rad_R+rot_rad)-0.5*DEF_ACC_R[i]*pow(CostTime-t,2);
				
			}
			else
			{
				if (R_starP.at(i) == R_endP.at(i))
					PathPlanPoint_R.m_arr[i]=R_endP.at(i);
				else
					if(t<tb_R[i])
						PathPlanPoint_R.m_arr[i]=R_starP.at(i)+0.5*DEF_ACC_R[i]*pow(t,2);
					else if (t<CostTime-tb_R[i])
						PathPlanPoint_R.m_arr[i]=R_starP.at(i)+0.5*DEF_ACC_R[i]*pow(tb_R[i],2)+DEF_ACC_R[i]*tb_R[i]*(t-tb_R[i]);   
					else
						PathPlanPoint_R.m_arr[i]=R_endP.at(i)-0.5*DEF_ACC_R[i]*pow(CostTime-t,2);
			}    
        
			//==Left hand
			if(i<3)
			{
				if(t<tb_L[i])
					current_rad_L=ini_rad_L+0.5*DEF_ACC_L[i]*pow(t,2);
				else if (t<CostTime-tb_L[i])
					current_rad_L=ini_rad_L+0.5*DEF_ACC_L[i]*pow(tb_L[i],2)+DEF_ACC_L[i]*tb_L[i]*(t-tb_L[i]); 
				else
					current_rad_L=(ini_rad_L+rot_rad)-0.5*DEF_ACC_L[i]*pow(CostTime-t,2);
				
				i=2;// 0~2 calculate the same thing current_rad_L
			}
			else
				if (L_starP.at(i) == L_endP.at(i))
					PathPlanPoint_L.m_arr[i]=L_endP.at(i);
				else
					if(t<tb_L[i])
						PathPlanPoint_L.m_arr[i]=L_starP.at(i)+0.5*DEF_ACC_L[i]*pow(t,2);
					else if (t<CostTime-tb_L[i])
						PathPlanPoint_L.m_arr[i]=L_starP.at(i)+0.5*DEF_ACC_L[i]*pow(tb_L[i],2)+DEF_ACC_L[i]*tb_L[i]*(t-tb_L[i]);   
					else
						PathPlanPoint_L.m_arr[i]=L_endP.at(i)-0.5*DEF_ACC_L[i]*pow(CostTime-t,2);			
		}
    

		//==with parabolic==//
		PathPlanPoint_R.m_arr[DEF_X]=arc_cen.at(DEF_X)+(float)(rR*cos(current_rad_R)); 
		PathPlanPoint_R.m_arr[DEF_Y]=arc_cen.at(DEF_Y)+(float)(rR*sin(current_rad_R)); 
		PathPlanPoint_R.m_arr[DEF_Z]=arc_cen.at(DEF_Z);
		//PathPlanPoint_R.m_arr[DEF_ALPHA]=R_starP.at(DEF_ALPHA)+(R_endP.at(DEF_ALPHA)-R_starP.at(DEF_ALPHA))*t/CostTime; 
		//PathPlanPoint_R.m_arr[DEF_BETA]=R_starP.at(DEF_BETA)+(R_endP.at(DEF_BETA)-R_starP.at(DEF_BETA))*t/CostTime;
		//PathPlanPoint_R.m_arr[DEF_GAMMA]=R_starP.at(DEF_GAMMA)+(R_endP.at(DEF_GAMMA)-R_starP.at(DEF_GAMMA))*t/CostTime;
		//PathPlanPoint_R.m_arr[DEF_REDNT_ALPHA]=R_starP.at(DEF_REDNT_ALPHA)+(R_endP.at(DEF_REDNT_ALPHA)-R_starP.at(DEF_REDNT_ALPHA))*t/CostTime;

		PathPlanPoint_L.m_arr[DEF_X]=arc_cen.at(DEF_X)+(float)(rL*(cos(current_rad_L))); 
		PathPlanPoint_L.m_arr[DEF_Y]=arc_cen.at(DEF_Y)+(float)(rL*(sin(current_rad_L))); 
		PathPlanPoint_L.m_arr[DEF_Z]=arc_cen.at(DEF_Z);
		//PathPlanPoint_L.m_arr[DEF_ALPHA]=L_starP.at(DEF_ALPHA)+(L_endP.at(DEF_ALPHA)-L_starP.at(DEF_ALPHA))*t/CostTime; 
		//PathPlanPoint_L.m_arr[DEF_BETA]=L_starP.at(DEF_BETA)+(L_endP.at(DEF_BETA)-L_starP.at(DEF_BETA))*t/CostTime;
		//PathPlanPoint_L.m_arr[DEF_GAMMA]=L_starP.at(DEF_GAMMA)+(L_endP.at(DEF_GAMMA)-L_starP.at(DEF_GAMMA))*t/CostTime;
		//PathPlanPoint_L.m_arr[DEF_REDNT_ALPHA]=L_starP.at(DEF_REDNT_ALPHA)+(L_endP.at(DEF_REDNT_ALPHA)-L_starP.at(DEF_REDNT_ALPHA))*t/CostTime;

		//==calculate IK and Output to arm==//
		IKOutputToArm(PathPlanPoint_R,PathPlanPoint_L);

		do
		{
			Sleep(0);
			QueryPerformanceCounter(&nEndTime);
			//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
		}
		while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart < gCycleT);
	}

}

void MoveToInitailPoint(int Coordinate,CStaArray &R_starP,CStaArray &L_starP)
{
	//Transfer To Robot frame
	if (Coordinate == DEF_OBJFRAME_COOR)
	{
		for (int i = 0; i < 3; i++)
		{
			R_starP.m_arr[i] = R_starP.at(i) + gTranFrameToRobot.at(i);
			L_starP.m_arr[i] = L_starP.at(i) + gTranFrameToRobot.at(i);
		}
	}
	double vel_deg_R=20;
	double vel_deg_L=20;

	//MoveToPoint_Dual(R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
	MoveToPoint(DEF_RIGHT_HAND,R_starP.m_arr,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,L_starP.m_arr,vel_deg_L);
	
	
	printf("move to p0..\n");
	WaitMotionDoneDual();


#ifdef	DEF_WAIT_ENTER
	printf("In initial point. press any key to continue...\n");
	getchar();
#endif
}

void TestSewingAction()
{
	//==variable for reocrd file==//
	gstatic_abst=0;
	//open file
#ifdef	RECORD_JOINT_ANGLE
	gfileR.open("D://SewJoint_FeedBack_R.csv",ios::out|ios::trunc);
	gfileL.open("D://SewJoint_FeedBack_L.csv",ios::out|ios::trunc);
#endif

#ifdef	RECORD_JOINT_VEL
	gfileJointVel_R.open("D://JointVel_FeedBack_R.csv", ios::out | ios::trunc);
	gfileJointVel_L.open("D://JointVel_FeedBack_L.csv", ios::out | ios::trunc);
#endif
	
#ifdef CHECK_CARTESIAN_PATH
	gfileR.open("D://GetSewCartesian_R.csv",ios::out|ios::trunc);
	gfileL.open("D://GetSewCartesian_L.csv",ios::out|ios::trunc);

#endif

#ifdef CHECK_JOINT_VEL_CMD
	gfileJointVelCmd_R.open("D://JointVelCmdR.csv",ios::out|ios::trunc);
	gfileJointVelCmd_L.open("D://JointVelCmdL.csv",ios::out|ios::trunc);

#endif
	

#ifdef	CHECK_JOINT_PATH
	gfileJointR.open("D://SewJoint_CMD_R.csv",ios::out|ios::trunc);
	gfileJointL.open("D://SewJoint_CMD_L.csv",ios::out|ios::trunc);
#endif




#ifdef	RECORD_JOINT_LOAD
	gfileJointLoad_R.open("D://SewJoint_LOAD_R.csv",ios::out|ios::trunc);
	gfileJointLoad_L.open("D://SewJoint_LOAD_L.csv",ios::out|ios::trunc);
#endif

#ifdef	RECORD_JOINT_MOVING
	gfileJointMoving_R.open("D://SewJoint_MOVING_R.csv", ios::out | ios::trunc);
	gfileJointMoving_L.open("D://SewJoint_MOVING_L.csv", ios::out | ios::trunc);
#endif

	//==static parameter==//
	const float MovOutLen=50;//移出抓取點的長度
	const float SewingLength=60;//縫紉行程
	const float RelMovLen=180;//框架抓取點間距

	//==MoveToInitailPoint==//
#ifdef MOVE_TO_INITIAL_POINT
	CStaArray R_IniP(-90,-90,0,50,0,0,-50);
	CStaArray L_IniP(-90,90,0,-90,0,0,90);
	MoveToInitailPoint(DEF_OBJFRAME_COOR,R_IniP,L_IniP);
	Sleep(2000);
#endif

	int IODelayTime=1000;
	int HoldTime=800;
	int RelTime=800;

#ifdef F446RE_GRIPPER_EN
	//抬壓腳 抬
	gpF446RE->FootLifter(true);
	Sleep(5000);

	//右手夾 左手夾
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND,true,HoldTime);
	Sleep(IODelayTime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND,true,HoldTime);
	Sleep(IODelayTime);

	//抬壓腳 壓
	gpF446RE->FootLifter(false);
	Sleep(IODelayTime);

	//主軸啟動
	gpF446RE->Spindle(true);
	
#endif

	//右手往正X SewingLenth 左手往正X 縫線長度 SewingLenth
	CStaArray R_starP(-90,-90,0,50,0,0,-50);
	CStaArray R_endP(-90+SewingLength,-90,0,50,0,0,-50);
	CStaArray L_starP(-90,90,0,-90,0,0,90);
	CStaArray L_endP(-90+SewingLength,90,0,-90,0,0,90);

	float CostTime=5;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

#ifdef F446RE_GRIPPER_EN	
	//主軸停止
	gpF446RE->Spindle(false);

	Sleep(IODelayTime);

	//右手不動 左手開
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND,false,RelTime);
	Sleep(IODelayTime);
#endif

	//右手不動 左手往正y移動 
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength,90,0,-90,0,0,90);
	L_endP.SetArray(-90+SewingLength,90+MovOutLen,0,-90,0,0,90);
	CostTime=5;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

	//右手不動 左手往正X 抓取點間隔長度(Release move length)
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength,90+MovOutLen,0,-90,0,0,90);
	L_endP.SetArray(-90+SewingLength+RelMovLen,90+MovOutLen,0,-60,0,0,90);
	CostTime=5;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

	//右手不動 左手往負y移動MovOutLen
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength+RelMovLen,90+MovOutLen,0,-60,0,0,90);
	L_endP.SetArray(-90+SewingLength+RelMovLen,90,0,-60,0,0,90);
	CostTime=3;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

#ifdef F446RE_GRIPPER_EN
	//右手不動 左手夾
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND,true,HoldTime);
	Sleep(IODelayTime);

	//抬壓腳抬
	gpF446RE->FootLifter(true);
	Sleep(IODelayTime);
#endif

	//右手旋轉往正X 左手旋轉往負X
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(90,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength+RelMovLen,90,0,-60,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CStaArray arc_cen(gNeedle_ini_Plate); //旋轉圓心為針在架子上的起始點
	double rot_rad=0.5*DEF_PI; //旋轉時的起始旋轉角度
	CostTime=10;
	RotateMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,arc_cen,rot_rad,CostTime);

#ifdef F446RE_GRIPPER_EN
	//抬壓腳壓
	gpF446RE->FootLifter(false);
	Sleep(IODelayTime);

	//右手開 左手不動
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND,false,RelTime);
	Sleep(IODelayTime);
#endif

	//右手往X負Y負移出  左手不動1 
	R_starP.SetArray(90,-90,0,50,0,0,-50);
	R_endP.SetArray(90-MovOutLen,-90-MovOutLen,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=5;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

	//右手往X負移動RelMovLen  左手不動1 
	R_starP.SetArray(90-MovOutLen,-90-MovOutLen,0,50,0,0,-70);
	R_endP.SetArray(90-MovOutLen-RelMovLen,-90-MovOutLen,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=10;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

	//右手往X往Y正MovOutLen  左手不動1 
	R_starP.SetArray(90-MovOutLen-RelMovLen,-90-MovOutLen,0,50,0,0,-70);
	R_endP.SetArray(90-RelMovLen,-90,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=4;
	LineMoveTo(DEF_OBJFRAME_COOR,L_starP,L_endP,R_starP,R_endP,CostTime);

#ifdef F446RE_GRIPPER_EN
	//右手夾 左手不動1
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND,true,HoldTime);
	Sleep(IODelayTime);
#endif

	
#if	defined(RECORD_JOINT_ANGLE) || defined(CHECK_CARTESIAN_PATH) 
	gfileR.close();
	gfileL.close();
#endif
	
#ifdef	CHECK_JOINT_PATH
	gfileR.close();
	gfileR.close();
#endif

}


void PickSewingObject()
{
	int IODelayTime = 1000;
	int HoldTime = 800;
	int RelTime = 800;

	double temp = 0;
	double rx=temp;//right hand target point from image
	double ry=temp;
	double rz= temp;

	double lx= temp;//left hand target point from image
	double ly= temp;
	double lz= temp;

	//the offset between catch point and stand by point
	double rx_offset= temp;
	double ry_offset= temp;
	double lx_offset= temp;
	double ly_offset= temp;


	
	//move to initail point
	CStaArray R_IniP(-90, -90, 0, 50, 0, 0, -50);
	CStaArray L_IniP(-90, 90, 0, -90, 0, 0, 90);
	MoveToInitailPoint(DEF_OBJFRAME_COOR, R_IniP, L_IniP);

	//release end effector
#ifdef F446RE_GRIPPER_EN
	//右手開 左手開
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, false, RelTime);
	Sleep(IODelayTime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, false, RelTime);
	Sleep(IODelayTime);
#endif


	//move end-effector to the point beside the catch point in the same plane(same z-axis)
	double CostTime = 5;
	CStaArray R_starP(-90 + 320, -90 - 270, 0 + 30, 50, 0, 0, -50);
	CStaArray L_starP(-90 + 320, 90 - 270, 0 + 30, -90, 0, 0, 90);
	
	CStaArray R_endP(rx+rx_offset, ry+ry_offset, rz, 50, 0, 0, -70);
	CStaArray L_endP(lx+lx_offset, ly + ly_offset, lz, 50, 0, 0, 90);
	LineMoveTo(DEF_ROBOT_COOR, L_starP, L_endP, R_starP, R_endP, CostTime);

	//move along the orient to catch
	R_starP.SetArray(rx + rx_offset, ry + ry_offset, rz, 50, 0, 0, -50);
	L_starP.SetArray(lx + lx_offset, ly + ly_offset, lz, 50, 0, 0, 90);

	R_endP.SetArray(rx, ry , rz, 50, 0, 0, -50);
	L_endP.SetArray(lx, ly, lz, 50, 0, 0, 90);
	
	CostTime = 3;
	LineMoveTo(DEF_ROBOT_COOR, L_starP, L_endP, R_starP, R_endP, CostTime);

	//catch the frame
#ifdef F446RE_GRIPPER_EN
	//右手夾 左手夾
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, true, HoldTime);
	Sleep(IODelayTime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, true, HoldTime);
	Sleep(IODelayTime);
#endif

	//take the object to up to sewing machine
	R_starP.SetArray(rx, ry, rz, 50, 0, 0, -50);
	L_starP.SetArray(lx, ly, lz, 50, 0, 0, 90);

	R_endP.SetArray(-90 + 320, -90 - 270, 0 + 30, 50, 0, 0, -50);
	L_endP.SetArray(-90 + 320, 90 - 270, 0 + 30, -90, 0, 0, 90);
	CostTime = 10;
	LineMoveTo(DEF_ROBOT_COOR, L_starP, L_endP, R_starP, R_endP, CostTime);

	//prepare for sewing action
	
}

int TestMoveToSewingHome_Dual()
{
	//float theta_R[7]={0.02,-1.02,-0.06,1.48,0.19,0.34,1.18};
	double theta_R[7]={0.08,-0.72,0.36,1.48,0.78,0.05,1.22};
	double theta_L[7]={-0.97,-0.16,1.39,1.07,0.22,-0.35,-0.91};

	//output to motor
	unsigned short int velocity_R[MAX_AXIS_NUM]={6,3,5,2,4,4,4};
	unsigned short int velocity_L[MAX_AXIS_NUM]={4,4,4,4,4,4,4};
	
	Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	WaitMotionDoneDual();

	Torque_Switch(0);
	printf("Torque_Disable\n");	
	return 0;

}

int Torque_Switch(int sw)
{	
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM * 2; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapAllAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(sw));//0 disable 1 enable
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;


	return 0;
}

int SetAllAccTo(double deg_s2)
{
	unsigned short int acc_pus=(unsigned short int)(deg_s2*DEF_RATIO_ACC_DEG_TO_PUS_DXL2);
	
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0; 
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_ACC));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_ACC));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_ACC_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_ACC_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM * 2; i++)
	{
		dxl2_set_txpacket_parameter(idx++, (unsigned char)gMapAllAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(acc_pus)));//1st data
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(acc_pus)));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(acc_pus)));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(acc_pus)));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;

	return 0;
}



int Output_to_Dynamixel(int RLHand,const double *Ang_rad,const unsigned short int *velocity)
{
	unsigned char i=0;

	//===================================================//
	//==trnasformat to pulse and offset to motor domain==//
	//====================================================//
	short int Ang_pulse[MAX_AXIS_NUM]={0};
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse[i]=(short int)(Ang_rad[i]*DEF_RATIO_RAD_TO_PUS);

		if(RLHand==DEF_RIGHT_HAND)
			Ang_pulse[i]+=gr2m_offset_pulse_R[i];
		else if(RLHand==DEF_LEFT_HAND)
			Ang_pulse[i]+=gr2m_offset_pulse_L[i];

		if( Ang_pulse[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXIS%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by syncpage===//
	//===============================//
	//unsigned short int SyncPageR[21]=
	//{ 
	//	ID_RAXIS1,(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
	//	ID_RAXIS2,(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
	//	ID_RAXIS3,(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
	//	ID_RAXIS4,(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
	//	ID_RAXIS5,(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
	//	ID_RAXIS6,(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
	//	ID_RAXIS7,(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
	//};

	//unsigned short int SyncPageL[21]=
	//{ 
	//	ID_LAXIS1,(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
	//	ID_LAXIS2,(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
	//	ID_LAXIS3,(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
	//	ID_LAXIS4,(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
	//	ID_LAXIS5,(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
	//	ID_LAXIS6,(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
	//	ID_LAXIS7,(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
	//};

	//================================//
	//==output to motor by sync write===//
	//===============================//
	//==send profile vel==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		if (RLHand == DEF_RIGHT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		else if (RLHand == DEF_LEFT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(velocity[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;

	//==send goal position==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		if (RLHand == DEF_RIGHT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		else if (RLHand == DEF_LEFT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(Ang_pulse[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(Ang_pulse[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(Ang_pulse[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(Ang_pulse[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;


	
//#if (DEBUG)
//	//for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
//	//	printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
//#endif
//	if(RLHand==DEF_RIGHT_HAND)
//		syncWrite_x86(GOAL_POSITION,2,SyncPageR,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
//	else if(RLHand==DEF_LEFT_HAND)
//		syncWrite_x86(GOAL_POSITION,2,SyncPageL,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
//	
	
	return 0;
}


int Output_to_Dynamixel_Dual(const double *Ang_rad_R,const unsigned short int *velocity_R,const double *Ang_rad_L,const unsigned short int *velocity_L)
{
	unsigned char i=0;

	//===================================================//
	//==trnasformat to pulse and offset to motor domain==//
	//====================================================//
	short int Ang_pulse_R[MAX_AXIS_NUM]={0};
	short int Ang_pulse_L[MAX_AXIS_NUM]={0};

	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse_R[i]=(short int)(Ang_rad_R[i]*DEF_RATIO_RAD_TO_PUS);
		Ang_pulse_L[i]=(short int)(Ang_rad_L[i]*DEF_RATIO_RAD_TO_PUS);

		
		Ang_pulse_R[i]+=gr2m_offset_pulse_R[i];
		Ang_pulse_L[i]+=gr2m_offset_pulse_L[i];

		if( Ang_pulse_R[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXISR%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_R[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
		if( Ang_pulse_L[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXISL%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_L[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by sync write===//
	//===============================//
	//==send profile vel==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(velocity_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(velocity_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(velocity_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(velocity_R[i])));
	}
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(velocity_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(velocity_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(velocity_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(velocity_L[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;

	//==send goal positino==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH)); 
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(Ang_pulse_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(Ang_pulse_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(Ang_pulse_R[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(Ang_pulse_R[i])));
	}
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(Ang_pulse_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(Ang_pulse_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(Ang_pulse_L[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(Ang_pulse_L[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;

	return 0;
}

int Output_to_Dynamixel_pulse(const unsigned short int *Ang_pulse,const unsigned short int *velocity) 
{
	unsigned char i=0;

	//===================================================================//
	//==limit axis  if not zero ,the return value is the overlimit axis==//
	//===================================================================//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		if( (Ang_pulse[i] > grobot_lim_pus_R_High[i]) || (Ang_pulse[i] < grobot_lim_pus_R_Low[i]) )
		{
			DBGMSG(("AXIS%d over limit Ang_pus=%d,grobot_lim_pus_L=%d,grobot_lim_pus_H=%d\n",gMapAxisNO[i],Ang_pulse[i],grobot_lim_pus_R_Low[i],grobot_lim_pus_R_High[i]))
			return -gMapAxisNO[i];
		}
	}

	//====================================================//
	//==trnasformat to pulse and offset to motor domain===//
	//====================================================//
	unsigned short int Ang_pulse_with_offset[MAX_AXIS_NUM]={0};
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse_with_offset[i]=Ang_pulse[i]+gr2m_offset_pulse_R[i];

		if( Ang_pulse_with_offset[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXIS%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_with_offset[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by syncpage===//
	//===============================//
	//unsigned short int SyncPage1[21]=
	//{ 
	//	ID_RAXIS1,Ang_pulse_with_offset[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
	//	ID_RAXIS2,Ang_pulse_with_offset[Index_AXIS2],velocity[Index_AXIS2], 
	//	ID_RAXIS3,Ang_pulse_with_offset[Index_AXIS3],velocity[Index_AXIS3], 
	//	ID_RAXIS4,Ang_pulse_with_offset[Index_AXIS4],velocity[Index_AXIS4], 
	//	ID_RAXIS5,Ang_pulse_with_offset[Index_AXIS5],velocity[Index_AXIS5], 
	//	ID_RAXIS6,Ang_pulse_with_offset[Index_AXIS6],velocity[Index_AXIS6], 
	//	ID_RAXIS7,Ang_pulse_with_offset[Index_AXIS7],velocity[Index_AXIS7], 
	//};

	//================================//
	//==output to motor by sync write===//
	//===============================//
	//==send profile vel==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(velocity[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(velocity[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;

	//==send goal position==//
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));
	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(Ang_pulse_with_offset[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(Ang_pulse_with_offset[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(Ang_pulse_with_offset[i])));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(Ang_pulse_with_offset[i])));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		return -1;


	
//#if (DEBUG)
//	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
//		printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
//#endif
//
//	syncWrite_x86(GOAL_POSITION,2,SyncPage1,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
  
	return 0;
}

//歐拉 Z1X2Y3 Intrinsic Rotaions相對於當前坐標系的的旋轉
//Matrix R_z1x2y3(float alpha,float beta,float gamma)
//{
//	Matrix ans(3,3);
//	ans << cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)	<< -cos(beta)*sin(alpha)	<< cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)
//        << cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)	<< cos(alpha)*cos(beta)		<< sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)
//        << -cos(beta)*sin(gamma)									<< sin(beta)				<< cos(beta)*cos(gamma);
//
//	return ans;
//
//}

//使用CV Mat
Mat R_z1x2y3(double alpha,double beta,double gamma)
{
	Mat ans=(Mat_<double>(3,3) <<	cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma),	-cos(beta)*sin(alpha)	,	cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta),  //0.06ms
									cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma),	cos(alpha)*cos(beta)	,	sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta),
									-cos(beta)*sin(gamma),sin(beta),cos(beta)*cos(gamma));
	return ans;
}


//float norm(const Matrix& v)
//{
//	int i=0;
//	float ans=0;
//
//	for(i=1;i<=v.getRows();i++)
//	{
//		ans+=pow(v(i,1),2);
//	}
//	
//	ans=sqrt(ans);
//
//	return ans;
//}


//Matrix Rogridues(float theta,const Matrix& V_A)
//{
//	Matrix R_a(4,4);
//
//	float cs = cos( theta );
//    float sn = sin( theta );
//
//	R_a	<<	cs + pow(V_A.getNumber(1,1),2)*(1-cs)								<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)-V_A.getNumber(3,1)*sn	<<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(2,1)*sn  <<	0
//		<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)+V_A.getNumber(3,1)*sn	<<	cos(theta)+pow(V_A.getNumber(2,1),2)*(1-cs)							<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(1,1)*sn	<<	0
//        <<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(2,1)*sn	<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(1,1)*sn	<<	cs+pow(V_A.getNumber(3,1),2)*(1-cs)									<<	0
//        <<	0																	<<	0																	<<	0																	<<	1;
//    
//	return R_a;
//}
//opencv matrix
Mat Rogridues(double theta,Mat V_A)
{
	double cs = cos( theta );
    double sn = sin( theta );

	double* pV_A=V_A.ptr<double>(0);
	


	Mat R_a=(Mat_<double>(4,4) <<	cs + pow(pV_A[0],2)*(1-cs)											,	pV_A[0]*pV_A[1]*(1-cs)-pV_A[2]*sn						,	pV_A[0]*pV_A[2]*(1-cs)+pV_A[1]*sn				,	0,
									pV_A[0]*pV_A[1]*(1-cs)+pV_A[2]*sn									,	cos(theta)+pow(pV_A[1],2)*(1-cs)						,	pV_A[1]*pV_A[2]*(1-cs)-pV_A[0]*sn				,	0,		
									pV_A[0]*pV_A[2]*(1-cs)-pV_A[1]*sn									,	pV_A[1]*pV_A[2]*(1-cs)+pV_A[0]*sn						,	cs+pow(pV_A[2],2)*(1-cs)						,	0,
									0																	,	0														,	0												,	1);

	return R_a;
}

Mat RotX( double radians )
{
    double cs = cos( radians );
    double sn = sin( radians );

    Mat rotate=(Mat_<double>(4,4) << 1 , 0  ,  0  , 0,
									0 , cs , -sn , 0,
									0 , sn ,  cs , 0,
									0 , 0  ,  0  , 1);

    return rotate;

}

Mat RotY( double radians )
{
    double cs = cos( radians );
    double sn = sin( radians );

    Mat rotate=(Mat_<double>(4,4) <<	cs		, 0   ,  sn  , 0,
									0		, 1   ,  0   , 0,
									-sn		, 0   ,  cs  , 0,
									0		, 0   ,  0   , 1);

    return rotate;
}


//int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta)
//{
//	int i=0;
//	
//	//Out put initial to zero
//	for(i=Index_AXIS1;i<=Index_AXIS7;i++)
//	{
//		theta[i]=0;
//	}
//
//	Matrix R(3,3);
//	R=R_z1x2y3(alpha,beta,gamma);
//
//	Matrix V_H_hat_x(3,1);
//	V_H_hat_x=Matrix::ExportCol(R,1);//取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
//	V_H_hat_x*=1/norm(V_H_hat_x);
//	
//	Matrix V_H_hat_y(3,1);
//	V_H_hat_y=Matrix::ExportCol(R,2);//取出歐拉角轉換的旋轉矩陣，取出第2行為Y軸旋轉後向量
//	V_H_hat_y*=1/norm(V_H_hat_y);
//	
//
//	Matrix V_r_end(3,1);
//	V_r_end	<<x_end-x_base
//			<<y_end-y_base
//			<<z_end-z_base;
//
//
//	Matrix V_r_h(3,1);
//	V_r_h=V_H_hat_x*L3;
//
//	Matrix V_r_wst(3,1);
//	V_r_wst=V_r_end-V_r_h;	
//
//	//theat 4
//	theta[Index_AXIS4]=-(float)(M_PI-acos((pow(l1,2)+pow(l2,2)-pow(norm(V_r_wst),2))/(2*l1*l2)));
//
//
//	Matrix V_r_m(3,1);
//	V_r_m=(pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;
//
//
//
//	//Redundant circle 半徑R
//	float Rednt_cir_R = pow(l1,2)- pow( (pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
//	Rednt_cir_R=sqrt(Rednt_cir_R);
//
//
//	//圓中心點到Elbow向量 V_r_u
//	Matrix V_shz(3,1);
//	V_shz	<<0
//			<<0
//			<<1;
//
//	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
//	Matrix temp_cross(3,1);
//	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //錯誤
//	V_alpha_hat=temp_cross*(1/norm(temp_cross));
//
//	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
//	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
//	V_beta_hat=temp_cross*(1/norm(temp_cross));
//
//
//
//	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha的方向和論文上的方向性相反
//	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
//	Matrix V_temp3x1(3,1);//需要寫一個可以補1的函試
//	Matrix V_temp4x1(4,1);
//	V_temp3x1=V_beta_hat*Rednt_cir_R;
//	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1
//
//	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;
//
//
//	Matrix V_R_u(3,1);
//	V_R_u.Vec_export_3_row(temp);
//	
//
//	Matrix V_r_u(3,1);
//	V_r_u=V_r_m+V_R_u;
//
//	theta[Index_AXIS1]=atan2(-V_r_u(1,1),-V_r_u(3,1));//theta(1)=atan2(-V_r_u(1),-V_r_u(3));
//
//
//	if (theta[Index_AXIS1] !=0) 
//		theta[Index_AXIS2]=atan2(V_r_u(2,1),-V_r_u(1,1)/sin(theta[Index_AXIS1]));
//	else
//		theta[Index_AXIS2]=atan2(-V_r_u(2,1),V_r_u(3,1));
//	
//
//
//	//theat 3
//	//theta(3)=atan2( sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3),cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3));
//	theta[Index_AXIS3]=atan2( sin(theta[Index_AXIS2])*sin(theta[Index_AXIS1])*V_r_wst(1,1)+cos(theta[Index_AXIS2])*V_r_wst(2,1)+sin(theta[Index_AXIS2])*cos(theta[Index_AXIS1])*V_r_wst(3,1),cos(theta[Index_AXIS1])*V_r_wst(1,1)-sin(theta[Index_AXIS1])*V_r_wst(3,1));
//
//
//
//	//theat 5
//	Matrix V_r_f(3,1);
//	V_r_f=V_r_wst-V_r_u;
//
//	Matrix V_Axis6(3,1);
//	V_Axis6=MatrixMath::cross(V_H_hat_y,-V_r_f)*(1/norm(MatrixMath::cross(V_H_hat_y,-V_r_f)));//V_Axis6=cross(V_H_hat_y,-V_r_f)/norm(cross(V_H_hat_y,-V_r_f));
//
//	Matrix V_r_wst_u(3,1);
//	V_r_wst_u=V_r_wst+V_Axis6;
//
//	Matrix A1_4(4,4);
//	A1_4=MatrixMath::RotY(theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*MatrixMath::RotZ(theta[Index_AXIS3])*MatrixMath::Tz(-l1)*MatrixMath::RotY(theta[Index_AXIS4]);//A1_4=Ry(theta(1))*Rx(theta(2))*Rz(theta(3))*Tz(-L1)*Ry(theta(4));
//	
//
//	Matrix V_temp_f(4,1);
//	Matrix V_r_wst_u_4x1(4,1);
//	V_r_wst_u_4x1.Vec_ext_1_row(V_r_wst_u,1);
//	
//
//	V_temp_f=MatrixMath::Inv(A1_4)*V_r_wst_u_4x1;//V_temp_f=inv(A1_4)*[V_r_wst_u;1]; //(3.31) 這個是補一列1上去的意思,need fix
//	theta[Index_AXIS5]=atan2(V_temp_f(2,1),V_temp_f(1,1));//theta(5)=atan2(V_temp_f(2),V_temp_f(1));
//
//	
//	//theat 6
//	Matrix V_r_wst_r(3,1);
//	V_r_wst_r=V_r_wst+V_H_hat_y;
//
//	Matrix A1_5(4,4);
//	A1_5=A1_4*MatrixMath::RotZ(theta[Index_AXIS5])*MatrixMath::Tz(-l2);//A1_5=A1_4*Rz(theta(5))*Tz(-L2);
//	
//	Matrix V_temp_g(4,4);
//	Matrix V_r_wst_r_4x1(4,1);
//	V_r_wst_r_4x1.Vec_ext_1_row(V_r_wst_r,1);
//	
//	V_temp_g=MatrixMath::Inv(A1_5)*V_r_wst_r_4x1; //V_temp_g=inv(A1_5)*[V_r_wst_r;1]; //(3.38)  這個是補一列1上去的意思,need fix
//	
//	theta[Index_AXIS6]=atan2(V_temp_g(3,1),V_temp_g(2,1));
//
//
//	//theat 7
//	Matrix V_r_wst_f(3,1);
//	V_r_wst_f=V_r_wst+V_H_hat_x;
//
//	Matrix A1_6(4,4);
//	A1_6=A1_5*MatrixMath::RotX(theta[Index_AXIS6]);
//
//	Matrix V_temp_h(3,1);
//	Matrix V_r_wst_f_4x1(4,1);
//	V_r_wst_f_4x1.Vec_ext_1_row(V_r_wst_f,1);
//	
//	V_temp_h=MatrixMath::Inv(A1_6)*V_r_wst_f_4x1; //V_temp_h=inv(A1_6)*[V_r_wst_f;1]; 
//	
//	theta[Index_AXIS7]=atan2(-V_temp_h(1,1),-V_temp_h(3,1));//theta(7)=atan2(-V_temp_h(1),-V_temp_h(3));
//
//
//	return 0;
//}


//第七軸為roll軸
//目前測試矩形路徑平均大概需要10.8ms
/*
int IK_7DOF_FB7roll(int RLHand,const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta)
{
    //輸出參數initial
	float theta[7]={0};
	float tempfloat=0.0;
    //輸入連桿長度
	//linkL[0];//L0 頭到肩膀
	//linkL[1];//L1 上臂L型長邊
	//linkL[2];//L2 上臂L型短邊
	//linkL[3];//L3 上臂L型短邊
	//linkL[4];//L4 上臂L型長邊
	//linkL[5];//L5 end effector

    // == 求出H_hat_x ==//
	Matrix  R(3,3);
    R=R_z1x2y3(PoseAngle[0],PoseAngle[1],PoseAngle[2]); //alpha,beta,gamma

    Matrix V_H_hat_x(3,1);
	V_H_hat_x=Matrix::ExportCol(R,1);//取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
	V_H_hat_x*=1/norm(V_H_hat_x);

    Matrix V_H_hat_z(3,1);
	V_H_hat_z=Matrix::ExportCol(R,3);//取出歐拉角轉換的旋轉矩陣，取出第3行為Z軸旋轉後向量
	V_H_hat_z*=1/norm(V_H_hat_z);
 
	Matrix V_r_end(3,1);
	V_r_end	<<Pend[0]-base[0]//x
			<<Pend[1]-base[1]//y
			<<Pend[2]-base[2];//z
    
	Matrix V_r_h(3,1);
    V_r_h=V_H_hat_x*linkL[5]; //L5

	Matrix V_r_wst(3,1);
	V_r_wst=V_r_end-V_r_h;	
  
	// ==Axis4== //
    float ru_norm=sqrt(pow(linkL[1],2)+pow(linkL[2],2)); //L型的斜邊長度
    float rf_norm=sqrt(pow(linkL[3],2)+pow(linkL[4],2));
	
    float theta_tmp=acos((pow(ru_norm,2) + pow(rf_norm,2)- pow(norm(V_r_wst),2)) / (2*ru_norm*rf_norm));
    theta[Index_AXIS4]=(float)(2*M_PI)-atan2(linkL[1],linkL[2])-atan2(linkL[4],linkL[3])-theta_tmp;

    // ==AXIS1 2== //
	Matrix V_r_m(3,1);
    V_r_m=(pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;

	//Redundant circle 半徑R
	float Rednt_cir_R = pow(ru_norm,2)- pow((pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);

   
	//圓中心點到Elbow向量 V_r_u
	Matrix V_shx(3,1);
	V_shx	<<1
			<<0
			<<0;

	Matrix V_shy(3,1);
	V_shy	<<0
			<<1
			<<0;

	Matrix V_shz(3,1);
	V_shz	<<0
			<<0
			<<1;

	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
	Matrix temp_cross(3,1);
	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //錯誤
	V_alpha_hat=temp_cross*(1/norm(temp_cross));

	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
	V_beta_hat=temp_cross*(1/norm(temp_cross));

	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha的方向和論文上的方向性相反
	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
	Matrix V_temp3x1(3,1);//需要寫一個可以補1的函試
	Matrix V_temp4x1(4,1);
	V_temp3x1=V_beta_hat*Rednt_cir_R;

	LARGE_INTEGER nFreq;

	LARGE_INTEGER nBeginTime;

	LARGE_INTEGER nEndTime;

	double time;
	double delaytime=1;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime); 
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1  0.23MS

	QueryPerformanceCounter(&nEndTime);
	time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart;
	


	
	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1; //7.4ms

	

	Matrix V_R_u(3,1);//V_R_u=temp(1:3,1);
	V_R_u.Vec_export_3_row(temp);
	

	Matrix V_r_u(3,1);//V_r_u=V_r_m+V_R_u;
	V_r_u=V_r_m+V_R_u;

	Matrix V_r_f(3,1);// V_r_f=V_r_wst-V_r_u;
	V_r_f=V_r_wst-V_r_u;

	Matrix Vn_u_f(3,1);//Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); //ru 及 rf的z法向量
	temp_cross=MatrixMath::cross(V_r_u,V_r_f); 
	Vn_u_f=temp_cross*(1/norm(temp_cross));
	float theta_upoff=atan(linkL[2]/linkL[1]);
	V_temp4x1.Vec_ext_1_row(V_r_u,1);//temp=Rogridues(-theta_upoff,Vn_u_f)*[V_r_u;1];  
	temp=Rogridues(-theta_upoff,Vn_u_f)*V_temp4x1;
	Matrix V_ru_l1(3,1);//旋轉 V_r_u  到V_ru_l1
	V_ru_l1.Vec_export_3_row(temp);
	
	V_ru_l1=linkL[1]*V_ru_l1*(1/norm(V_ru_l1)); //調整成L1長度
	
	theta[Index_AXIS1]=atan2(V_ru_l1(1,1),-V_ru_l1(3,1));//theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));


	if (theta[Index_AXIS1] !=0) 
		theta[Index_AXIS2]=atan2(V_ru_l1(2,1),V_ru_l1(1,1)/sin(theta[Index_AXIS1]));
	else
		theta[Index_AXIS2]=atan2(V_ru_l1(2,1),-V_ru_l1(3,1));


	// ==AXIS3== //
	//看shy(V_r_u,V_r_f的法向量)經過1,2軸旋轉後  與V_r_u,V_r_f 需要第3軸轉多少
	Matrix nV_shy;
	nV_shy=V_shy*(-1);
	V_temp4x1.Vec_ext_1_row(nV_shy,1);// V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  //第一軸和大地Y座標方向相反
	temp=MatrixMath::RotY(-theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*V_temp4x1;
	
	Matrix V_n_yrot12(3,1);
	V_n_yrot12.Vec_export_3_row(temp);//V_n_yrot12=V_n_yrot12(1:3,1);

	Matrix Vn_nuf_nyrot12;
	Vn_nuf_nyrot12=MatrixMath::cross(Vn_u_f,V_n_yrot12);
	Vn_nuf_nyrot12=Vn_nuf_nyrot12*(1/norm(Vn_nuf_nyrot12));
	tempfloat=MatrixMath::dot(V_n_yrot12,Vn_u_f)*(1/norm(V_n_yrot12))*(1/norm(Vn_u_f));// temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 
	
	//防止在acos(1.000000.....)的時候會出現虛部的情況
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}

	//Vn_u_f 和 V_n_yrot12的法向量   與 V_ru_l1同方向 theta(3)需要加負號
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1*(1/norm(V_ru_l1))) < DEF_NORM_VERY_SMALL )
		theta[Index_AXIS3]=-acos(tempfloat);
	else
		theta[Index_AXIS3]=acos(tempfloat);
	
	
    // ==Axis5== //
    float theat_lowoff=atan(linkL[3]/linkL[4]); //旋轉V_r_f 到 V_rf_l4
	V_temp4x1.Vec_ext_1_row(V_r_f,1);//temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  //旋轉 V_r_f  V_rf_l4
	temp=Rogridues(theat_lowoff,Vn_u_f)*V_temp4x1; 
	Matrix V_rf_l4(3,1);
	V_rf_l4.Vec_export_3_row(temp);// V_rf_l4=temp(1:3,1);
	V_rf_l4=V_rf_l4*linkL[4]*(1/norm(V_rf_l4)); //調整成L4長度
	
	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	Matrix Vn_rfl4_nuf;
	Vn_rfl4_nuf=MatrixMath::cross(V_rf_l4,Vn_u_f)*(1/norm(MatrixMath::cross(V_rf_l4,Vn_u_f)));//Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
	float t_rfl4_nuf=(MatrixMath::dot(Vn_rfl4_nuf,V_r_wst)- MatrixMath::dot(Vn_rfl4_nuf,V_r_end))/pow(norm(Vn_rfl4_nuf),2);//  t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); //V_n_rf,V_n_rfl4平面上，且經過V_r_end點的直線參數式的t 為rfl4_nuf
	Matrix Vproj_end_rfl4_nuf;
	Vproj_end_rfl4_nuf=V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;//V_r_end 沿著V_n_rfl4,V_n_rf平面法向量投影在平面上的點
	Matrix V_wst_to_projend_rfl4_nuf;
	V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;
	
	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	//防止在acos(1.000000.....)的時候會出現虛部的情況
	tempfloat=MatrixMath::dot(V_rf_l4,V_wst_to_projend_rfl4_nuf)*(1/norm(V_rf_l4))*(1/norm(V_wst_to_projend_rfl4_nuf));//temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL) 
	{
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}	

	Matrix Vn_rfl4_WstToProjEndRfl4Nuf;
	Vn_rfl4_WstToProjEndRfl4Nuf=MatrixMath::cross(V_rf_l4*(1/norm(V_rf_l4)) , V_wst_to_projend_rfl4_nuf*(1/norm(V_wst_to_projend_rfl4_nuf)));
	Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf*(1/norm(Vn_rfl4_WstToProjEndRfl4Nuf));
 
	//平面法向量 和 Vn_rfl4_nuf  同邊要加負號  判斷theta5要往上或往下轉
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS5]=-acos(tempfloat); 
    else
        theta[Index_AXIS5]=acos(tempfloat); 

	// ==Axis6== //
	V_temp4x1.Vec_ext_1_row(Vn_u_f,1);
	temp=Rogridues(-theta[Index_AXIS5],Vn_rfl4_nuf)*V_temp4x1; 
	Matrix Vn_nuf_rotx5_along_NRfl4Nuf(3,1);
	Vn_nuf_rotx5_along_NRfl4Nuf.Vec_export_3_row(temp);  //Vn_nuf_rotx5_along_NRfl4Nuf=temp(1:3,1);//nuf 沿著 Vn_rfl4_nuf 旋轉第5軸角度得到投影點與目標點平面的法向量
	Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf*(1/norm(Vn_nuf_rotx5_along_NRfl4Nuf));

	Matrix V_wst_to_end;
	V_wst_to_end=V_r_end-V_r_wst;

	Matrix Vn_WstToEnd_WstToProjEndRfl4Nuf;
	Vn_WstToEnd_WstToProjEndRfl4Nuf=MatrixMath::cross(V_wst_to_end,V_wst_to_projend_rfl4_nuf);//V_wst_to_projend 和 V_wst_to_end的法向量
	Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf*(1/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf));
    
	//利用法向量方向 判斷theta6旋轉方向
	tempfloat=MatrixMath::dot(V_wst_to_projend_rfl4_nuf,V_wst_to_end)*(1/norm(V_wst_to_projend_rfl4_nuf)*(1/norm(V_wst_to_end))); //temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);

	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS6]=-acos(tempfloat); 
    else
        theta[Index_AXIS6]=acos(tempfloat); 
	
	// ==Axis7== //
	Matrix V_x_rot1to6;
	V_temp4x1.Vec_ext_1_row(V_shx,1);
	V_x_rot1to6=MatrixMath::RotY(-theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*V_temp4x1;//V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  //第一軸和大地Z座標方向相反
	
	//V_shx經過1to軸旋轉後變應該要與末點座標系的Z軸貼齊
	temp=Rogridues(theta[Index_AXIS3],V_ru_l1*(1/norm(V_ru_l1)))*V_x_rot1to6;		//temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
	temp=Rogridues(theta[Index_AXIS4],Vn_u_f*(1/norm(Vn_u_f)))*temp;				//temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
	temp=Rogridues(theta[Index_AXIS5],Vn_rfl4_nuf*(1/norm(Vn_rfl4_nuf)))*temp;	//temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
	temp=Rogridues(theta[Index_AXIS6],Vn_nuf_rotx5_along_NRfl4Nuf)*temp;			//temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
	V_x_rot1to6.Vec_export_3_row(temp);										//V_x_rot1to6=temp(1:3,1); 
	V_x_rot1to6=V_x_rot1to6*(1/norm(V_x_rot1to6));

	//xrot1to6 和 V_H_hat_z 的法向量來判斷第7軸旋轉方向
	Matrix Vn_xrot1to6_VHhatz;
	Vn_xrot1to6_VHhatz=MatrixMath::cross(V_x_rot1to6,V_H_hat_z);
	Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz*(1/norm(Vn_xrot1to6_VHhatz));

	//V_shx經過123456軸旋轉後和末點座標系的Z軸還差幾度
	tempfloat=MatrixMath::dot(V_x_rot1to6,V_H_hat_z)*(1/norm(V_x_rot1to6))*(1/norm(V_H_hat_z));// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}

	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  DEF_NORM_VERY_SMALL)
        theta[Index_AXIS7]=acos(tempfloat);
    else
        theta[Index_AXIS7]=-acos(tempfloat);  
	
	
	// ==左右手第1軸方向相反== //
    if (RLHand == DEF_LEFT_HAND) //左手和右手第一軸方向相反
        theta[Index_AXIS1]=-theta[Index_AXIS1];
    
	//==output degree==//
	memcpy(out_theta,theta,sizeof(theta));

	return 0;
}
*/
//#define SHOW_MATRIX
//使用opencv matrix計算  第一次執行建構Mat 需要50ms 下次執行後時間可縮到1.5ms
int IK_7DOF_FB7roll(int RLHand,const double linkL[6],const double base[3],const double Pend[3],const double PoseAngle[3],const double Rednt_alpha, double* out_theta)
{
    //輸出參數initial
	double theta[7]={0};
	double tempdouble = 0.0;
	//輸入連桿長度
	//linkL[0];//L0 頭到肩膀
	//linkL[1];//L1 上臂L型長邊
	//linkL[2];//L2 上臂L型短邊
	//linkL[3];//L3 上臂L型短邊
	//linkL[4];//L4 上臂L型長邊
	//linkL[5];//L5 end effector
	

    //== 求出H_hat_x ==//
    Mat R=R_z1x2y3(PoseAngle[0],PoseAngle[1],PoseAngle[2]); //alpha,beta,gamma
#ifdef SHOW_MATRIX
	cout<< "R="<<endl<<""<<R<<endl<<endl;
#endif
  
	Mat V_H_hat_x=R.col(0);  //取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量//0.016ms

	
	V_H_hat_x = V_H_hat_x/norm(V_H_hat_x,NORM_L2);


#ifdef SHOW_MATRIX
	cout<< "V_H_hat_x="<<endl<<""<<V_H_hat_x<<endl<<endl;
#endif

    //==H_hat_x==//
	Mat V_H_hat_z=R.col(2); //取出歐拉角轉換的旋轉矩陣，取出第3行為Z軸旋轉後向量
	V_H_hat_z = V_H_hat_z/norm(V_H_hat_z,NORM_L2);

	Mat V_r_end=(Mat_<double>(3,1) <<Pend[0]-base[0],//x 
									Pend[1]-base[1],//y
									Pend[2]-base[2]);//z
	
   
	Mat V_r_h=V_H_hat_x*linkL[5];//L5


	Mat V_r_wst=V_r_end-V_r_h;
  
	// ==Axis4== //
    double ru_norm=sqrt(pow(linkL[1],2)+pow(linkL[2],2)); //L型的斜邊長度
    double rf_norm=sqrt(pow(linkL[3],2)+pow(linkL[4],2));
	
    double theta_tmp=acos((pow(ru_norm,2) + pow(rf_norm,2)- pow(norm(V_r_wst),2)) / (2*ru_norm*rf_norm));
    theta[Index_AXIS4]=2*M_PI-atan2(linkL[1],linkL[2])-atan2(linkL[4],linkL[3])-theta_tmp;

    // ==AXIS1 2== //
	Mat V_r_m=(pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;

	//Redundant circle 半徑R
	double Rednt_cir_R =pow(ru_norm,2)- pow((pow(ru_norm,2)-pow(rf_norm,2) + pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);

   
	//圓中心點到Elbow向量 V_r_u
	Mat V_shx=(Mat_<double>(3,1) <<	1,  
									0,
									0);

	Mat V_shy=(Mat_<double>(3,1) <<	0,  
									1,
									0);

	Mat V_shz=(Mat_<double>(3,1) <<	0,  
									0,
									1);


	Mat temp_cross=V_r_wst.cross(V_shz);
	Mat V_alpha_hat=temp_cross/norm(temp_cross,NORM_L2);

	
	temp_cross=V_r_wst.cross(V_alpha_hat);
	Mat V_beta_hat=temp_cross/norm(temp_cross,NORM_L2);

	Mat V_r_wst_unit=V_r_wst/norm(V_r_wst,NORM_L2);
	Mat V_temp3x1=V_beta_hat*Rednt_cir_R;

	double *pV_temp3x1=V_temp3x1.ptr<double>(0);
	Mat V_temp4x1=(Mat_<double>(4,1)<<	pV_temp3x1[0],
										pV_temp3x1[1],
										pV_temp3x1[2],
										1);

	Mat temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha的方向和論文上的方向性相反 //0.04ms
	
	double* ptemp=temp.ptr<double>(0);

	Mat V_R_u=(Mat_<double>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);					

	Mat V_r_u=V_r_m+V_R_u;
	Mat V_r_f=V_r_wst-V_r_u;

	temp_cross=V_r_u.cross(V_r_f);
	Mat Vn_u_f=temp_cross/norm(temp_cross,NORM_L2);//Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); //ru 及 rf的z法向量
	
	double theta_upoff=atan(linkL[2]/linkL[1]);
	double* pV_r_u=V_r_u.ptr<double>(0);
	V_temp4x1=(Mat_<double>(4,1)<<	pV_r_u[0],
									pV_r_u[1],
									pV_r_u[2],
									1);				

	temp=Rogridues(-theta_upoff,Vn_u_f)*V_temp4x1;//temp=Rogridues(-theta_upoff,Vn_u_f)*[V_r_u;1];  //旋轉 V_r_u  到V_ru_l1
	Mat V_ru_l1=(Mat_<double>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);		
	V_ru_l1=linkL[1]*V_ru_l1/norm(V_ru_l1,NORM_L2); //調整成L1長度
	
	double* pV_ru_l1=V_ru_l1.ptr<double>(0);
	theta[Index_AXIS1]=atan2(pV_ru_l1[0],-pV_ru_l1[2]);//theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));

	if (theta[Index_AXIS1] !=0) 
		theta[Index_AXIS2]=atan2(pV_ru_l1[1],pV_ru_l1[0]/sin(theta[Index_AXIS1]));
	else
		theta[Index_AXIS2]=atan2(pV_ru_l1[1],-pV_ru_l1[2]);

	
	// ==AXIS3== //
	Mat nV_shy=V_shy*(-1);
	double* pnV_shy=nV_shy.ptr<double>(0);
	V_temp4x1=(Mat_<double>(4,1)<<	pnV_shy[0],
									pnV_shy[1],
									pnV_shy[2],
									1);			
	temp=RotY(-theta[Index_AXIS1])*RotX(theta[Index_AXIS2])*V_temp4x1;// V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  //第一軸和大地Y座標方向相反

	Mat V_n_yrot12=(Mat_<double>(3,1)<<	ptemp[0],
										ptemp[1],
										ptemp[2]);		

	Mat Vn_nuf_nyrot12=Vn_u_f.cross(V_n_yrot12);
	Vn_nuf_nyrot12=Vn_nuf_nyrot12/norm(Vn_nuf_nyrot12,NORM_L2);
	tempdouble=V_n_yrot12.dot(Vn_u_f)/norm(V_n_yrot12,NORM_L2)/norm(Vn_u_f,NORM_L2);// temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 
	

	//防止在acos(1.000000.....)的時候會出現虛部的情況
	if (abs(tempdouble-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempdouble >0)
			tempdouble=1;
		else
			tempdouble=-1;
	}

	//Vn_u_f 和 V_n_yrot12的法向量   與 V_ru_l1同方向 theta(3)需要加負號
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1/norm(V_ru_l1,NORM_L2),NORM_L2) < DEF_NORM_VERY_SMALL )
		theta[Index_AXIS3]=-acos(tempdouble);
	else
		theta[Index_AXIS3]=acos(tempdouble);

	
    // ==Axis5== //
	double theat_lowoff=atan(linkL[3]/linkL[4]); //temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  //旋轉 V_r_f  V_rf_l4
	double *pV_r_f=V_r_f.ptr<double>(0);
	V_temp4x1=(Mat_<double>(4,1)<<	pV_r_f[0],
									pV_r_f[1],
									pV_r_f[2],
									1);			
	temp=Rogridues(theat_lowoff,Vn_u_f)*V_temp4x1; 
	Mat V_rf_l4=(Mat_<double>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);		
	V_rf_l4=V_rf_l4*linkL[4]/norm(V_rf_l4,NORM_L2); //調整成L4長度

	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	Mat Vn_rfl4_nuf=V_rf_l4.cross(Vn_u_f)/norm(V_rf_l4.cross(Vn_u_f));//Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
	double t_rfl4_nuf=(Vn_rfl4_nuf.dot(V_r_wst)-Vn_rfl4_nuf.dot(V_r_end))/pow(norm(Vn_rfl4_nuf,NORM_L2),2);//  t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); //V_n_rf,V_n_rfl4平面上，且經過V_r_end點的直線參數式的t 為rfl4_nuf
	Mat Vproj_end_rfl4_nuf =V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;//V_r_end 沿著V_n_rfl4,V_n_rf平面法向量投影在平面上的點
	Mat V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;

	//V_n_rfl4 及V_n_rf形成的平面 的法向量
	//防止在acos(1.000000.....)的時候會出現虛部的情況
	tempdouble=V_rf_l4.dot(V_wst_to_projend_rfl4_nuf)/norm(V_rf_l4,NORM_L2)/norm(V_wst_to_projend_rfl4_nuf,NORM_L2);//temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);

	if (abs(tempdouble-1) < DEF_COSVAL_VERY_SMALL) 
	{
		if (tempdouble >0)
			tempdouble=1;
		else
			tempdouble=-1;
	}	

	Mat V_rf_l4_unit=V_rf_l4/norm(V_rf_l4,NORM_L2);
	Mat V_wst_to_projend_rfl4_nuf_unit=V_wst_to_projend_rfl4_nuf/norm(V_wst_to_projend_rfl4_nuf,NORM_L2);
	Mat Vn_rfl4_WstToProjEndRfl4Nuf=V_rf_l4_unit.cross(V_wst_to_projend_rfl4_nuf_unit);
	Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf/norm(Vn_rfl4_WstToProjEndRfl4Nuf,NORM_L2);

	//平面法向量 和 Vn_rfl4_nuf  同邊要加負號  判斷theta5要往上或往下轉
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf,NORM_L2) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS5]=-acos(tempdouble); 
    else
        theta[Index_AXIS5]=acos(tempdouble); 

	// ==Axis6== //
	double *pVn_u_f=Vn_u_f.ptr<double>(0);
	V_temp4x1=(Mat_<double>(4,1)<<	pVn_u_f[0],
									pVn_u_f[1],
									pVn_u_f[2],
									1);			
	temp=Rogridues(-theta[Index_AXIS5],Vn_rfl4_nuf)*V_temp4x1; //nuf 沿著 Vn_rfl4_nuf 旋轉第5軸角度得到投影點與目標點平面的法向量
	Mat Vn_nuf_rotx5_along_NRfl4Nuf=(Mat_<double>(3,1)<<ptemp[0],
														ptemp[1],
														ptemp[2]);		
	Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf/norm(Vn_nuf_rotx5_along_NRfl4Nuf,NORM_L2);

	Mat V_wst_to_end=V_r_end-V_r_wst;
	    
	Mat Vn_WstToEnd_WstToProjEndRfl4Nuf=V_wst_to_end.cross(V_wst_to_projend_rfl4_nuf);//V_wst_to_projend 和 V_wst_to_end的法向量
	Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf,NORM_L2);

	//利用法向量方向 判斷theta6旋轉方向
	tempdouble=V_wst_to_projend_rfl4_nuf.dot(V_wst_to_end)/norm(V_wst_to_projend_rfl4_nuf,NORM_L2)/norm(V_wst_to_end,NORM_L2);//temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);
	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS6]=-acos(tempdouble); 
    else
        theta[Index_AXIS6]=acos(tempdouble); 
	
	// ==Axis7== //	
	double *pV_shx=V_shx.ptr<double>(0);
	V_temp4x1=(Mat_<double>(4,1)<<	pV_shx[0],
									pV_shx[1],
									pV_shx[2],
									1);		
	Mat V_x_rot1to6=RotY(-theta[Index_AXIS1])*RotX(theta[Index_AXIS2])*V_temp4x1;//V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  //第一軸和大地Z座標方向相反

	//V_shx經過1to軸旋轉後變應該要與末點座標系的Z軸貼齊
	temp=Rogridues(theta[Index_AXIS3],V_ru_l1/norm(V_ru_l1,NORM_L2))*V_x_rot1to6;		//temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
	temp=Rogridues(theta[Index_AXIS4],Vn_u_f*(1/norm(Vn_u_f)))*temp;				//temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
	temp=Rogridues(theta[Index_AXIS5],Vn_rfl4_nuf*(1/norm(Vn_rfl4_nuf)))*temp;		//temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
	temp=Rogridues(theta[Index_AXIS6],Vn_nuf_rotx5_along_NRfl4Nuf)*temp;			//temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
	
	V_x_rot1to6=(Mat_<double>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);	
	V_x_rot1to6=V_x_rot1to6/norm(V_x_rot1to6,NORM_L2);

	//xrot1to6 和 V_H_hat_z 的法向量來判斷第7軸旋轉方向
	Mat Vn_xrot1to6_VHhatz=V_x_rot1to6.cross(V_H_hat_z);
	Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz/norm(Vn_xrot1to6_VHhatz,NORM_L2);

	//V_shx經過123456軸旋轉後和末點座標系的Z軸還差幾度
	tempdouble=V_x_rot1to6.dot(V_H_hat_z)/norm(V_x_rot1to6,NORM_L2)/norm(V_H_hat_z,NORM_L2);// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));
	if (abs(tempdouble-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempdouble >0)
			tempdouble=1;
		else
			tempdouble=-1;
	}
	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x,NORM_L2) <  DEF_NORM_VERY_SMALL)
        theta[Index_AXIS7]=acos(tempdouble);
    else
        theta[Index_AXIS7]=-acos(tempdouble);  
	
	// ==左右手第1軸方向相反== //
    if (RLHand == DEF_LEFT_HAND) //左手和右手第一軸方向相反
        theta[Index_AXIS1]=-theta[Index_AXIS1];
    
	//==output degree==//
	memcpy(out_theta,theta,sizeof(theta));
	
	return 0;
}

//================================================================================//
//==prevent angle over constrain.If over occur,over_index shows which axis over ==//
//================================================================================//
bool AngleOverConstrain(int RLHand, const double theta[MAX_AXIS_NUM],int *OverIndex)
{
	bool bOver=false;
	*OverIndex=NULL;

	if (RLHand ==DEF_RIGHT_HAND)
	{
		for(int index=Index_AXIS1;index<=Index_AXIS7;index++)
		{
			if (theta[index]<grobot_lim_rad_R_Low[index] || theta[index]>grobot_lim_rad_R_High[index])  
			{
				bOver=true;
				*OverIndex=index;
				break;
			}
		}
	}
	else if(RLHand ==DEF_LEFT_HAND)
	{
		for(int index=Index_AXIS1;index<=Index_AXIS7;index++)
		{
			if (theta[index]<grobot_lim_rad_L_Low[index] || theta[index]>grobot_lim_rad_L_High[index])  
			{
				bOver=true;
				*OverIndex=index;
				break;
			}
		}
	}


	return bOver;
}


int MoveToPoint(int RLHand,double Point[7], double vel_deg)  //point[x,y,z,alpha,beta,gamma,redant_alpha]
{
	const double linkL[6]={L0,L1,L2,L3,L4,L5};
	double base[3]={0.0};
	double Pend[3]={Point[DEF_X],Point[DEF_Y],Point[DEF_Z]};
	double Pose_rad[3]={Point[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	double Rednt_alpha=Point[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;
	double theta[7]={0};
	unsigned short int vel_pus=(unsigned short int)(vel_deg*DEF_RATIO_VEL_DEG_TO_PUS_DXL2);
	int rt=0;
	
	//inverse kinematics
	if(RLHand==DEF_RIGHT_HAND)
		base[DEF_Y]=-L0;
	else if(RLHand==DEF_LEFT_HAND)
		base[DEF_Y]=L0;

	//LARGE_INTEGER nFreq;
	//LARGE_INTEGER nBeginTime;

	//LARGE_INTEGER nEndTime;

	//double time;
	//double delaytime=1;

	//QueryPerformanceFrequency(&nFreq);
	//QueryPerformanceCounter(&nBeginTime); 



	rt= IK_7DOF_FB7roll(RLHand,linkL,base,Pend,Pose_rad,Rednt_alpha,theta);

	//QueryPerformanceCounter(&nEndTime);
	//time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart;
	//printf("%fms\n",time);


	if(RLHand==DEF_RIGHT_HAND)
	{
		for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		{
			DBGMSG(("R%d:%3.0f, ",gMapAxisNO[i],theta[i]*DEF_RATIO_RAD_TO_DEG))
		}
		DBGMSG(("\n"))
	}
	else if(RLHand==DEF_LEFT_HAND)
	{
		for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		{
			DBGMSG(("L%d:%3.0f, ",gMapAxisNO[i],theta[i]*DEF_RATIO_RAD_TO_DEG))
		}
		DBGMSG(("\n"))
	}
	//==prevent angle over constrain
	int over_index=0;
	bool bOver=AngleOverConstrain(RLHand,theta,&over_index);
	if(bOver)
	{
		if(RLHand==DEF_RIGHT_HAND)
			DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_R_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_R_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		else if(RLHand==DEF_LEFT_HAND)
			DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_L_High[over_index]*DEF_RATIO_RAD_TO_DEG))

		return 1;
	}

	//output to motor
	unsigned short int velocity[MAX_AXIS_NUM]={vel_pus,vel_pus,vel_pus,vel_pus,vel_pus,vel_pus,vel_pus};
	
	rt=Output_to_Dynamixel(RLHand,theta,velocity); 

	return 0;
}

int MoveToPoint_Dual(double Point_R[7],double Point_L[7])
{
	const double linkL[6]={L0,L1,L2,L3,L4,L5};
	double base_R[3]={0,-L0,0};
	double base_L[3]={0,L0,0};

	double Pend_R[3]={Point_R[DEF_X],Point_R[DEF_Y],Point_R[DEF_Z]};
	double Pend_L[3]={Point_L[DEF_X],Point_L[DEF_Y],Point_L[DEF_Z]};
	double Pose_rad_R[3]={Point_R[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point_R[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point_R[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	double Pose_rad_L[3]={Point_L[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point_L[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point_L[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};

	double Rednt_alpha_rad_R=Point_R[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;
	double Rednt_alpha_rad_L=Point_L[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;

	//float theta_R[7]={0};
	//float theta_L[7]={0};

	CStaArray theta_rad_R;
	CStaArray theta_rad_L;

	//int vel_pus_R=(int)(vel_deg_R*DEF_RATIO_VEL_DEG_TO_PUS);//vel_deg=deg/s  0.684deg/s~702deg/s
	//int vel_pus_L=(int)(vel_deg_R*DEF_RATIO_VEL_DEG_TO_PUS);
	int rt=0;
	int over_index=0;
	bool bOver=false;

	//inverse kinematics right hand
	rt= IK_7DOF_FB7roll(DEF_RIGHT_HAND,linkL,base_R,Pend_R,Pose_rad_R,Rednt_alpha_rad_R,theta_rad_R.m_arr);


	//確認joint 使用
#ifdef CHECK_JOINT_PATH
	char buffer[100];
	int n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,theta_rad_R.m_arr[Index_AXIS1]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS2]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS3]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS4]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS5]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS6]*DEF_RATIO_RAD_TO_DEG,theta_rad_R.m_arr[Index_AXIS7]*DEF_RATIO_RAD_TO_DEG);
	gfileJointR.write(buffer,n);
#endif
	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//{
	//	DBGMSG(("R%d:%3.0f, ",gMapAxisNO[i],theta_rad_R[i]*DEF_RATIO_RAD_TO_DEG))
	//}
	//DBGMSG(("\n"))

	//==prevent angle over constrain right hand 
	over_index=0;
	bOver=AngleOverConstrain(DEF_RIGHT_HAND,theta_rad_R.m_arr,&over_index);
	if(bOver)
	{
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta_rad_R.m_arr[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_R_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_R_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		return 1;
	}

	//inverse kinematics left hand
	rt= IK_7DOF_FB7roll(DEF_LEFT_HAND,linkL,base_L,Pend_L,Pose_rad_L,Rednt_alpha_rad_L,theta_rad_L.m_arr);
	
	//確認joint 使用
#ifdef CHECK_JOINT_PATH
	 buffer[100];
	 n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,theta_rad_L.m_arr[Index_AXIS1]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS2]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS3]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS4]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS5]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS6]*DEF_RATIO_RAD_TO_DEG,theta_rad_L.m_arr[Index_AXIS7]*DEF_RATIO_RAD_TO_DEG);
	 gfileJointL.write(buffer,n);
#endif

	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//{
	//	DBGMSG(("L%d:%3.0f, ",gMapAxisNO[i],theta_rad_L[i]*DEF_RATIO_RAD_TO_DEG))
	//}
	//DBGMSG(("\n"))

	
	//==prevent angle over constrain left hand 
	over_index=0;
	bOver=AngleOverConstrain(DEF_LEFT_HAND,theta_rad_L.m_arr,&over_index);
	if(bOver)
	{
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta_rad_L.m_arr[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_L_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		return 1;
	}

	//==calculate the velocity in joint space
	static CStaArray last_theta_rad_R=theta_rad_R;
	static CStaArray last_theta_rad_L=theta_rad_L;

	const float speed_ratio=0.9f;//use the speed ratio to set a speed that cannot achieve the goal before next command to prevent the shake problem
	CStaArray vel_pus_R=(theta_rad_R-last_theta_rad_R)*DEF_RATIO_VEL_RAD_TO_PUS_DXL2*(speed_ratio/gCycleT); //vel_deg=deg/s  0.684deg/s~702deg/s
	CStaArray vel_pus_L=(theta_rad_L-last_theta_rad_L)*DEF_RATIO_VEL_RAD_TO_PUS_DXL2*(speed_ratio/gCycleT);


	last_theta_rad_R=theta_rad_R;
	last_theta_rad_L=theta_rad_L;

	//==output to motor==//
	unsigned short int vel_pus_R_int[MAX_AXIS_NUM]={0};
	unsigned short int vel_pus_L_int[MAX_AXIS_NUM]={0};

	for(int i=Index_AXIS1;i<=Index_AXIS7;i++) 
	{
		vel_pus_R_int[i]=(unsigned short int)abs(vel_pus_R.m_arr[i]);
		vel_pus_L_int[i]=(unsigned short int)abs(vel_pus_L.m_arr[i]);

		vel_pus_R_int[i] = (vel_pus_R_int[i] == 0) ? 1 :vel_pus_R_int[i];
		vel_pus_L_int[i] = (vel_pus_L_int[i] == 0) ? 1 :vel_pus_L_int[i];
	}

#ifdef CHECK_JOINT_VEL_CMD
	 //char buffer[100];

	 //int n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",gstatic_abst,vel_pus_R_int[Index_AXIS1]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS2]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS3]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS4]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS5]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS6]*DEF_RATIO_VEL_PUS_TO_DEG,vel_pus_R_int[Index_AXIS7]*DEF_RATIO_VEL_PUS_TO_DEG);
	 n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%d,%d,%d,%d,%d,%d,%d\n", gstatic_abst, vel_pus_R_int[Index_AXIS1], vel_pus_R_int[Index_AXIS2], vel_pus_R_int[Index_AXIS3], vel_pus_R_int[Index_AXIS4], vel_pus_R_int[Index_AXIS5], vel_pus_R_int[Index_AXIS6], vel_pus_R_int[Index_AXIS7]);
	 gfileJointVelCmd_R.write(buffer,n);

	 //n = sprintf_s(buffer, sizeof(buffer), "%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n", gstatic_abst, vel_pus_L_int[Index_AXIS1] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS2] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS3] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS4] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS5] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS6] * DEF_RATIO_VEL_PUS_TO_DEG, vel_pus_L_int[Index_AXIS7] * DEF_RATIO_VEL_PUS_TO_DEG);
	 n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%d,%d,%d,%d,%d,%d,%d\n",gstatic_abst,vel_pus_L_int[Index_AXIS1],vel_pus_L_int[Index_AXIS2],vel_pus_L_int[Index_AXIS3],vel_pus_L_int[Index_AXIS4],vel_pus_L_int[Index_AXIS5],vel_pus_L_int[Index_AXIS6],vel_pus_L_int[Index_AXIS7]);
	 gfileJointVelCmd_L.write(buffer,n);

#endif


	rt=Output_to_Dynamixel_Dual(theta_rad_R.m_arr,vel_pus_R_int,theta_rad_L.m_arr,vel_pus_L_int); 

	////output to motor
	//unsigned short int velocity_R[MAX_AXIS_NUM]={vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R};
	//unsigned short int velocity_L[MAX_AXIS_NUM]={vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L};
	//
	//rt=Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	return 0;
}

int IsMoving(int RLHand,bool *stillmoving)	
{
	int rt=0;
	unsigned short moving=0;

	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_READ);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING_LENGTH));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING_LENGTH));

	for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
	{
		if (RLHand == DEF_RIGHT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapRAxisID[i]);
		else if (RLHand == DEF_LEFT_HAND)
			dxl2_set_txpacket_parameter(idx++, gMapLAxisID[i]);
	}

	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();

	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		rt = 1;
	else
	{
		moving=0;
		for (int i = Index_AXIS1; i < MAX_AXIS_NUM; i++)
		{
			if (RLHand == DEF_RIGHT_HAND)
				moving = dxl2_get_sync_read_data_word(gMapRAxisID[i], STILL_MOVING);
			else if (RLHand == DEF_LEFT_HAND)
				moving = dxl2_get_sync_read_data_word(gMapLAxisID[i], STILL_MOVING);
			
			if (moving == 1)
				break;
		}
	}

	(*stillmoving)=(moving==1) ? true:false;
	return rt;
}

void QPDelay_ms(int t_ms)
{
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;

	QueryPerformanceFrequency(&nFreq);
	

	QueryPerformanceCounter(&nBeginTime); 
	do
	{
		Sleep(0);
		QueryPerformanceCounter(&nEndTime);
		//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
	}
	while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart < t_ms);
}	

//int syncWrite_x86(unsigned short int start_addr, unsigned short int data_length, unsigned short int *param, unsigned short int param_length) // WORD(16bit) syncwrite() for DXL  stanley
//{
//    //syncWrite_u16base(GOAL_POSITION,2,SyncPage1,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
//	dxl_set_txpacket_id(BROADCAST_ID);
//	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
//	dxl_set_txpacket_parameter(0, start_addr);
//	dxl_set_txpacket_parameter(1,data_length*2);
//
//	int slaveNum=param_length/(data_length+1);
//    int OneRawByte=(1+data_length*2);//ID(1byte) + worddata*len(2byte*len)
//
//    int i=0; //offset of slave number(number of row)
//    int j=0; //offset of data in raw
//    int k=1;//offset of int *param 
//    int index=0;
//
//    for( i=0; i<slaveNum; i++ )
//    { 
//        index=OneRawByte*i+2;
//        dxl_set_txpacket_parameter(index,(unsigned char)param[i*(data_length+1)]);//ID
//        k=1;
//
//        for(j=1;j<OneRawByte;j+=2)
//        {
//            dxl_set_txpacket_parameter(index+j,(unsigned char)(param[i*(data_length+1)+k]&0xff)); //DATA L    
//            dxl_set_txpacket_parameter(index+j+1,(unsigned char)(param[i*(data_length+1)+k]>>8)); //DATA H
//            k++;
//        }
//    } 
//	
//    dxl_set_txpacket_length(OneRawByte*slaveNum+4);
//
//	//for(int i=0;i<50;i++)
//	//	printf("gbInstructionPacket[%d]=%x\n",i,gbInstructionPacket[i]);//stanley test
//
//    dxl_txrx_packet();
//	int CommStatus =0;
//	CommStatus=dxl_get_comm_result();
//
//    return CommStatus;
//
//}

//int setPosition_x86(int ServoID, int Position, int Speed)//stanley
//{
//	dxl_set_txpacket_id(ServoID);
//	dxl_set_txpacket_instruction(INST_WRITE);
//	dxl_set_txpacket_parameter(0,GOAL_POSITION);
//	dxl_set_txpacket_parameter(1,(unsigned char)dxl_get_lowbyte(Position));
//	dxl_set_txpacket_parameter(2,(unsigned char)dxl_get_highbyte(Position));
//	dxl_set_txpacket_parameter(3,(unsigned char)dxl_get_lowbyte(Speed));
//	dxl_set_txpacket_parameter(4,(unsigned char)dxl_get_highbyte(Speed));
//	dxl_set_txpacket_length(7);
//
//	dxl_txrx_packet();
//
//	int CommStatus =0;
//	CommStatus=dxl_get_comm_result();
//
//	return CommStatus;
//}

int setPosition_x64(int ServoID, int Position, int Speed)//stanley
{
	dxl_set_txpacket_id(ServoID);
	dxl_set_txpacket_instruction(INST_WRITE);
	dxl_set_txpacket_parameter(0,GOAL_POSITION);
	dxl_set_txpacket_parameter(1,DXL_LOBYTE(Position));
	dxl_set_txpacket_parameter(2,DXL_HIBYTE(Position));
	dxl_set_txpacket_parameter(3,DXL_LOBYTE(Speed));
	dxl_set_txpacket_parameter(4,DXL_HIBYTE(Speed));
	dxl_set_txpacket_length(7);

	dxl_txrx_packet();

	int CommStatus =0;
	CommStatus=dxl_get_comm_result();

	return CommStatus;
}



int DXL_Initial()
{
	int rt=0;
	//const int default_portnum=6;//latte_panda
	const int default_portnum=5;//my pc
	//const int default_portnum=5;//vaio plate
	const int default_baudnum=1000000;

	printf("DXL_port=%d\n",default_portnum);
	rt=dxl_initialize( default_portnum,default_baudnum);
	
	return rt;
}
 
int DXL_Terminate()
{
	dxl_terminate();
	
	return 0;
}



//========================
//==Modbus control gripper
//========================
#ifdef MODBUS_GRIPPER
#pragma comment(lib,"modbus.lib")

modbus_t *ctx;  //CLI不能有全域變數所以會跳LNK4248

int Initial_Modbus()
{
	
	ctx = modbus_new_rtu("COM8",460800,'N',8,1);
	
	//modbus_set_debug(ctx, 1);//會print執行狀況
 
	//設定debug偵測錯誤
	int rt=0;
	rt=modbus_connect(ctx);//連接modbus
 
	modbus_set_slave(ctx,1);//輸入設備ID＝1
 
	
	//判斷是否有正常連線
	if (rt == -1)
	{
		printf("modbus link error");
		modbus_free(ctx);
		return -1;
	}
 
	uint16_t dest[10]={0};
	/*modbus_read_registers(ctx, 4001,10, dest);
 
	modbus_write_register(ctx,4001,1000);

	modbus_read_registers(ctx, 4001,10, dest);*/
	//modbus_write_register(ctx,0x0259,0x0003); 
	//寫入資料PLC的位置 modbus_write_register(usb port,PLC位置,數值)
 
	///*  結束流程 */
	//modbus_close(ctx);
 //
	//modbus_free(ctx);
	return 0;
}

void Terminate_Modbus()
{
	/*  結束流程 */
	modbus_close(ctx);
	modbus_free(ctx);
}

#define DEF_HOLD 1
#define DEF_RELEASE 0

int GripperHold(int RLHand,bool Hold)
{
	//address4001  //RIGHT_HAND
	//address4002  //Left_HAND

	if(RLHand==DEF_RIGHT_HAND)
	{
		if(Hold)
			modbus_write_register(ctx,4000,DEF_HOLD);
		else
			modbus_write_register(ctx,4000,DEF_RELEASE);
	}
	else if(RLHand==DEF_LEFT_HAND)
	{
		if(Hold)
			modbus_write_register(ctx,4001,DEF_HOLD);
		else
			modbus_write_register(ctx,4001,DEF_RELEASE);
	}

	uint16_t dest[10]={0};
	modbus_read_registers(ctx, 4000,10, dest);
	return 0;
}

#endif

//=========================================
//==LattePanda Arduino Leonardo for Gripper
//==using C# "LattePanda Firmata
//=========================================
//using namespace  LattePandaFirmata;
//#using "LattePandaFirmata.dll"
//#include <windows.h> //sleep使用
//#pragma warning (disable: 4538)
//
//#define DEF_LATTE_D1_GRIPPER_R1		1
//#define DEF_LATTE_D2_GRIPPER_R2		2
//#define DEF_LATTE_D3_GRIPPER_RPWM	3
//
//#define DEF_LATTE_D4_GRIPPER_L1		4
//#define DEF_LATTE_D5_GRIPPER_L2		5
//#define DEF_LATTE_D6_GRIPPER_LPWM	6
//
//#define DEF_LATTE_D13_LED		13
//ref class GlobalObjects
//{
//public:
//	static Arduino ^arduino=nullptr;
//
//	static int Initial()
//	{
//		arduino = gcnew Arduino();
//		arduino->pinMode(DEF_LATTE_D1_GRIPPER_R1, arduino->OUTPUT);//Set the digital pin 1 as output      
//		arduino->pinMode(DEF_LATTE_D2_GRIPPER_R2, arduino->OUTPUT);//Set the digital pin 2 as output
//		arduino->pinMode(DEF_LATTE_D3_GRIPPER_RPWM, arduino->PWM);//Set the digital pin 3 as pwm
//
//		arduino->pinMode(DEF_LATTE_D4_GRIPPER_L1, arduino->OUTPUT);//Set the digital pin 4 as output      
//		arduino->pinMode(DEF_LATTE_D5_GRIPPER_L2, arduino->OUTPUT);//Set the digital pin 5 as output
//		arduino->pinMode(DEF_LATTE_D6_GRIPPER_LPWM, arduino->PWM);//Set the digital pin 6 as pwm
//
//		arduino->pinMode(DEF_LATTE_D13_LED, arduino->OUTPUT);//Set the digital pin 13 as output just test
//
//
//		arduino->analogWrite(DEF_LATTE_D3_GRIPPER_RPWM, 185);//0~255  255:4.5V  185:3.3V
//		arduino->analogWrite(DEF_LATTE_D6_GRIPPER_LPWM, 185);//0~255
//		return 0;
//	}
//	static int Close()
//	{
//		arduino->Close();
//		return 0;
//	}
//};
//
//
//int Gripper_LattePanda_Initial()
//{
//	 GlobalObjects::Initial();
//	 return 0;
//}
//
//void Gripper_LattePanda_Close()
//{
//	 GlobalObjects::Close();
//}
//
//int Gripper_LattePanda_Hold(int RLHand,bool Hold,int delay_ms)
//{
//	//=========
//	//=Test LED
//	//=========
//    //for(int i=0;i<5;i++)
//    //{
//    //    // ==== set the led on or off  
//    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->HIGH);//set the LED　on  
//    //    Sleep(500);//delay a seconds  
//    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->LOW);//set the LED　off  
//    //    Sleep(500);//delay a seconds  
//    //}
//
//	if(RLHand==DEF_RIGHT_HAND)
//	{
//		if(Hold)
//		{
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->HIGH);
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->LOW);
//		}
//		else
//		{
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->LOW);
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->HIGH);
//		}
//
//		QPDelay_ms(delay_ms);
//	
//		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->LOW);
//		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->LOW);
//
//	}
//	else if(RLHand==DEF_LEFT_HAND)
//	{
//		if(Hold)
//		{
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->HIGH);
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->LOW);
//		}
//		else
//		{
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->LOW);
//			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->HIGH);
//		}
//
//		QPDelay_ms(delay_ms);
//
//		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->LOW);
//		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->LOW);
//	}
//
//
//
//	return 0;
//}


//=========================================
//==RoboticIO
//=========================================
//using namespace RoboticArmIO;
//#using "RoboticArmIO.dll"
//#pragma warning (disable: 4538)
//
//ref class F446REObj
//{
//public:
//	static RobotIO ^robotio=nullptr;
//
//	static int Initial()
//	{
//		robotio = gcnew RobotIO();
//		robotio->initial();
//		return 0;
//	}
//	static int Close()
//	{
//		robotio->close();
//		return 0;
//	}
//	static bool RotateMotor(bool dir,int deg) 
//	{
//		return robotio->RotateMotor(dir,deg);
//	}
//	static void Gripper_Hold(int RLHand,bool Hold,int delay_ms)
//	{
//		robotio->Gripper_Hold(RLHand,Hold,delay_ms);
//	}
//
//	static void FootLifter(bool sw)
//    {
//       robotio->FootLifter(sw);
//    }
//
//    static void Spindle(bool sw)
//    {
//         robotio->Spindle(sw);
//    }
//
//	 static void Trimmer(bool sw)
//    {
//         robotio->Trimmer(sw);
//    }
//};
//
//
//int F446RE_Initial()
//{
//	 F446REObj::Initial();
//	 return 0;
//}
//
//void F446RE_Close()
//{
//	 F446REObj::Close();
//}
//
//void F446RE_RotateMotor(bool dir,int deg)
//{
//	 F446REObj::RotateMotor(dir,deg);
//}
//
//void F446RE_Gripper_Hold(int RLHand,bool Hold,int delay_ms)
//{
//	 F446REObj::Gripper_Hold(RLHand,Hold,delay_ms);
//}
//
//void F446RE_FootLifter(bool sw)
//{
//	F446REObj::FootLifter(sw);
//}
//
//void F446RE_Spindle(bool sw)
//{
//	F446REObj::Spindle(sw);
//}
//
//void F446RE_Trimmer(bool sw)
//{
//	F446REObj::Trimmer(sw);
//}

//=================================
//==c++ rs232 communicate to f446re
//==================================
cF446RE::cF446RE()
{
	_hserialPort = NULL;
}

bool cF446RE::initial(int com, int baudrate)
{
	_hserialPort = RSLINK(com, baudrate);

	return 0;
}

void cF446RE::close()
{
	if (_hserialPort != NULL)
		CloseHandle(_hserialPort);
}

HANDLE cF446RE::RSLINK(unsigned long Port, unsigned long BRate)
{
	COMMTIMEOUTS TimeOut;
	DCB dcb;
	//CString csPort;
	//csPort.Format(L"COM%d:", Port);
	std::string ss = "";
	char sb[10];  // You had better have room for what you are sprintf()ing!
	sprintf_s(sb,10,"COM%d", Port);
	ss = sb;

	HANDLE hRS;

	hRS = CreateFile(ss.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hRS == INVALID_HANDLE_VALUE) return hRS;

	GetCommTimeouts(hRS, &TimeOut);
	TimeOut.ReadIntervalTimeout = 80;
	TimeOut.ReadTotalTimeoutMultiplier = 80;
	TimeOut.ReadTotalTimeoutConstant = 80;
	TimeOut.WriteTotalTimeoutMultiplier = 80;
	TimeOut.WriteTotalTimeoutConstant = 80;
	SetCommTimeouts(hRS, &TimeOut);

	GetCommState(hRS, &dcb);
	dcb.BaudRate = BRate;             // Current baud
	dcb.fBinary = TRUE;               // Binary mode; no EOF check
	dcb.fParity = TRUE;               // Enable parity checking
	dcb.fOutxCtsFlow = FALSE;         // No CTS output flow control
	dcb.fOutxDsrFlow = FALSE;         // No DSR output flow control
	dcb.fDtrControl = DTR_CONTROL_ENABLE; // DTR flow control type
	dcb.fDsrSensitivity = FALSE;      // DSR sensitivity
	dcb.fTXContinueOnXoff = TRUE;     // XOFF continues Tx
	dcb.fOutX = FALSE;                // No XON/XOFF out flow control
	dcb.fInX = FALSE;                 // No XON/XOFF in flow control
	dcb.fErrorChar = FALSE;           // Disable error replacement
	dcb.fNull = FALSE;                // Disable null stripping
	dcb.fRtsControl = RTS_CONTROL_ENABLE; // RTS flow control
	dcb.fAbortOnError = FALSE;        // Do not abort reads/writes on error

	dcb.StopBits = ONESTOPBIT;        // 0,1,2=1, 1.5, 2
	dcb.Parity = NOPARITY;            // 0-4=no,odd,even,mark,space
	dcb.ByteSize = 8;                 // Number of bits/byte, 4-8

	SetCommState(hRS, &dcb);  // resetting default config 

							  //SetupComm(hRS,BRate,BRate);

							  //WriteFile(hRS,send,len,&dwWrite,0);
							  //ReadFile(hRS, &Get, 1, &dwRead, 0);
							  //FlushFileBuffers(hRS);
	return hRS;
}

DWORD cF446RE::ReadComm(HANDLE hRS, LPVOID lpInBuffer, DWORD dwBytesToRead)
{
	//lpInBuffer為接收數據的緩衝區指針，dwBytesToRead為準備讀取的數據長度（字節數） 
	//串行設備狀態結構 
	DWORD dwBytesRead;//,dwErrorFlags;  
					  //設備未打開 
					  ////if(!bOpen) return 0; 
	if (hRS == INVALID_HANDLE_VALUE) return false;
	//讀取串行設備的當前狀態 
	//ClearCommError(hRS,&dwErrorFlags,&ComStat); 
	//應該讀取的數據長度 
	//dwBytesRead=min(dwBytesToRead,ComStat.cbInQue); 
	dwBytesRead = dwBytesToRead;
	if (dwBytesRead>0)
		//讀取數據 
		if (!ReadFile(hRS, lpInBuffer, dwBytesRead, &dwBytesRead, NULL)) dwBytesRead = 0;
	return dwBytesRead;
}

BOOL cF446RE::WriteComm(HANDLE hRS, LPCVOID lpSndBuffer, DWORD dwBytesToWrite)
{
	//lpSndBuffer為發送數據緩衝區指針，dwBytesToWrite為將要發送的字節長度 
	//設備已打開 
	BOOL bWriteState;
	//實際發送的字節數 
	DWORD dwBytesWritten;
	//設備未打開 
	if (hRS == INVALID_HANDLE_VALUE) return false;
	////if(!bOpen) return FALSE; 
	bWriteState = WriteFile(hRS, lpSndBuffer, dwBytesToWrite, &dwBytesWritten, NULL);
	if (!bWriteState || dwBytesToWrite != dwBytesWritten) return FALSE; //發送失敗 	
	else return TRUE; //發送成功 
}

bool cF446RE::RotateMotor(bool dir, int deg)
{
	//construct parameter
	byte dir_para = 0;
	if (dir == DEF_CLOCK_WISE)
		dir_para = 0x01;
	else
		dir_para = 0x00;

	byte Data[4] = { DEF_CMD_ROTATE_MOTOR, dir_para, (byte)(deg >> 8), (byte)(deg & 0xff) };

	if (_hserialPort != NULL)
		WriteComm(_hserialPort, Data, 4);


	_readecho = false; //wait for echo
	int readbyte = 0;
	byte ReadData;

	int count = 200;
	while (_readecho == false)
	{
		count--;
		Sleep(1000);//ms

		readbyte = ReadComm(_hserialPort, &ReadData, sizeof(ReadData));
		if (readbyte != 0)
			_readecho = true;

		if (count == 0)
			break;
	}

	return _readecho;
}

void cF446RE::Gripper_Hold(int RLHand, bool Hold, int Delay_ms)
{
	byte lr = 0;
	byte hold = 0;


	if (RLHand == DEF_RIGHT_HAND)
		lr = 1;
	else
		lr = 2;

	if (Hold == true)
		hold = 1;
	else
		hold = 0;

	byte Data[5] = { DEF_CMD_GRIP,lr,hold,(byte)(Delay_ms >> 8), (byte)(Delay_ms & 0xff) };

	if (_hserialPort != NULL)
		WriteComm(_hserialPort, Data, 5);
}

void cF446RE::FootLifter(bool sw)
{
	byte onoff = 0;
	if (sw == true)
		onoff = 1;
	else
		onoff = 0;

	byte Data[2] = { DEF_CMD_FL, onoff };

	if (_hserialPort != NULL)
		WriteComm(_hserialPort, Data, 2);

}

void cF446RE::Spindle(bool sw)
{
	byte onoff = 0;
	if (sw == true)
		onoff = 1;
	else
		onoff = 0;

	byte Data[2] = { DEF_CMD_SPINDLE, onoff };

	if (_hserialPort != NULL)
		WriteComm(_hserialPort, Data, 2);
}

void cF446RE::Trimmer(bool sw)
{
	byte onoff = 0;
	if (sw == true)
		onoff = 1;
	else
		onoff = 0;

	byte Data[2] = { DEF_CMD_TRIMMER, onoff };

	WriteComm(_hserialPort, Data, 2);
}


//========
//==CV_Test
//========
DWORD WINAPI TestThread(LPVOID lpParam)
{
	cv::VideoCapture video_in;
	video_in.open(0);

	Mat test_fram;
	while (true)
	{
		video_in >> test_fram;
		cv::imshow("test_fram", test_fram);
		cv::waitKey(100);
	}
	return 0;
}

void testcv()
{
	int RecvParam = 0;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)TestThread, (void*)&RecvParam, 0, NULL);

}


//==========
//vision on
//==========
//==============
//vision on ORB
//==============
#include "stats.h" // Stats structure definition
#include "utils.h" // Drawing and printing functions

const double akaze_thresh = 3e-4; // AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f; // RANSAC inlier threshold
const double nn_match_ratio = 0.8f; // Nearest-neighbour matching ratio
const int bb_min_inliers = 10; // Minimal number of inliers to draw bounding box
const int stats_update_period = 10; // On-screen statistics are updated every 10 frames
namespace example {
	class Tracker
	{
	public:
		Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher) :
			detector(_detector),
			matcher(_matcher)
		{}
		void setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats);
		void setFirstFrame(const Mat frame, const Mat ref_frame, Stats& stats);//stanley

		Mat process(const Mat frame, Stats& stats, vector<Point2f> &bb);//stanley add bb
		Ptr<Feature2D> getDetector() {
			return detector;
		}
	protected:
		Ptr<Feature2D> detector;
		Ptr<DescriptorMatcher> matcher;
		Mat first_frame, first_desc;
		vector<KeyPoint> first_kp;
		vector<Point2f> object_bb;
	};
	void Tracker::setFirstFrame(const Mat frame, vector<Point2f> bb, string title, Stats& stats)
	{
		cv::Point *ptMask = new cv::Point[bb.size()];
		const Point* ptContain = { &ptMask[0] };
		int iSize = static_cast<int>(bb.size());
		for (size_t i = 0; i<bb.size(); i++) {
			ptMask[i].x = static_cast<int>(bb[i].x);
			ptMask[i].y = static_cast<int>(bb[i].y);
		}
		first_frame = frame.clone();
		cv::Mat matMask = cv::Mat::zeros(frame.size(), CV_8UC1);
		cv::fillPoly(matMask, &ptContain, &iSize, 1, cv::Scalar::all(255));
		detector->detectAndCompute(first_frame, matMask, first_kp, first_desc);
		stats.keypoints = (int)first_kp.size();
		drawBoundingBox(first_frame, bb);
		putText(first_frame, title, Point(0, 60), FONT_HERSHEY_PLAIN, 5, Scalar::all(0), 4);
		object_bb = bb;
		delete[] ptMask;
	}
	void Tracker::setFirstFrame(const Mat frame, const Mat ref_frame, Stats& stats)//stanley
	{
		first_frame = ref_frame.clone();
		//cv::Mat matMask = cv::Mat::zeros(ref_frame.size(), CV_8UC1);

		detector->detectAndCompute(ref_frame, noArray(), first_kp, first_desc);
		stats.keypoints = (int)first_kp.size();

		vector<Point2f> bb;

		bb.push_back(cv::Point2f(static_cast<float>(0), static_cast<float>(0)));
		bb.push_back(cv::Point2f(static_cast<float>(ref_frame.cols), static_cast<float>(0)));
		bb.push_back(cv::Point2f(static_cast<float>(ref_frame.cols), static_cast<float>(ref_frame.rows)));
		bb.push_back(cv::Point2f(static_cast<float>(0), static_cast<float>(ref_frame.rows)));


		drawBoundingBox(first_frame, bb);
		object_bb = bb;

	}

	Mat Tracker::process(const Mat frame, Stats& stats,vector<Point2f> &bb)
	{
		TickMeter tm;
		vector<KeyPoint> kp;
		Mat desc;
		tm.start();
		detector->detectAndCompute(frame, noArray(), kp, desc);
		stats.keypoints = (int)kp.size();
		vector< vector<DMatch> > matches;
		vector<KeyPoint> matched1, matched2;
		matcher->knnMatch(first_desc, desc, matches, 2);
		for (unsigned i = 0; i < matches.size(); i++) {
			if (matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
				matched1.push_back(first_kp[matches[i][0].queryIdx]);
				matched2.push_back(kp[matches[i][0].trainIdx]);
			}
		}
		stats.matches = (int)matched1.size();
		Mat inlier_mask, homography;
		vector<KeyPoint> inliers1, inliers2;
		vector<DMatch> inlier_matches;
		if (matched1.size() >= 4) {
			homography = findHomography(Points(matched1), Points(matched2),
				RANSAC, ransac_thresh, inlier_mask);
		}
		tm.stop();
		stats.fps = 1. / tm.getTimeSec();
		if (matched1.size() < 4 || homography.empty()) {
			Mat res;
			//hconcat(first_frame, frame, res);
			res = frame;//stanley

			stats.inliers = 0;
			stats.ratio = 0;
			return res;
		}
		for (unsigned i = 0; i < matched1.size(); i++) {
			if (inlier_mask.at<uchar>(i)) {
				int new_i = static_cast<int>(inliers1.size());
				inliers1.push_back(matched1[i]);
				inliers2.push_back(matched2[i]);
				inlier_matches.push_back(DMatch(new_i, new_i, 0));
			}
		}
		stats.inliers = (int)inliers1.size();
		stats.ratio = stats.inliers * 1.0 / stats.matches;
		vector<Point2f> new_bb;
		perspectiveTransform(object_bb, new_bb, homography);
		Mat frame_with_bb = frame.clone();
		if (stats.inliers >= bb_min_inliers) {
			drawBoundingBox(frame_with_bb, new_bb);
		}
		Mat res;
		//drawMatches(first_frame, inliers1, frame_with_bb, inliers2,inlier_matches, res,Scalar(255, 0, 0), Scalar(255, 0, 0));
		drawKeypoints(frame_with_bb, inliers2, frame_with_bb, Scalar::all(-1), DrawMatchesFlags::DEFAULT);//stanley
		res = frame_with_bb;//stanley
		bb = new_bb;//return object area rectangle 
		return res;
	}
}



//==============
//vision on find blue
//==============

int g_H_L = 89;
int g_H_H = 134;

int g_S_L = 95;
int g_S_H = 224;

int g_V_L = 61;
int g_V_H = 208;

int g_erode_size = 5;
RNG g_rng(12345);


#define MASK_WINDOW "mask"
#define ERODE_MASK_WINDOW "erode_mask"
#define OPENING_MASK_WINDOW "opening_mask"
#define ORB_RESULT_WINDOW "ORB_result"
#define DEST_WINDOW "dst"
void on_TrackbarNumcharge(int, void*)
{
	g_H_H = g_H_H;

}
void on_ErodeSizeChange(int, void*)
{
	g_erode_size = g_erode_size;
}

struct contoursCmpY {
	bool operator()(const Point2f &a, const Point2f &b) const {

		if (abs(a.y - b.y) < 30)
			return a.x < b.x;

		return a.y < b.y;
	}
} contoursCmpY_;

void tran2robotframe(Point2d img_cen, vector<Point2f> cam_point, vector<Point2f> &robotframe_point);

DWORD WINAPI VisionOnThread(LPVOID lpParam)
{
	//CommandLineParser parser(argc, argv, "{@input_path |0|input path can be a camera id, like 0,1,2 or a video filename}");
	//parser.printMessage();
	//string input_path = parser.get<string>(0);
	//string video_name = input_path;

	//============================================
	//==find the frame by ORB and find blue corner
	//============================================
	//==ORB variable==//
	VideoCapture video_in;
	video_in.open(0); //stanley
	Mat src;
	Stats stats, orb_stats;

	Ptr<ORB> orb = ORB::create();
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	example::Tracker orb_tracker(orb, matcher);
	
	Mat Ref_Image = imread("Ref_Image.png");
	imshow("Ref_Image", Ref_Image);//stanley

	//orb_tracker.setFirstFrame(frame, bb, "ORB", stats); //org
	orb_tracker.setFirstFrame(src, Ref_Image, stats); //stanley
	
	Stats orb_draw_stats;//stanley
	Mat  orb_res, res_frame;
	std::vector<Point2f> ObjectArea;
	int i = 0;

	//==HSV Filter variable==//
	Mat mask = Mat::zeros(src.rows, src.cols, CV_8U); //為了濾掉其他顏色
	Mat hsv;
	Mat	dst;

	namedWindow("src", WINDOW_AUTOSIZE);//show b
	namedWindow(MASK_WINDOW, WINDOW_AUTOSIZE);//show mask

	createTrackbar("HL", MASK_WINDOW, &g_H_L, 255, on_TrackbarNumcharge);
	createTrackbar("HH", MASK_WINDOW, &g_H_H, 255, on_TrackbarNumcharge);

	createTrackbar("SL", MASK_WINDOW, &g_S_L, 255, on_TrackbarNumcharge);
	createTrackbar("SH", MASK_WINDOW, &g_S_H, 255, on_TrackbarNumcharge);

	createTrackbar("VL", MASK_WINDOW, &g_V_L, 255, on_TrackbarNumcharge);
	createTrackbar("VH", MASK_WINDOW, &g_V_H, 255, on_TrackbarNumcharge);

	namedWindow(ERODE_MASK_WINDOW, WINDOW_AUTOSIZE);
	createTrackbar("erode size", ERODE_MASK_WINDOW, &g_erode_size, 30, on_ErodeSizeChange);

	while(1)
	{
		//=======//
		//==ORB==//
		//=======//
		i++;
		bool update_stats = (i % stats_update_period == 0);
		video_in >> src;
		// stop the program if no more images
		if (src.empty()) break;

		orb->setMaxFeatures(stats.keypoints);
		orb_res = orb_tracker.process(src, stats, ObjectArea);
		orb_stats += stats;
		if (update_stats)
		{
			orb_draw_stats = stats;
		}
		//drawStatistics(orb_res, orb_draw_stats);
		cv::imshow(ORB_RESULT_WINDOW, orb_res);
		waitKey(10);
		if (waitKey(1) == 27) break; //quit on ESC button
	
	
		//draw a object area mask
		cv::Point *ptMask = new cv::Point[ObjectArea.size()];
		const Point* ptContain = { &ptMask[0] };
		int iSize = static_cast<int>(ObjectArea.size());
		for (size_t i = 0; i<ObjectArea.size(); i++) 
		{
			ptMask[i].x = static_cast<int>(ObjectArea[i].x);
			ptMask[i].y = static_cast<int>(ObjectArea[i].y);
		}
		//src = src.clone();
		cv::Mat matObjectMask = cv::Mat::zeros(src.size(), CV_8UC1);
		cv::fillPoly(matObjectMask, &ptContain, &iSize, 1, cv::Scalar::all(255));


		//if (ObjectArea.size() != 4) //if the boundary box is not rectangle
		//	continue;

		//=============================
		//==find blue corner coordinate
		//=============================
		Point2f image_cen = Point2f(src.cols*0.5f, src.rows*0.5f);

		//==transfer to HSV==//
		Mat ObjInSrc;
		src.copyTo(ObjInSrc, matObjectMask);
		imshow("Obj in src", ObjInSrc);

		cvtColor(ObjInSrc, hsv, CV_BGR2HSV);//轉成hsv平面

		blur(hsv, hsv, Size(1, 1));
		
		Mat mask;
		inRange(hsv, Scalar(g_H_L, g_S_L, g_V_L), Scalar(g_H_H, g_S_H, g_V_H), mask);  //filter out blue 二值化：h值介於20~40 & s值介於0~100 & v值介於100~255

		Mat element = getStructuringElement(MORPH_RECT, Size(g_erode_size, g_erode_size));
		Mat erode_mask;
		erode(mask, erode_mask, element);

		//dilate
		Mat dilate_mask; //erode and dilate
		dilate(erode_mask, dilate_mask, element);

		//==canny==//
		Mat canny_output;
		//Mat blur_mask;
		//blur(opening_mask,blur_mask,Size(20,20));
		Canny(dilate_mask, canny_output, 3, 9, 3);

		//==find countours==//
		Mat contour_out;
		vector<Vec4i> Hierarchy;
		vector<vector<Point>> Countours;
		findContours(canny_output, Countours, Hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//RETR_EXTERNAL 只取外層 RETR_LIST取全部 CHAIN_APPROX_NONE所有輪廓點 CHAIN_APPROX_SIMPLE只取角點

		char text[100];

		vector<Moments> mu(Countours.size());
		vector<Point2f> mc(Countours.size());

		for (size_t i = 0; i < Countours.size(); i++)
		{
			mu[i] = moments(Countours[i], false); //從四個找到的角落corner中找moment
			mc[i] = Point2f(static_cast<float>(mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m01 / mu[i].m00)); //將moment中的值轉成center
		}

		std::sort(mc.begin(), mc.end(), contoursCmpY_);

		//==draw contours to dst
		//Mat drawing=Mat::zeros(canny_output.size(),CV_8UC3);
		Mat dst = src.clone();

		//計算轉換到robot frame的座標
		vector<Point2f> robotframe_point(Countours.size());
		tran2robotframe(image_cen, mc, robotframe_point);


		for (unsigned int i = 0; i<Countours.size(); i++)
		{
			circle(dst, mc[i], 1, Scalar(0, 0, 200), 1);//圈出center點

			sprintf_s(text, "   (%3.1f,%3.1f)", robotframe_point[i].x, robotframe_point[i].y);//標註四角落座標
			putText(dst, text, Point2f(mc[i].x, mc[i].y), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 100, 100));

			//Scalar color=Scalar(g_rng.uniform(0,255),g_rng.uniform(0,255),g_rng.uniform(0,255));
			Scalar color = Scalar(0, 255, 0);
			drawContours(dst, Countours, i, color, 5, 8, Hierarchy, 0, Point());
		}

		//此張影像中心的紅色十字架
		//circle(dst,image_cen,3,Scalar(0,0,200),3);
		int cross_len = 10;
		line(dst, Point2f(image_cen.x - cross_len, image_cen.y), Point2f(image_cen.x + cross_len, image_cen.y), Scalar(0, 0, 200), 2);  //crosshair horizontal
		line(dst, Point2f(image_cen.x, image_cen.y - cross_len), Point2f(image_cen.x, image_cen.y + cross_len), Scalar(0, 0, 200), 2);  //crosshair horizontal

		if (Countours.size() == 4)
		{
			line(dst, mc[0], mc[1], Scalar(0, 200, 0), 2);//將4角落點連線
			line(dst, mc[1], mc[3], Scalar(0, 200, 0), 2);
			line(dst, mc[3], mc[2], Scalar(0, 200, 0), 2);
			line(dst, mc[2], mc[0], Scalar(0, 200, 0), 2);
		}

		//show
		imshow("src", src);
		imshow(MASK_WINDOW, mask);//show mask
		imshow(ERODE_MASK_WINDOW, erode_mask);
		imshow(OPENING_MASK_WINDOW, dilate_mask);
		//imshow("blur_mask",blur_mask);
		imshow("canny output", canny_output);
		imshow("dst", dst);//show結果
		waitKey(30);

	}
	orb_stats /= i - 1;
	printStatistics("ORB", orb_stats);
	
	return 0;


}
void tran2robotframe(Point2d img_cen, vector<Point2f> cam_point, vector<Point2f> &robotframe_point)
{
	float mm_per_pixel = 0.75f;
	//float mm_per_pixel=1;
	//cam_point[0].x=img_cen.x+1; //test
	//cam_point[0].y=img_cen.y+1;//test


	Mat P_o_r = (Mat_<float>(2, 1) << 0, 0);//robot position in orignal frame
	Mat P_o_i = (Mat_<float>(2, 1) << 165, -70);//image position in image frame  mm:unit

												//旋轉矩陣
	Mat R_i2r = (Mat_<float>(2, 2) << 0, 1,
		1, 0);

	for (size_t i = 0; i < cam_point.size(); i++)
	{
		cam_point[i].x = (float)(cam_point[i].x - img_cen.x)*mm_per_pixel;
		cam_point[i].y = (float)(cam_point[i].y - img_cen.y)*mm_per_pixel;


		//point in imageA 
		Mat P_i = (Mat_<float>(2, 1) << cam_point[i].x, cam_point[i].y);
		Mat P_r = R_i2r*P_i + (P_o_i - P_o_r);

		robotframe_point[i].x = P_r.at<float>(0, 0);
		robotframe_point[i].y = P_r.at<float>(1, 0);
	}
}


void VisionOn()
{
	int RecvParam = 0;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)VisionOnThread, (void*)&RecvParam, 0, NULL);

}


void GetObjCornerCoorFromImage()
{
	//mc[0] = mc[0];
}

int dxl2test()
{
	int giID_List[2] = {1,2};


	int port_num = 5, baud_rate = 1000000;
	if (dxl_initialize(port_num, baud_rate) == 0)
	{
		_cprintf("Failed to open USB2Dynamixel!\n");
		_cprintf("Press any key to terminate...\n");
		_getch();
		return 0;
	}
	else
		_cprintf("Succeed to open USB2Dynamixel!\n\n");

	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], MAX_POS_LIMIT, 4090);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}

	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], MIN_POS_LIMIT, 0);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}




	_cprintf("/======================= read/write test =======================/\n");

	_cprintf("Torque On\n");
	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_byte(giID_List[i], TORQUE_ENABLE, 1);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Torque on Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Torque on Succeed\n", giID_List[i]);
	}

	_cprintf("\nWrite\n");


	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], PROFILE_ACC, 10);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}

	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], PROFILE_VEL, 100);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}


	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], GOAL_POSITION, 1020);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}

	_cprintf("\nRead\n");
	unsigned char is_moving = 1;
	while (is_moving)
	{
		
		for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
		{
			long pos = (long)dxl2_read_dword(giID_List[i], PRESENT_POS);
			if (dxl_get_comm_result() != COMM_RXSUCCESS)
				_cprintf(" ID %d : Read Faild\n", giID_List[i]);
			else
				_cprintf(" [ID %d : %d]", giID_List[i], pos);
		}
			//if (pos < 2000 - 300)
			//	is_moving = true;
		is_moving= dxl2_read_byte(giID_List[0], STILL_MOVING);

		
		_cprintf("\n");

		if (is_moving == 0)
			break;
	}


	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_write_dword(giID_List[i], TORQUE_ENABLE, 0);
		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf("ID %d : Write Faild\n", giID_List[i]);
		else
			_cprintf("ID %d : Write Succeed\n", giID_List[i]);
	}

	return 0;
}

int dxl2_sync_test()
{
	int giID_List[2] = { 1,14 };

	int port_num = 5, baud_rate = 1000000;
	if (dxl_initialize(port_num, baud_rate) == 0)
	{
		_cprintf("Failed to open USB2Dynamixel!\n");
		_cprintf("Press any key to terminate...\n");
		_getch();
		return 0;
	}
	else
		_cprintf("Succeed to open USB2Dynamixel!\n\n");


	
	printf("/==================== sync write/read test ====================/\n");
	
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	unsigned short idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE_LENGTH));
	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(1));//data
		
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		_cprintf(" Sync write torque enable Faild\n");

	
	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PROFILE_VEL_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PROFILE_VEL_LENGTH));
	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(10)));//1st data
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(10)));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(10)));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(10)));
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		_cprintf(" Sync write vel enable Faild\n");

	dxl2_set_txpacket_id(BROADCAST_ID);//1ms
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));
	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_LOWORD(2000)));//ID1 data
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_LOWORD(2000)));//ID1 data
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(DXL_HIWORD(2000)));//ID1 data
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(DXL_HIWORD(2000)));//ID1 data
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();
	if (dxl_get_comm_result() != COMM_RXSUCCESS)
		_cprintf(" Sync write goal position\n");

	bool is_moving = true;
	while (is_moving)
	{
		dxl2_set_txpacket_id(BROADCAST_ID);//10ms
		dxl2_set_txpacket_instruction(INST_SYNC_READ);
		idx = 0;
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(PRESENT_POS));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(PRESENT_POS));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(GOAL_POSITION_LENGTH));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(GOAL_POSITION_LENGTH));
		for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
			dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_length(idx + 3);
	
		dxl2_txrx_packet();

		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf(" Sync Read Faild\n");


		for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
		{
			long pos = dxl2_get_sync_read_data_dword(giID_List[i], PRESENT_POS);
			_cprintf(" [ID %d : %d]", giID_List[i], pos);

			
		}
		_cprintf("\n");


		dxl2_set_txpacket_id(BROADCAST_ID);
		dxl2_set_txpacket_instruction(INST_SYNC_READ);
		idx = 0;
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING));
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(STILL_MOVING_LENGTH));
		dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(STILL_MOVING_LENGTH));

		for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
			dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_length(idx + 3);
		dxl2_txrx_packet();

		if (dxl_get_comm_result() != COMM_RXSUCCESS)
			_cprintf(" Sync Read Faild\n");
		
		for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
		{
			unsigned short moving = dxl2_get_sync_read_data_word(giID_List[i], STILL_MOVING);
			_cprintf(" [ID %d : %d]", giID_List[i], moving);

			
			is_moving = (moving == 1) ? true :false;
			
		}

	}

	dxl2_set_txpacket_id(BROADCAST_ID);
	dxl2_set_txpacket_instruction(INST_SYNC_WRITE);
	idx = 0;
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE));
	dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(TORQUE_ENABLE_LENGTH)); //data length
	dxl2_set_txpacket_parameter(idx++, DXL_HIBYTE(TORQUE_ENABLE_LENGTH));
	for (int i = 0; i < (sizeof(giID_List) / sizeof(*giID_List)); i++)
	{
		dxl2_set_txpacket_parameter(idx++, (unsigned char)giID_List[i]);
		dxl2_set_txpacket_parameter(idx++, DXL_LOBYTE(0));//1st data
	}
	dxl2_set_txpacket_length(idx + 3);
	dxl2_txrx_packet();


	_cprintf("/=============== sync write/read test complete ================/\n");
	_cprintf("\n");

	return 0;
}