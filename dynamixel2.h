#ifndef _DYNAMIXEL2_HEADER_
#define _DYNAMIXEL2_HEADER_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ID				(252)
#define BROADCAST_ID		(254)  //BroadCast ID

////////////// Communication Errorrobotis1	1 List //////////////
#define	COMM_TXSUCCESS		(0)  //Succeed transmit instruction packet
#define COMM_RXSUCCESS		(1)  //Succeed get status packet
#define COMM_TXFAIL			(2)  //Failed transmit instruction packet
#define COMM_RXFAIL			(3)  //Failed get status packet
#define COMM_TXERROR		(4)  //Incorrect instruction packet
#define COMM_RXWAITING		(5)  //Now recieving status packet
#define COMM_RXTIMEOUT		(6)  //There is no status packet
#define COMM_RXCORRUPT		(7)  //Incorrect status packet

//////////////// Error Status List ////////////////
#define ERRBIT_ALERT		(128)//When the device has a problem, it is et as 1. Check "Device Status Check" value.

#define ERR_RESULT_FAIL		(1)  //Failed to process the instruction packet.
#define ERR_INSTRUCTION		(2)  //Instruction error
#define ERR_CRC				(3)  //CRC check error
#define ERR_DATA_RANGE		(4)  //Data range error
#define ERR_DATA_LENGTH		(5)  //Data length error
#define ERR_DATA_LIMIT		(6)  //Data limit error
#define ERR_ACCESS			(7)  //Access error


////////// for Protocol 1.0
#define PRT1_PKT_ID					(2)
#define PRT1_PKT_LENGTH				(3)
#define PRT1_PKT_INSTRUCTION		(4)
#define PRT1_PKT_ERRBIT				(4)
#define PRT1_PKT_PARAMETER0			(5)

////////// for Protocol 2.0
#define PRT2_PKT_HEADER0				(0)
#define PRT2_PKT_HEADER1				(1)
#define PRT2_PKT_HEADER2				(2)
#define PRT2_PKT_RESERVED				(3)
#define PRT2_PKT_ID						(4)
#define PRT2_PKT_LENGTH_L				(5)
#define PRT2_PKT_LENGTH_H				(6)
#define PRT2_PKT_INSTRUCTION			(7)
#define PRT2_INSTRUCTION_PKT_PARAMETER0	(8)
#define PRT2_PKT_ERRBIT					(8)
#define PRT2_STATUS_PKT_PARAMETER0		(9)
//////// Instruction for Dynamixel Protocol ////////
/////// Common Instruction for 1.0 and 2.0
#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)
#define	INST_BULK_READ      (146)  // 0x92
/////// Added Instruction for 2.0
#define INST_REBOOT         (8)
#define INST_STATUS         (85)   // 0x55
#define INST_SYNC_READ      (130)  // 0x82
#define INST_BULK_WRITE     (147)  // 0x93


#define PING_INFO_MODEL_NUM   (1)
#define PING_INFO_FIRM_VER	  (2)
	
///////////////// utility for value ///////////////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))


//MX64 Register for protocol 2.0 stanley///
#define ID_ADDRESS			(7)
#define RETURN_DELAY_TIME	(9)
#define	MAX_POS_LIMIT		(48)
#define	MIN_POS_LIMIT		(52)
#define TORQUE_ENABLE		(64)
#define ADDRESS_LED			(65)
#define POS_D_GAIN			(80)
#define POS_I_GAIN			(82)
#define POS_P_GAIN			(84)
#define PROFILE_ACC			(108)
#define PROFILE_VEL			(112)
#define GOAL_POSITION		(116)
#define STILL_MOVING		(122)
#define PRESENT_CURRENT		(126)
#define PRESENT_VEL			(128)
#define PRESENT_POS			(132)	

//data length
#define	GOAL_POSITION_LENGTH	4
#define	PROFILE_VEL_LENGTH		4
#define	PROFILE_ACC_LENGTH		4
#define TORQUE_ENABLE_LENGTH	1
#define	STILL_MOVING_LENGTH		1 
#define	PRESENT_CURRENT_LENGTH  2

//wait for comment
#define PRESENT_LOAD		(126)
#define GOAL_ACC			(108)
#define CW_ANGLE_LIMIT_L	(6)
#define CW_ANGLE_LIMIT_H	(7)
#define CCW_ANGLE_LIMIT_L	(8)
#define CCW_ANGLE_LIMIT_H	(9)
#define D_GAIN				(26)
#define I_GAIN				(27)
#define P_GAIN				(28)
#define MAX_TORQUE		(14)
#define MULTITURN_OFFSET	(20)
#define GOAL_SPEED		(32)
#define TORQUE_LIMIT		(34)
/////////////////// Common Method for 1.0 & 2.0 ///////////////////	
////////////// device control method //////////////
int __stdcall dxl_initialize(int port_num, int baud_rate);
int __stdcall dxl_change_baudrate(int baud_rate);
int __stdcall dxl_terminate(void);

///////// get communication result method /////////
int __stdcall dxl_get_comm_result(void);



/////////////////// Dynamixel Protocol 1.0 ///////////////////	
///////// 1.0 packet communocation method /////////
void __stdcall dxl_tx_packet(void);
void __stdcall dxl_rx_packet(void);
void __stdcall dxl_txrx_packet(void);

////////////// get/set packet methods /////////////
void __stdcall dxl_set_txpacket_id(int id);
void __stdcall dxl_set_txpacket_instruction(int instruction);
void __stdcall dxl_set_txpacket_parameter(int index, int value);
void __stdcall dxl_set_txpacket_length(int length);
int  __stdcall dxl_get_rxpacket_error(int error);
int  __stdcall dxl_get_rxpacket_error_byte(void);

int  __stdcall dxl_get_rxpacket_parameter( int index );
int  __stdcall dxl_get_rxpacket_length();

///////// high communication method /////////
void __stdcall dxl_ping(int id);
int  __stdcall dxl_read_byte(int id, int address);
void __stdcall dxl_write_byte(int id, int address, int value);
int  __stdcall dxl_read_word(int id, int address);
void __stdcall dxl_write_word(int id, int address, int value);
//////////////////////////////////////////////////////////////



/////////////////// Dynamixel Protocol 2.0 ///////////////////
///////// 1.0 packet communocation method /////////
void __stdcall dxl2_tx_packet(void);
void __stdcall dxl2_rx_packet(void);
void __stdcall dxl2_txrx_packet(void);

///////// get/set packet methods /////////
void __stdcall dxl2_set_txpacket_id(unsigned char id);
void __stdcall dxl2_set_txpacket_instruction(unsigned char instruction);
void __stdcall dxl2_set_txpacket_parameter(unsigned short index, unsigned char value);
void __stdcall dxl2_set_txpacket_length(unsigned short length);
int  __stdcall dxl2_get_rxpacket_error_byte(void);

int  __stdcall dxl2_get_rxpacket_parameter( int index );
int  __stdcall dxl2_get_rxpacket_length();

///////////// high communication method /////////
void __stdcall dxl2_ping(unsigned char id);
int  __stdcall dxl2_get_ping_result(unsigned char id, int info_num);
void __stdcall dxl2_broadcast_ping();

void __stdcall dxl2_reboot(unsigned char id);
void __stdcall dxl2_factory_reset(unsigned char id, int option);

unsigned char  __stdcall dxl2_read_byte(unsigned char id, int address);
void           __stdcall dxl2_write_byte(unsigned char id, int address, unsigned char value);
unsigned short __stdcall dxl2_read_word(unsigned char id, int address);
void           __stdcall dxl2_write_word(unsigned char id, int address, unsigned short value);
unsigned long  __stdcall dxl2_read_dword(unsigned char id, int address);
void           __stdcall dxl2_write_dword(unsigned char id, int address, unsigned long value);

//////////////// method for sync/bulk read ////////////////
unsigned char  __stdcall dxl2_get_bulk_read_data_byte(unsigned char id, unsigned int start_address);
unsigned short __stdcall dxl2_get_bulk_read_data_word(unsigned char id, unsigned int start_address);
unsigned long  __stdcall dxl2_get_bulk_read_data_dword(unsigned char id, unsigned int start_address);
							
unsigned char  __stdcall dxl2_get_sync_read_data_byte(unsigned char id, unsigned int start_address);
unsigned short __stdcall dxl2_get_sync_read_data_word(unsigned char id, unsigned int start_address);
unsigned long  __stdcall dxl2_get_sync_read_data_dword(unsigned char id, unsigned int start_address);


//void __stdcall dxl2_sync_write(unsigned char param[], int param_length);
////
//////////////////////////////////////////////////////////////////
//
#ifdef __cplusplus
}
#endif
#endif