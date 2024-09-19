/*
 * rom_protocols.h
 *
 *  Created on: Aug 3, 2024
 *      Author: Pyae Soan Aung
 */

#ifndef _ROM_PROTOCOLS_H_
#define _ROM_PROTOCOLS_H_

/*****************************************************************************
 ** MCU or PC ?
 *****************************************************************************/
//#define MCU

/*****************************************************************************
 ** What PROTOCOL ? , General Macros .
 *****************************************************************************/
//define PROTOCOL_1
#define PROTOCOL_2
//#define PROTOCOL_3

#define ROM_UNUSED(x) ((void)(x))

/*****************************************************************************
 ** C Headers or C++ Headers ?
 *****************************************************************************/
#ifdef MCU
	//# include MCU specific headers
#else
	#include <cstdint>
	#include <cstring>
#endif

/*****************************************************************************
 ** Implementation for PROTOCOL_1
 *****************************************************************************/
#ifdef PROTOCOL_1
	typedef struct {
		int16_t right_actual_rpm;
		int16_t left_actual_rpm;
		int32_t right_encoder_count;
		int32_t left_encoder_count;
		int16_t volt;
		float ampere;
		int16_t e1234567;
		int16_t checksum;
	} MCU_DATA;

	typedef struct {
		int16_t right_desire_rpm;
		int16_t left_desire_rpm;
		int16_t e1234567;
		int16_t checksum;
	} PC_DATA;

	struct e1234567_status {
		_Bool estop_status;
		_Bool led1_status;
		_Bool led2_status;
		_Bool led3_status;
		_Bool led4_status;
		_Bool led5_status;
		_Bool led6_status;
		_Bool led7_status;
	};
/* PROTOCOL_1 */
// MCU2PC [ rar lar rec lec reserve e1234567 vol ampere crc_check "\r" "\n" ]
//checksum မှာ float တန်ဖိုးထည့်မပေါင်းပါ။
// PC2MCU [ rdr ldr reserve e1234567 crc_check "\r" "\n" ]
#endif

/*****************************************************************************
 ** Implementation for PROTOCOL_2
 *****************************************************************************/
#ifdef PROTOCOL_2

typedef struct {
		int16_t right_actual_rpm;
		int16_t left_actual_rpm;
		int32_t right_encoder_count;
		int32_t left_encoder_count;
		int16_t volt;
		float 	ampere;
		int16_t estop;
		int16_t led1;
		int16_t led2;
		int16_t led3;
		int16_t led4;
		int16_t bump1;
		float   cliff;
		float   ir;
		uint16_t   imu_z_vel;
		uint16_t   imu_z_rotate;
		int16_t reserved1;
		int16_t reserved2;
		int16_t checksum;
	} MCU_DATA;

	typedef struct {
		int16_t right_desire_rpm;
		int16_t left_desire_rpm;
		int16_t estop;
		int16_t led1;
		int16_t led2;
		int16_t led3;
		int16_t led4;
		int16_t checksum;
	} PC_DATA;
short calculateChecksumForMcuData(MCU_DATA *data) //passed by pointer
{
	int64_t tmp = (int64_t) (data->right_actual_rpm+data->left_actual_rpm+
					         data->right_encoder_count+data->left_encoder_count+
					         data->volt+data->estop+data->led1+data->led2+
					         data->led3+data->led4+data->bump1+data->reserved1+
					         data->reserved2);
	short tmp2 = (short)((tmp >> 8) & (0x000000000000ffff));
	return tmp2;
}
short calculateChecksumForPcData(PC_DATA *data)  //passed by pointer
{
	short tmp = (short)(data->right_desire_rpm+data->left_desire_rpm+
							data->estop+data->led1+data->led2+data->led3+
							data->led4);
	return tmp;
}
/* PROTOCOL_2 */
// MCU2PC
// [ rar lar rec lec rdr ldr  vol ampere estop led1 led2 led3 led4 bump1 cliff  ir  imu_z_vel imu_z_rotat reserved1 reserved2 checksum "\r\n"  ]
// [  2   2   4   4   2   2   2     4f     2     2     2    2    2    2   4f    4f     4f       4f          2         2         2        2 ]  =  58 Bytes
// checksum မှာ float တန်ဖိုးထည့်မပေါင်းပါ။
// PC2MCU
// [ rdr ldr estop led1 led2 led3 led4 checksum "\r\n" ]
// [  2   2    2     2    2    2   2      2        2   ] = 18 Bytes

#endif

/*****************************************************************************
 ** Implementation for PROTOCOL_3
 *****************************************************************************/
#ifdef PROTOCOL_3
#endif

#endif /* _ROM_PROTOCOLS_H_ */
