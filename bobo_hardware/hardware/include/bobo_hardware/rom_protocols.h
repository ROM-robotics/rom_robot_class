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
#define PROTOCOL_1
//#define PROTOCOL_2
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
// PC2MCU [ rdr ldr reserve e1234567 crc_check "\r" "\n" ]
#endif

/*****************************************************************************
 ** Implementation for PROTOCOL_2
 *****************************************************************************/
#ifdef PROTOCOL_2

/* PROTOCOL_2 */
// [ rar lar rec lec rdr ldr e1234567 vol ampere bump cliff    ir  sound crc_check "\r" "\n"  ]   
// [  2   2   4   4   2   2     2      2     4    x2     x2    x2    x2       2       1    1  ]  =  B
#endif

/*****************************************************************************
 ** Implementation for PROTOCOL_3
 *****************************************************************************/
#ifdef PROTOCOL_3
#endif

#endif /* _ROM_PROTOCOLS_H_ */