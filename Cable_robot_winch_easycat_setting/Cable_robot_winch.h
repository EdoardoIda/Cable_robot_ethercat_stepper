#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project Cable_robot_winch.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	12
#define CUST_BYTE_NUM_IN	16
#define TOT_BYTE_NUM_ROUND_OUT	12
#define TOT_BYTE_NUM_ROUND_IN	16


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		int32_t     target_position;
		int32_t     target_speed;
		uint16_t    control_word;
		int16_t     target_torque;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int32_t     actual_position;
		int32_t     actual_speed;
		uint16_t    status_word;
		uint16_t    loadcell_value;
		int16_t     actual_torque;
		int16_t     actual_position_aux;
	}Cust;
} PROCBUFFER_IN;

#endif