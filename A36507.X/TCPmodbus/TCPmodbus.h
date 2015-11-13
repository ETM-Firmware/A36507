
/*********************************************************************
 *
 *  Main Application Entry Point and TCP/IP Stack Demo
 *  Module for Microchip TCP/IP Stack
 *   -Demonstrates how to call and use the Microchip TCP/IP stack
 *   -Reference: Microchip TCP/IP Stack Help (TCPIPStack Help.chm)1150

 *
 *********************************************************************/
#ifndef __TCP_MODBUS_H
#define __TCP_MODBUS_H

#include "P1395_CAN_MASTER.h"


//void TCPmodbusSetIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1);
//void TCPmodbusSetRemoteIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1);

typedef struct {
  unsigned long remote_ip_addr;
  unsigned long ip_addr;

} IPCONFIG;



void TCPmodbus_init(IPCONFIG* ip_config);

void TCPmodbus_task(void);

unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset);
unsigned int SendPulseData(unsigned char is_buffer_a);


//#define TEST_MODBUS	   1



#define MAX_TX_SIZE    800	 // ethernet header for TCP/modbus is 60 bytes
#define MAX_DATA_SIZE  640	// leave some room for modbus header

#define ETH_EVENT_SIZE  100

enum
{
	MODBUS_WR_HVLAMBDA = 1, 	
	MODBUS_WR_ION_PUMP,
	MODBUS_WR_AFC,
	MODBUS_WR_COOLING,
	MODBUS_WR_HTR_MAGNET,
	MODBUS_WR_GUN_DRIVER,
	MODBUS_WR_MAGNETRON_CURRENT,
	MODBUS_WR_PULSE_SYNC,
	MODBUS_WR_ETHERNET,
	MODBUS_WR_DEBUG_DATA,
	MODBUS_WR_EVENTS,	   /* 11 */
	
	MODBUS_WR_ONE_CAL_ENTRY,
	MODBUS_WR_PULSE_LOG,
	MODBUS_RD_COMMAND_DETAIL,


};

#define MODBUS_COMMAND_REFRESH_TOTAL     MODBUS_WR_EVENTS




#define MAX_CUSTOM_DATA_LENGTH  32


// STANDARD LOGGING DATA


typedef struct {
  unsigned int index ;                  // command index
  unsigned int data_2;
  unsigned int data_1;
  unsigned int data_0;
} ETMEthernetMessageFromGUI;


#define ETH_GUI_MESSAGE_BUFFER_SIZE   8
extern ETMEthernetMessageFromGUI eth_message_from_GUI[ ETH_GUI_MESSAGE_BUFFER_SIZE ];


typedef struct {
  unsigned int index ;                  // command index
  unsigned int scale;
  unsigned int offset;
} ETMEthernetCalToGUI;


#define ETH_CAL_TO_GUI_BUFFER_SIZE  8
extern ETMEthernetCalToGUI eth_cal_to_GUI[ ETH_CAL_TO_GUI_BUFFER_SIZE ];




extern ETMEthernetMessageFromGUI GetNextMessage(void);
/*
  index will be 0xFFFF if there is no message
*/

#endif
