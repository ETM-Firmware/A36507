
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

typedef struct {
  unsigned char *header_ptr;
  unsigned char *data_ptr;
  unsigned int  header_length;
  unsigned int  data_length;
} ETMModbusTXData;

//void TCPmodbusSetIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1);
//void TCPmodbusSetRemoteIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1);

typedef struct {
  unsigned long remote_ip_addr;
  unsigned long ip_addr;

} IPCONFIG;



void TCPmodbus_init(IPCONFIG* ip_config);

void TCPmodbus_task(void);


#define MAX_RX_SIZE    48

#define MAX_TX_SIZE    800	 // ethernet header for TCP/modbus is 60 bytes



// DPARKER WHAT IS THIS FOR???
#define MODBUS_COMMAND_REFRESH_TOTAL     MODBUS_WR_EVENTS  

extern unsigned char         modbus_cmd_need_repeat;  


#endif
