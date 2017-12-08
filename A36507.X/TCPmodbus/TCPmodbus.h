#ifndef __TCP_MODBUS_H
#define __TCP_MODBUS_H

typedef struct {
  unsigned long remote_ip_addr;
  unsigned long ip_addr;
  unsigned long mask;
  unsigned long gate;
  unsigned long dns;
  unsigned char mac_addr[6];
  char          net_bios_name[16];
} IPCONFIG;


typedef struct {
  unsigned long cable_select_pin;
  unsigned long reset_pin;
  unsigned int  spi_port;
} TYPE_ENC28J60_CONFIG;


typedef struct {
  unsigned char header_data[24];        // Max header size is 24 bytes
  unsigned char *data_ptr;
  unsigned char tx_ready;
  unsigned char  header_length;
  unsigned int  data_length;
} ETMModbusTXData;

#define TCPMODBUS_USE_SPI_PORT_1         1
#define TCPMODBUS_USE_SPI_PORT_2         2

#define MAX_RX_SIZE    48                // This is the maximum size of recieved data (including header)

void ETMModbusApplicationSpecificTXData(ETMModbusTXData* tx_data_to_send);
//ETMModbusTXData ETMModbusApplicationSpecificTXData(void);
/*
  This function must be defined in the user application file.
  It is used to generate the TX data

  Every time the "ETMTCPModbusTask" is called it will run through the TCP client
  If the client is ready to send a message it will call this function
  If the header length is greater than zero and there is available space in the socket, the data will be transmitted

  If the header lenght is zero (indicating no message is ready to send) or there is not enough space available in the socket,
  No message will be sent durring this cycle of "ETMTCPModbusTask"
*/


void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]);
/*
  This function must be defined in the user application file
  It is used to process the recieved data

  When a message is recieved durring a "ETMTCPModbusTask" cycle, the TCP Client will call this function with the recieved data
  It is the responsiblity of the Application file to process that data.

*/


void ETMTCPModbusENC28J60Initialize(TYPE_ENC28J60_CONFIG* ENC28J60_config);
/*
  This is called to initialize the ENC28J60 hardware
*/


void ETMTCPModbusInitialize(IPCONFIG* ip_config);
/*
  This is called to initialize the TCP Modbus module
*/


void ETMTCPModbusTask(void);
/*
  This must be called occassionaly to execute the TCP client state machine
*/



void ETMTCPModbusWaitForResponse(void);
/*
  This function is called to set flag that indicates a message has been sent
*/


unsigned char ETMTCPModbusWaitingForResponse(void);
/*
  This flag will return true until the response has been recieved
*/


#define ERROR_COUNT_SM_PROCESS_RESPONSE_TIMEOUT 0
#define ERROR_COUNT_SM_SOCKET_OBTAINED_TIMEOUT  1
#define ERROR_SM_PROCESS_RESPONSE_TIMEOUT_ID    2
unsigned int ETMTCPModbusGetErrorInfo(unsigned char error);


#endif
