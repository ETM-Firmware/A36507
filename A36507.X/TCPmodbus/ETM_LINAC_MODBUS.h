#ifndef __ETM_LINAC_MODBUS_H
#define __ETM_LINAC_MODBUS_H


typedef struct {
  unsigned int index ;                  // command index
  unsigned int data_2;
  unsigned int data_1;
  unsigned int data_0;
} ETMEthernetMessageFromGUI;


#define ETH_GUI_MESSAGE_BUFFER_SIZE   8
extern ETMEthernetMessageFromGUI eth_message_from_GUI[ ETH_GUI_MESSAGE_BUFFER_SIZE ];


ETMEthernetMessageFromGUI GetNextMessage(void);

#define SEND_BUFFER_A            1
#define SEND_BUFFER_B            0
void SendPulseData(unsigned int buffer_select);
unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset);
//TCPmodbus_task();
//TCPmodbus_init(&ip_config);


void ETMLinacModbusUpdate(void);

//void ETMLinacModbusInitialize(unsigned long ip_address, unsigned long remote_ip_address);
void ETMLinacModbusInitialize(void);




#endif
