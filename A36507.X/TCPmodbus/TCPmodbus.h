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
  unsigned char *header_ptr;
  unsigned char *data_ptr;
  unsigned int  header_length;
  unsigned int  data_length;
} ETMModbusTXData;




// DPARKER - why can't this be done, Why does the same variable need to be called out in TCPmodbus.c and ETM_LINAC_Modbus.c
//extern ETMModbusTXData ETMModbusApplicationSpecificTXData(void);
//extern ETMModbusApplicationSpecificRXData(unsigned char data_RX[]);



void TCPmodbus_init(IPCONFIG* ip_config);
void TCPmodbus_task(void);


#define MAX_RX_SIZE    48

extern unsigned char         modbus_cmd_need_repeat;  


#endif
