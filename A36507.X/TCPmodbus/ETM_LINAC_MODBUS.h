#ifndef __ETM_LINAC_MODBUS_H
#define __ETM_LINAC_MODBUS_H






TCPmodbus_task();
TCPmodbus_init(&ip_config);


void ETMLinacModbusUpdate(void);

void ETMLinacModbusInitialize(unsigned long ip_address, unsigned long remote_ip_address);




#endif
