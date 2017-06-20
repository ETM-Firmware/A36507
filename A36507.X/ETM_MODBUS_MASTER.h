
unsigned int etm_modbus_master_no_return_data;
#define NO_RETURN_DATA &etm_modbus_master_no_return_data

#define MODBUS_BUFFER_STATUS_EMPTY                 0
#define MODBUS_BUFFER_STATUS_QUEUED                1
#define MODBUS_BUFFER_STATUS_IN_PROCESS            2
#define MODBUS_BUFFER_STATUS_DONE                  3
#define MODBUS_BUFFER_STATUS_ERROR                 4


void ETMModbusMasterConfig(unsigned char uart_select,
			   unsigned char timer_select,
			   unsigned long baud_rate,
			   unsigned long fcy_clk);



unsigned char ETMModbusMasterSendMessage(unsigned char modbus_buffer,
					 unsigned char function_id,
					 unsigned char slave_address, 
					 unsigned int  register_address,
					 unsigned int  register_data,
					 unsigned int* data_ptr);

/*
  Loads the message into the selected register
  Will return one of the following

  The Folloing message_ids are supported
*/

#define ETM_MODBUS_MASTER_ERROR_UNKNOWN_MESSAGE_TYPE      1
#define ETM_MODBUS_MASTER_ERROR_BUFFER_NOT_VALID          2
#define ETM_MODBUS_MASTER_ERROR_BUFFER_BUSY               3
#define ETM_MODBUS_MASTER_BUFFER_LOADED                   0


#define MESSAGE_TYPE_READ_COILS                     1
#define MESSAGE_TYPE_READ_DISCRETE_INPUTS           2
#define MESSAGE_TYPE_READ_HOLDING_REGISTERS         3
#define MESSAGE_TYPE_READ_INPUT_REGISTERS           4

#define MESSAGE_TYPE_WRITE_SINGLE_COIL              5
#define MESSAGE_TYPE_WRITE_SINGLE_HOLDING_REGISTER  6




void ETMModbusMasterDoModbus(void);

void ETMModbusMasterUpdateCounterMilliSeconds(unsigned int milliseconds);

void ETMModbusMasterUpdateCounter10mS(void);

unsigned char ETMModbusMasterCheckMessageStatus(unsigned char modbus_buffer);

unsigned int ETMModbusMasterGetMessageReturn(unsigned char modbus_buffer);

void ETMModbusMasterRXISR(void);

void ETMModbusMasterTXISR(void);






#define ETM_MODBUS_MASTER_UART_1                    1
#define ETM_MODBUS_MASTER_UART_2                    2


#define ETM_MODBUS_MASTER_TIMER_1                   1
#define ETM_MODBUS_MASTER_TIMER_2                   2
#define ETM_MODBUS_MASTER_TIMER_3                   3
#define ETM_MODBUS_MASTER_TIMER_4                   4
#define ETM_MODBUS_MASTER_TIMER_5                   5





