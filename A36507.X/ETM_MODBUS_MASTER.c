#include "ETM_CRC.h"
#include "ETM_MODBUS_MASTER.h"
#include <xc.h>
#include <uart.h>

/*
  Our Modbus module talks to a single slave at a time
  
  There are 8 modbus message buffers
  
  Message buffer 0 is the highest priority, message buffer 7 is the lowest priority

  Messages consits of a transmit to a slave and response from a slave.

  After the response has been recieved, the Queued message with the highest priorty will start sending.
  
*/



// Local Functions
void MBEnableTX(void);
void MBDisableTX(void);
void MBEnableRX(void);
void MBDisableRX(void);

void MBTransmitMessage(unsigned char modbus_buffer);
void MBProcessMessage(unsigned char modbus_buffer);
void MBAbortMessage(unsigned char modbus_buffer);

unsigned int MBTransmitDone(void);
unsigned int MBTransmitWaitDone(void);
unsigned int MBCheckTimeout(void);
unsigned int MBGetTimer(void);
unsigned int MBCheckMessageReceived(void);
unsigned char ETMModbusGetNextMessageToSend(void);





typedef struct {
  unsigned char slave_address;  // Always Used
  unsigned char function_id;    // Always Used
  unsigned int  start_address;  // Always Used
  unsigned int  data_or_number; // Always Used
  unsigned int* data_ptr;       // Optional
  unsigned char status;
} TYPE_MODBUS_MASTER_BUFF;

// Do NOT MAKE THIS BIGGER THAN 255 as this is stored by a char
#define NUMBER_OF_MODBUS_BUFFERS  8
#define ALL_BUFFERS_EMPTY         NUMBER_OF_MODBUS_BUFFERS

TYPE_MODBUS_MASTER_BUFF modbus_message_buffer[NUMBER_OF_MODBUS_BUFFERS];



#define INPUT_BUFFER_SIZE  12
#define OUTPUT_BUFFER_SIZE  12

typedef struct {
  unsigned char state;
  unsigned char current_message;
  unsigned char message_index;
  
  unsigned char input_buffer[INPUT_BUFFER_SIZE];
  unsigned char input_buffer_index;
  
  unsigned char output_buffer[OUTPUT_BUFFER_SIZE];
  unsigned char output_buffer_index;

  unsigned char uart_port;
  unsigned char selected_timer;
  unsigned int  minimum_time_28_bits;

  unsigned int  last_byte_recieved_time;
  unsigned int  last_byte_recieved_counter;
  unsigned int  first_byte_recieved;
  unsigned int  counter_10_ms;

  unsigned int  transmit_time;
  unsigned int  transmit_count;

} TYPE_ETM_MODBUS_MASTER;

TYPE_ETM_MODBUS_MASTER etm_modbus_master_data;



#define MESSAGE_TYPE_READ               0
#define MESSAGE_TYPE_WRITE              1

#define STATE_IDLE               0
#define STATE_WAIT_TRANSMIT      1
#define STATE_TRANSMIT_ACTIVE    2  
#define STATE_WAIT_RESPONSE      3








unsigned char ETMModbusMasterSendMessage(unsigned char modbus_buffer,
					 unsigned char function_id,
					 unsigned char slave_address, 
					 unsigned int  register_address,
					 unsigned int  register_data,
					 unsigned int* data_ptr) {
  unsigned char type;

  // Check that it is valid message type
  if ((function_id >= 1) && (function_id <=4)) {
    type = MESSAGE_TYPE_READ;
  } else if ((function_id == 5) || (function_id == 6)) {
    type = MESSAGE_TYPE_WRITE;
  } else {
    return ETM_MODBUS_MMASTER_ERROR_UNKNOWN_MESSAGE_TYPE;
  }
  
  // Check that the modbus buffer is available
  if (modbus_buffer >= NUMBER_OF_MODBUS_BUFFERS) {
    return ETM_MODBUS_MASTER_ERROR_BUFFER_NOT_VALID;
  }
  if (modbus_message_buffer[modbus_buffer].status == MODBUS_BUFFER_STATUS_QUEUED) {
    return ETM_MODBUS_MASTER_ERROR_BUFFER_BUSY;
  }
  if (modbus_message_buffer[modbus_buffer].status == MODBUS_BUFFER_STATUS_IN_PROCESS) {
    return ETM_MODBUS_MASTER_ERROR_BUFFER_BUSY;
  }
  
  // Fill up the modbus message
  modbus_message_buffer[modbus_buffer].status         = MODBUS_BUFFER_STATUS_QUEUED;
  modbus_message_buffer[modbus_buffer].slave_address  = slave_address;
  modbus_message_buffer[modbus_buffer].function_id    = function_id;
  modbus_message_buffer[modbus_buffer].start_address  = register_address;
  modbus_message_buffer[modbus_buffer].data_or_number = register_data;
  modbus_message_buffer[modbus_buffer].data_ptr       = data_ptr;

  return ETM_MODBUS_MASTER_BUFFER_LOADED;
}



void ETMModbusMasterDoModbus(void) {

  switch (etm_modbus_master_data.state)
    {
    case STATE_IDLE:
      MBDisableTX();
      MBDisableRX();
      etm_modbus_master_data.current_message = ETMModbusGetNextMessageToSend();
      if (etm_modbus_master_data.current_message != ALL_BUFFERS_EMPTY) {
	etm_modbus_master_data.state = STATE_WAIT_TRANSMIT;
	etm_modbus_master_data.transmit_count = etm_modbus_master_data.counter_10_ms;
	etm_modbus_master_data.transmit_time = MBGetTimer();
      } 
      break;
      
    case STATE_WAIT_TRANSMIT:
      if (MBTransmitWaitDone()) {
	MBEnableTX();
	MBTransmitMessage(etm_modbus_master_data.current_message);
	etm_modbus_master_data.state = STATE_TRANSMIT_ACTIVE;
      } 

      if (MBCheckTimeout()) {
	MBAbortMessage(etm_modbus_master_data.current_message);
	etm_modbus_master_data.state = STATE_IDLE;
      }
      break;

    case STATE_TRANSMIT_ACTIVE:
      if (MBTransmitDone()) {
	MBDisableTX();
	MBEnableRX();
	etm_modbus_master_data.state = STATE_WAIT_RESPONSE;
      }
      
      if (MBCheckTimeout()) {
	MBAbortMessage(etm_modbus_master_data.current_message);
	etm_modbus_master_data.state = STATE_IDLE;
      }
      break;

    case STATE_WAIT_RESPONSE:
      if (MBCheckMessageReceived()) {
	MBProcessMessage(etm_modbus_master_data.current_message);
	etm_modbus_master_data.state = STATE_IDLE;
      }
      
      if (MBCheckTimeout()) {
	MBAbortMessage(etm_modbus_master_data.current_message);
	etm_modbus_master_data.state = STATE_IDLE;
      }
      break;
      
    default:
      etm_modbus_master_data.state = STATE_IDLE;
      break;
    }
}



void ETMModbusMasterUpdateCounter10mS(void) {
  etm_modbus_master_data.counter_10_ms++;
}

unsigned char ETMModbusMasterCheckMessageStatus(unsigned char modbus_buffer) {
  return modbus_message_buffer[modbus_buffer].status;
}

unsigned int ETMModbusMasterGetMessageReturn(unsigned char modbus_buffer) {
  return modbus_message_buffer[modbus_buffer].data_or_number;
}

/*
  Modbus works on one of the UARTS
*/


void ETMModbusMasterRXISR(void) {
  etm_modbus_master_data.last_byte_recieved_time = MBGetTimer();
  etm_modbus_master_data.last_byte_recieved_counter = etm_modbus_master_data.counter_10_ms;
  etm_modbus_master_data.first_byte_recieved = 1;
  
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    _U2RXIF = 0;
    while (U2STAbits.URXDA) {
      if (etm_modbus_master_data.input_buffer_index < INPUT_BUFFER_SIZE) {
	etm_modbus_master_data.input_buffer[etm_modbus_master_data.input_buffer_index] = U2RXREG;
      }
      etm_modbus_master_data.input_buffer_index++;
    }
#endif
  } else {
#ifdef U1TXREG
    _U1RXIF = 0;
    while (U1STAbits.URXDA) {
      if (etm_modbus_master_data.input_buffer_index < INPUT_BUFFER_SIZE) {
	etm_modbus_master_data.input_buffer[etm_modbus_master_data.input_buffer_index] = U1RXREG;
      }
      etm_modbus_master_data.input_buffer_index++;
    }
#endif    
  }
}


void ETMModbusMasterTXISR(void) {
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    _U2TXIF = 0;
    while ((!U2STAbits.UTXBF) && (etm_modbus_master_data.output_buffer_index < 8)) {
      U2TXREG = etm_modbus_master_data.output_buffer[etm_modbus_master_data.output_buffer_index];
      etm_modbus_master_data.output_buffer_index++;
    }
#endif
  } else {
#ifdef U1TXREG
    _U1TXIF = 0;
    while ((!U1STAbits.UTXBF) && (etm_modbus_master_data.output_buffer_index < 8)) {
      U1TXREG = etm_modbus_master_data.output_buffer[etm_modbus_master_data.output_buffer_index];
      etm_modbus_master_data.output_buffer_index++;
    }
#endif
  }
}












void MBAbortMessage(unsigned char modbus_buffer) {
  modbus_message_buffer[modbus_buffer].status = MODBUS_BUFFER_STATUS_ERROR;
}














unsigned int MBTransmitDone(void) {
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    if ((!U2STAbits.TRMT) && (etm_modbus_master_data.output_buffer_index >= 8)) {
      return 1;
    }
#endif
  } else {
#ifdef U1TXREG
    if ((!U1STAbits.TRMT) && (etm_modbus_master_data.output_buffer_index >= 8)) {
      return 1;
    }
#endif
  }
  return 0;
}

#define TIMEOUT_1_SECOND       100

unsigned int MBCheckTimeout(void) {
  if ((etm_modbus_master_data.counter_10_ms - etm_modbus_master_data.transmit_time) > TIMEOUT_1_SECOND) {
    return 1;
  }
  return 0;
}


unsigned int MBTransmitWaitDone(void) {
  unsigned int current_time;
  current_time = MBGetTimer();
  if ((current_time - etm_modbus_master_data.transmit_time) > etm_modbus_master_data.minimum_time_28_bits) {
    return 1;
  }

  if ((etm_modbus_master_data.counter_10_ms - etm_modbus_master_data.transmit_count) > 1) {
    return 1;
  }

  return 0;
}


unsigned char ETMModbusGetNextMessageToSend(void) {
  unsigned int index = 0;
  for (index = 0; index < NUMBER_OF_MODBUS_BUFFERS; index++) {
    if (modbus_message_buffer[index].status == MODBUS_BUFFER_STATUS_QUEUED) {
      break;
    }
  }
  return index;
}



void MBEnableTX(void) {
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    _U2TXIF = 0;
    _U2TXIE = 1;
    U2STAbits.UTXEN = 1;
#endif
  } else {
#ifdef U1TXREG
    _U1TXIF = 0;
    _U1TXIE = 1;
    U1STAbits.UTXEN = 1;
#endif
    
  }
}

void MBDisableTX(void) {
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    _U2TXIE = 0;
    U2STAbits.UTXEN = 0;
#endif
  } else {
#ifdef U1TXREG
    _U1TXIE = 0;
    U1STAbits.UTXEN = 0;
#endif
  }
}



void MBEnableRX(void) {
  unsigned char temp;

  // Prepare the receive register
  etm_modbus_master_data.input_buffer_index = 0;

  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2RXREG
    // Clear all data from the RX register
    while (U2STAbits.URXDA) {
      temp = U2RXREG;
    }
    _U2RXIF = 0;
    _U2RXIE = 1;
#endif
  } else {
#ifdef U1RXREG
    // Clear all data from the RX register
    while (U1STAbits.URXDA) {
      temp = U1RXREG;
    }
    _U1RXIF = 0;
    _U1RXIE = 1;
#endif
  }
}

void MBDisableRX(void) {
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2RXREG
    _U2RXIE = 0;
#endif
  } else {
#ifdef U1TXREG
    _U2RXIE = 1;
#endif
  }
}




void MBTransmitMessage(unsigned char message_index) {
  unsigned int crc;
  
  etm_modbus_master_data.output_buffer[0] = modbus_message_buffer[message_index].slave_address;
  etm_modbus_master_data.output_buffer[1] = modbus_message_buffer[message_index].function_id;
  *(unsigned int*)&etm_modbus_master_data.output_buffer[2] = modbus_message_buffer[message_index].start_address;
  *(unsigned int*)&etm_modbus_master_data.output_buffer[4] = modbus_message_buffer[message_index].data_or_number;
  crc = ETMCRCModbus(&etm_modbus_master_data.output_buffer[0],6);
  etm_modbus_master_data.output_buffer[6] = crc; // crc low byte
  etm_modbus_master_data.output_buffer[7] = (crc >> 8); // crc high byte
  
  
  if (etm_modbus_master_data.uart_port == ETM_MODBUS_MASTER_UART_2) {
#ifdef U2TXREG
    U2TXREG = etm_modbus_master_data.output_buffer[0];
    etm_modbus_master_data.output_buffer_index = 1;
#endif
  } else {
#ifdef U1TXREG
    U1TXREG = etm_modbus_master_data.output_buffer[0];
    etm_modbus_master_data.output_buffer_index = 1;
#endif
  }
}



  
  

unsigned int MBGetTimer(void) {
  switch (etm_modbus_master_data.selected_timer)
    {
    case ETM_MODBUS_MASTER_TIMER_1:
      return TMR1;
      break;

    case ETM_MODBUS_MASTER_TIMER_2:
      return TMR2;
      break;

    case ETM_MODBUS_MASTER_TIMER_3:
      return TMR3;
      break;

    case ETM_MODBUS_MASTER_TIMER_4:
      return TMR4;
      break;

    case ETM_MODBUS_MASTER_TIMER_5:
      return TMR5;
      break;

    default:
      return 0;
      break;
    }
}


unsigned int MBCheckMessageReceived(void) {
  unsigned int time_clear;
  
  time_clear = MBGetTimer();
  time_clear -= etm_modbus_master_data.last_byte_recieved_time;
  if (time_clear >= etm_modbus_master_data.minimum_time_28_bits) {
    return 1;
  }
  
  if ((etm_modbus_master_data.counter_10_ms - etm_modbus_master_data.last_byte_recieved_counter) > 1) {
    return 1;
  }
  
  return 0;
}





void MBProcessMessage(unsigned char modbus_buffer) {
  unsigned char length;
  unsigned int crc;
  // A message has been received
  

  // First check that the message is valid
  // How long should the message be??
  length = etm_modbus_master_data.input_buffer_index;
  // DPARKER - Confirm that the length is correct.

  // Check the CRC
  crc = ETMCRCModbus(&etm_modbus_master_data.input_buffer[0], (length-2));
  crc -=
  
  
  
  
  
  

}
