#include <xc.h>
#include "TCPmodbus.h"
#include "A36507.h"
#include "ETM_TICK.h"
#include "ETM_LINAC_MODBUS.h"

#include <string.h>
#include "ETM_IO_PORTS.h"  //DPARKER Fix this


//static unsigned char queue_buffer_room(unsigned char q_index);
//static unsigned char queue_is_empty(unsigned char q_index);


ETMEthernetMessageFromGUI GetNextMessage(void);
void InitModbusData(void);

static void queue_put_command(unsigned char * buffer_ptr);


unsigned int NewMessageInEventLog(void);
unsigned int EventLogMessageSize(void);

enum {
  MODBUS_WR_CYCLE_START,
  MODBUS_WR_HVLAMBDA, 	
  MODBUS_WR_ION_PUMP,
  MODBUS_WR_AFC,
  MODBUS_WR_COOLING,
  MODBUS_WR_HTR_MAGNET,
  MODBUS_WR_GUN_DRIVER,
  MODBUS_WR_MAGNETRON_CURRENT,
  MODBUS_WR_PULSE_SYNC,
  MODBUS_WR_ETHERNET,
  MODBUS_WR_DEBUG_DATA,
  MODBUS_WR_EVENTS,
  MODBUS_WR_CAL_DATA,
  MODBUS_WR_CYCLE_STOP,
  
  MODBUS_WR_PULSE_LOG,
  MODBUS_WR_,
  MODBUS_RD_COMMAND_DETAIL,
};


typedef struct {
  unsigned int index ;                  // command index
  unsigned int scale;
  unsigned int offset;
} ETMEthernetCalToGUI;




#define ETH_GUI_MESSAGE_BUFFER_SIZE   8
ETMEthernetMessageFromGUI    eth_message_from_GUI[ ETH_GUI_MESSAGE_BUFFER_SIZE ];


#define ETH_CAL_TO_GUI_BUFFER_SIZE  8
ETMEthernetCalToGUI          eth_cal_to_GUI[ ETH_CAL_TO_GUI_BUFFER_SIZE ];

static unsigned char         modbus_send_index = 0;
//static unsigned char         modbus_refresh_index = 1;
static unsigned char         modbus_command_request = 0;  /* how many commands from GUI */

static unsigned char         eth_message_from_GUI_put_index;
static unsigned char         eth_message_from_GUI_get_index;
static unsigned char         send_high_speed_data_buffer;  /* bit 0 high for buffer A, bit 1 high for buffer B */


unsigned long timer_write_holding_var;

unsigned char *data_ptr;

unsigned char buffer_header[MAX_RX_SIZE];


//ETMModbusTXData etm_modbus_tx_data;



/****************************************************************************
  Function:
    static void queue_put_command(ETMEthernetMessageFromGUI command)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
static void queue_put_command(unsigned char * buffer_ptr) {
  
  if (((eth_message_from_GUI_put_index + 1) & 0xF) == eth_message_from_GUI_get_index) {
    // The command buffer is full
    // Drop this message??
    // DPARKER what to do here
    return;
  }
  
  eth_message_from_GUI[eth_message_from_GUI_put_index].index = (*buffer_ptr << 8) | *(buffer_ptr + 1);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_2 = (*(buffer_ptr + 2) << 8) | *(buffer_ptr + 3);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_1 = (*(buffer_ptr + 4) << 8) | *(buffer_ptr + 5);
  eth_message_from_GUI[eth_message_from_GUI_put_index].data_0 = (*(buffer_ptr + 6) << 8) | *(buffer_ptr + 7);
  
  eth_message_from_GUI_put_index++;
  eth_message_from_GUI_put_index &= 0xF;
}


/****************************************************************************
  Function:
    ETMEthernetMessageFromGUI GetNextMessage(void)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
ETMEthernetMessageFromGUI GetNextMessage(void) {
  ETMEthernetMessageFromGUI command;
  
  if (eth_message_from_GUI_put_index == eth_message_from_GUI_get_index)  {
    // The message queue is empty
    command.index = 0xFFFF;
  } else {
    command = eth_message_from_GUI[eth_message_from_GUI_get_index]; 
    eth_message_from_GUI_get_index++;
    eth_message_from_GUI_get_index &= 0xF;
  }
  
  return (command);    
}



unsigned int NewMessageInEventLog(void) {
  if (event_log.gui_index == event_log.write_index) {
    return 0;
  }
  return 1;
}

unsigned int EventLogMessageSize(void) {
  unsigned int events_to_send = 0;

  if (event_log.gui_index > event_log.write_index) {
    events_to_send = 128 - event_log.gui_index; 
  } else {
    events_to_send = event_log.write_index - event_log.gui_index;
  }
  if (events_to_send >= 64) {
    // Max of 64 events per send
    events_to_send = 64;
  }	
  
  // Update the gui_index
  event_log.gui_index += events_to_send;
  event_log.gui_index &= 0x7F;

  return (events_to_send << 3);
}




void SendPulseData(unsigned int buffer_select) {
  if (buffer_select == SEND_BUFFER_A) {
    send_high_speed_data_buffer = 0x01;
  } else {
    send_high_speed_data_buffer = 0x02;
  }
}



#define SIZE_BOARD_MIRROR    104
#define SIZE_DEBUG_DATA      80
#define SIZE_HIGH_SPEED_DATA TBD


unsigned char scheduled_modbus_message_counter = 0;



unsigned char GetNextMessageType(void) {
  static unsigned char scheduled_modbus_message_counter = 0;

  scheduled_modbus_message_counter++;
  if (scheduled_modbus_message_counter >= MODBUS_WR_CYCLE_STOP) {
    scheduled_modbus_message_counter = MODBUS_WR_CYCLE_START;
    scheduled_modbus_message_counter++;
  }
  
  return scheduled_modbus_message_counter;
}







#define HEADER_LENGTH_CHAR 15

void BuildModbusHeader(unsigned int data_bytes, unsigned int data_id) {



  
}


void PrepareStandardMessage(ETMModbusTXData *tx_data, unsigned char data_type) {
  static unsigned pulse_index = 0;        // index for eash tracking
  static unsigned transaction_number = 0; // Index for each transaction


  tx_data->data_length = 0;
  tx_data->tx_ready = 0;
  
  tx_data->header_length = HEADER_LENGTH_CHAR;
  tx_data->header_ptr = buffer_header;
  

  switch (data_type)
    {
    case MODBUS_WR_HVLAMBDA:
      tx_data->data_ptr = (unsigned char *)&mirror_hv_lambda;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_ION_PUMP:
      tx_data->data_ptr = (unsigned char *)&mirror_ion_pump;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_AFC:
      tx_data->data_ptr = (unsigned char *)&mirror_afc;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
      
    case MODBUS_WR_COOLING:
      tx_data->data_ptr = (unsigned char *)&mirror_cooling;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_HTR_MAGNET:
      tx_data->data_ptr = (unsigned char *)&mirror_htr_mag;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;

    case MODBUS_WR_GUN_DRIVER:
      tx_data->data_ptr = (unsigned char *)&mirror_gun_drv;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_MAGNETRON_CURRENT:
      tx_data->data_ptr = (unsigned char *)&mirror_pulse_mon;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_PULSE_SYNC:
      tx_data->data_ptr = (unsigned char *)&mirror_pulse_sync;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_ETHERNET:
      tx_data->data_ptr = (unsigned char *)&local_data_ecb;
      tx_data->data_length = SIZE_BOARD_MIRROR;
      tx_data->tx_ready = 1;
      break;
    
    case MODBUS_WR_DEBUG_DATA:
      if (etm_can_active_debugging_board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
	tx_data->data_ptr = (unsigned char *)&debug_data_ecb;
      } else {
	tx_data->data_ptr = (unsigned char *)&debug_data_slave_mirror;
      }
      tx_data->data_length = SIZE_DEBUG_DATA;
      tx_data->tx_ready = 1;
      break;
      
    case MODBUS_WR_CAL_DATA:
      // FUTURE USE OF CALIBRATION DATA
      tx_data->data_length = 0;
      tx_data->tx_ready = 0;
      break;

    case MODBUS_WR_EVENTS:
      tx_data->tx_ready = 0;
      if (NewMessageInEventLog()) {
	tx_data->data_ptr = (unsigned char *)&event_log.event_data[event_log.gui_index];
	tx_data->data_length = EventLogMessageSize();
	tx_data->tx_ready = 1;
      }
      break;

    case MODBUS_RD_COMMAND_DETAIL:
      tx_data->data_ptr = (unsigned char *)&local_data_ecb;  // Dummy data location so that we don't crash the processor if this gets executed for some reason
      tx_data->data_length = 0;
      tx_data->tx_ready = 1;
      break;
    

    default: // move to the next for now, ignore some boards
      tx_data->data_length = 0;
      tx_data->data_ptr = (unsigned char *)&local_data_ecb; // DUMMY LOCATION
      tx_data->tx_ready = 0;
      break;
    }

  if (tx_data->tx_ready) {
 
    // DPARKER the header message still needs some clean up to make the specifiaction clear and uniform

   // Prepare the header message
    buffer_header[0] = (transaction_number >> 8) & 0xff;	    // transaction hi byte
    buffer_header[1] = transaction_number & 0xff;	    // transaction lo byte
    buffer_header[2] = 0;	                            // protocol hi - 0 
    buffer_header[3] = 0;	                            // protocol lo - 0
    buffer_header[4] = ((tx_data->data_length + HEADER_LENGTH_CHAR - 8) >> 8);              // This is the length of data remaining in the message
    buffer_header[5] = (tx_data->data_length + HEADER_LENGTH_CHAR - 8);                       // This is the length of data remaining in the message
    buffer_header[6] = data_type;                              // Unit Identifier - What Type of Data is this 
    if (data_type == MODBUS_RD_COMMAND_DETAIL) {
      buffer_header[7] = 0x3;                                // function code 0x03 = Read Multiple Holding Registers, 
    } else {
      buffer_header[7] = 0x10;                               // function code 0x10 = Write Multiple Holding Registers, 
    }
    // DATA STARTS HERE - HOW SHOULD THIS BE FORMATED FOR GENERIC ETM MESSAGES
    buffer_header[8] = 0;                                    // Reserved for pulse index High Word
    buffer_header[9] = 0;	                                   // Reserved for pulse index low word
    // This header data is not sent out
    buffer_header[10] = tx_data->data_length >> 9;                     // msg length in words hi
    buffer_header[11] = tx_data->data_length >> 1;                     // msg length in words lo
    buffer_header[12] = (tx_data->data_length +15) & 0xff;                   // data length in bytes // DPARKER is this used???
    buffer_header[13] = (pulse_index >> 8) & 0xff;           // pulse index high word
    buffer_header[14] = pulse_index & 0xff;                  // pulse index low word
    
    
    if (data_type == MODBUS_RD_COMMAND_DETAIL) {
      buffer_header[9] = MODBUS_RD_COMMAND_DETAIL;
      buffer_header[10] = 0;                     // msg length in words hi
      buffer_header[11] = 4;                     // msg length in words lo
      buffer_header[12] = 0;                   // data length in bytes // DPARKER is this used???
    }

    if (data_type == MODBUS_WR_PULSE_LOG) {
      pulse_index++;  // overflows at 65535
    }
    transaction_number++;
  }
}



ETMModbusTXData ETMModbusApplicationSpecificTXData(void) {
  ETMModbusTXData data_to_send;
  unsigned char send_message;

  // See if there are any high speed messages to be sent

  send_message = 0;
  if (send_high_speed_data_buffer) {
    /*
    total_bytes = BuildModbusOutputHighSpeedDataLog();
    if (send_high_speed_data_buffer & 0x01) {
      data_ptr = (unsigned char *)&high_speed_data_buffer_a[0];
    } else {
      data_ptr = (unsigned char *)&high_speed_data_buffer_b[0];
    } 
    msg_size_bytes = HIGH_SPEED_DATA_BUFFER_SIZE * sizeof(ETMCanHighSpeedData);
    */
    send_high_speed_data_buffer = 0;
  } else if (0) {
    // FUTURE Event log counter is greater than 32
  } else if (0) {
    // FUTURE Scope trace A is ready to send
  } else if (0) {
    // FUTURE Scope trace B is ready to send
  } else if (0) {
    // FUTURE Scope trace High Voltage is ready to send
  } else if (modbus_command_request) {
    modbus_send_index = MODBUS_RD_COMMAND_DETAIL;
    send_message = 1;
    modbus_command_request = 0;
  } else {
    // Execute regularly scheduled command - No need to check to see if they were recieved we will resend them again soon enough
    if (ETMTickRunOnceEveryNMilliseconds(100, &timer_write_holding_var)) {
      // 100ms has passed - Send the next Message
      modbus_send_index = GetNextMessageType();
      send_message = 1;
    }
  }


  if (send_message) { 
    PrepareStandardMessage(&data_to_send, modbus_send_index);
    if (modbus_send_index != MODBUS_WR_EVENTS) {
      //ETMTCPModbusWaitForResponse();  // Event log is not repeatable so no need to wait for response
    }
  }
  
  return data_to_send;
}




void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]) {
  
  if (data_RX[6] == modbus_send_index) {
    if (modbus_send_index == MODBUS_RD_COMMAND_DETAIL) {
      queue_put_command(&data_RX[9]);
    } else { 
      /* write commands return command count in the reference field */
      modbus_command_request = (data_RX[8] << 8) | data_RX[9];
    }
    
    etm_can_active_debugging_board_id = data_RX[10];
    switch (data_RX[10]) 
      {
      case MODBUS_WR_HVLAMBDA:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_HV_LAMBDA_BOARD;
	break;
	
      case MODBUS_WR_ION_PUMP:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_ION_PUMP_BOARD;
	break;
	
      case MODBUS_WR_AFC:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_AFC_CONTROL_BOARD;
	break;
	
      case MODBUS_WR_COOLING:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_COOLING_INTERFACE_BOARD;
	break;
	
      case MODBUS_WR_HTR_MAGNET:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_HEATER_MAGNET_BOARD;
	break;
	
      case MODBUS_WR_GUN_DRIVER:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_GUN_DRIVER_BOARD;
	break;
	
      case MODBUS_WR_MAGNETRON_CURRENT:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD;
	break;
	
      case MODBUS_WR_PULSE_SYNC:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_PULSE_SYNC_BOARD;
	break;
	
      case MODBUS_WR_ETHERNET:
	etm_can_active_debugging_board_id = ETM_CAN_ADDR_ETHERNET_BOARD;
	break;
      }
  } else {
    // does not match the sent command
    // DPARKER what to do here
  }
}



/****************************************************************************
  Function:
    static void InitModbusData(void)

  Description:
    This routine initializes modbus related data
 
  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
void InitModbusData(void)
{
 
  eth_message_from_GUI_put_index = 0;
  eth_message_from_GUI_get_index = 0;
 
}




/****************************************************************************
  Function:
		unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset)

  Input:
    pointer to data
    
  Description:
  Remarks:
  // This will add  a transmit message to the Send Calibration Data queue
  // It will return 0x0000 if the message was added to the queue or 0xFFFF if it was not (buffer full)
  ***************************************************************************/

unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset) {
  return (0xffff);
}


void ETMLinacModbusUpdate(void) {
  ETMTCPModbusTask();
}


void ETMLinacModbusInitialize(void) {
  IPCONFIG ip_config;
  TYPE_ENC28J60_CONFIG ENC28J60_config;
  
  ip_config.remote_ip_addr = 0x0F46A8C0;  // 192.168. 70. 15
  ip_config.ip_addr        = 0x6346A8C0;  // 192.168. 70. 99
  ip_config.mask           = 0x00FFFFFF;  // 255.255.255.  0
  ip_config.gate           = 0x00000000;  //   0.  0.  0.  0
  ip_config.dns            = 0x00000000;  //   0.  0.  0.  0
  ip_config.mac_addr[0]    = 0x00; 
  ip_config.mac_addr[1]    = 0x50; 
  ip_config.mac_addr[2]    = 0xC2; 
  ip_config.mac_addr[3]    = 0xB4; 
  ip_config.mac_addr[4]    = 0x20; 
  ip_config.mac_addr[5]    = 0x00;        //00-50-C2-B4-20-00 
  //ip_config.net_bios_name  = "ETMBoard Test";
  strcpy(ip_config.net_bios_name, "ETMBoard Test");

  ENC28J60_config.cable_select_pin = _PIN_RD15;
  ENC28J60_config.reset_pin = _PIN_RA15;
  ENC28J60_config.spi_port = TCPMODBUS_USE_SPI_PORT_1;


  ETMTickInitialize(FCY_CLK, ETM_TICK_USE_TIMER_1);  // DPARKER make this part of the configuration

  InitModbusData(); 

  ETMTCPModbusENC28J60Initialize(&ENC28J60_config);

  ETMTCPModbusInitialize(&ip_config);
}
