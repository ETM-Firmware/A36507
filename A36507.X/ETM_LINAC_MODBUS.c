#include <xc.h>
#include "TCPmodbus.h"
#include "A36507.h"
#include "ETM_TICK.h"
#include "ETM_LINAC_MODBUS.h"

#include <string.h>
#include "ETM_IO_PORTS.h"  //DPARKER Fix this


static unsigned char queue_buffer_room(unsigned char q_index);
static unsigned char queue_is_empty(unsigned char q_index);
static void queue_put_command(unsigned char * buffer_ptr);
ETMEthernetMessageFromGUI GetNextMessage(void);
void BuildModbusOutput_write_header(unsigned int total_bytes);
unsigned int BuildModbusOutputEventLog(unsigned char index);
unsigned int BuildModbusOutputGeneric(unsigned int msg_bytes,  unsigned char data_id);
unsigned int BuildModbusOutputHighSpeedDataLog(void);

unsigned int BuildModbusOutput_read_command(void);



void InitModbusData(void);




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
static unsigned int         transaction_number = 0;

static unsigned char         eth_message_from_GUI_put_index;
static unsigned char         eth_message_from_GUI_get_index;
static unsigned char         eth_cal_to_GUI_put_index;
static unsigned char         eth_cal_to_GUI_get_index;
static unsigned char         send_high_speed_data_buffer;  /* bit 0 high for buffer A, bit 1 high for buffer B */


unsigned long timer_write_holding_var;

unsigned char *data_ptr;

unsigned char buffer_header[MAX_RX_SIZE];


//ETMModbusTXData etm_modbus_tx_data;


#define QUEUE_MESSAGE_FROM_GUI  1
#define QUEUE_CAL_TO_GUI        2
/****************************************************************************
  Function:
    static unsigned char queue_buffer_room(q_index)

  Input:
    index to a queue
  Description:
 	return buffer room left for the queue
  Remarks:
    None
***************************************************************************/
static unsigned char queue_buffer_room(unsigned char q_index) {
  unsigned char room = 0;
  unsigned char put_index;
  unsigned char get_index;
  unsigned char size;
    
  switch (q_index) {
  case QUEUE_MESSAGE_FROM_GUI: 
    put_index = eth_message_from_GUI_put_index;
    get_index = eth_message_from_GUI_get_index;
    size = ETH_GUI_MESSAGE_BUFFER_SIZE;
    break;
  case QUEUE_CAL_TO_GUI:
    put_index = eth_cal_to_GUI_put_index;
    get_index = eth_cal_to_GUI_get_index;
    size = ETH_CAL_TO_GUI_BUFFER_SIZE;
    break;
  default:
    room = 0xff; // not defined
    break;
  }
    
  if (room != 0xff)
    {
      room  = (get_index - put_index - 1) & (size - 1);        
    }
  else
    room = 0;
        
  return (room);
}



/****************************************************************************
  Function:
    static unsigned char is_queue_empty(index)

  Input:
    index to a queue
  Description:
 	return length of the queue
  Remarks:
    None
***************************************************************************/
static unsigned char queue_is_empty(unsigned char q_index) {
  unsigned char is_empty = 0;
  unsigned char put_index;
  unsigned char get_index;
  unsigned char size;
    
  switch (q_index) {
  case QUEUE_MESSAGE_FROM_GUI: 
    put_index = eth_message_from_GUI_put_index;
    get_index = eth_message_from_GUI_get_index;
    size = ETH_GUI_MESSAGE_BUFFER_SIZE;
    break;
  case QUEUE_CAL_TO_GUI:
    put_index = eth_cal_to_GUI_put_index;
    get_index = eth_cal_to_GUI_get_index;
    size = ETH_CAL_TO_GUI_BUFFER_SIZE;
    break;
  default:
    is_empty = 0xff; // not defined
    break;
  }
    
  if (is_empty != 0xff)
    {
      if (put_index == get_index)
	is_empty = 1;
      // else default is_empty = 0 	    
    }
  else
    is_empty = 0; // not defined
        
  return (is_empty);
}


/****************************************************************************
  Function:
    static void queue_put_command(ETMEthernetMessageFromGUI command)

  Input:
    pointer to data
    
  Description:
  Remarks:
    None
***************************************************************************/
static void queue_put_command(unsigned char * buffer_ptr)
{
  if (queue_buffer_room(QUEUE_MESSAGE_FROM_GUI) > 0)
    {
      eth_message_from_GUI[eth_message_from_GUI_put_index].index = (*buffer_ptr << 8) | *(buffer_ptr + 1);
      eth_message_from_GUI[eth_message_from_GUI_put_index].data_2 = (*(buffer_ptr + 2) << 8) | *(buffer_ptr + 3);
      eth_message_from_GUI[eth_message_from_GUI_put_index].data_1 = (*(buffer_ptr + 4) << 8) | *(buffer_ptr + 5);
      eth_message_from_GUI[eth_message_from_GUI_put_index].data_0 = (*(buffer_ptr + 6) << 8) | *(buffer_ptr + 7);

      eth_message_from_GUI_put_index++;
      eth_message_from_GUI_put_index = eth_message_from_GUI_put_index & (ETH_GUI_MESSAGE_BUFFER_SIZE - 1);
    
    }
        
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
  
  if (queue_is_empty(QUEUE_MESSAGE_FROM_GUI) == 0) {
    command = eth_message_from_GUI[eth_message_from_GUI_get_index]; 
    eth_message_from_GUI_get_index++;
    eth_message_from_GUI_get_index = eth_message_from_GUI_get_index & (ETH_GUI_MESSAGE_BUFFER_SIZE - 1);
    
  } else {
    command.index = 0xffff;
  }
  return (command);    
}


void SendPulseData(unsigned int buffer_select) {
  if (buffer_select == SEND_BUFFER_A) {
    send_high_speed_data_buffer = 0x01;
  } else {
    send_high_speed_data_buffer = 0x02;
  }
}


/****************************************************************************
  Function:
    BuildModbusOutput_write_header(unsigned index)

  Description:
    Build the header for modbus command according to total bytes
 
	modbus header for write:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
	unit id (byte, 0xff), function code (byte, 0x10), reference number(word), data word count (word), 
	data byte count(byte), data bytes 
***************************************************************************/
void BuildModbusOutput_write_header(unsigned int total_bytes)
{
  buffer_header[0] = (transaction_number >> 8) & 0xff;	  // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	          // transaction lo byte
  buffer_header[2] = 0;	                                  // protocol hi 
  buffer_header[3] = 0;	                                  // protocol lo 
  buffer_header[4] = ((total_bytes + 7) >> 8) & 0xff;     // byte 4 and 5 for length
  buffer_header[5] = (total_bytes + 7) & 0xff;
  buffer_header[6] = modbus_send_index;	                  // unit Id 
  buffer_header[7] = 0x10;                                // function code 
  buffer_header[8] = 0;                                   // ref # hi
  buffer_header[9] = 0;	                                  // ref # lo
  buffer_header[10] = (total_bytes >> 9) & 0xff;          // data length in words hi, always 0, assume data length < 256
  buffer_header[11] = total_bytes >> 1;                   // data length in words lo
  buffer_header[12] = total_bytes & 0xff;                 // data length in bytes

}



/****************************************************************************
  Function:
    BuildModbusOutputEventLog(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
// DPARKER rewrite this to use point to data instead of data buffer
unsigned int BuildModbusOutputEventLog(unsigned char index)
{
  unsigned int x, i; 
  unsigned int total_bytes = 0;  // default: no cmd out 
  switch (index) // otherwise index is wrong, don't need send any cmd out
    {
    case MODBUS_WR_EVENTS: 
      // check if there are new events
      if (event_log.gui_index == event_log.write_index) break;  /* no new events */

      total_bytes = ((event_log.write_index - event_log.gui_index) & 0x7F) * sizeof(TYPE_EVENT);
	
      BuildModbusOutput_write_header(total_bytes);   

      // data starts at offset 13
      for (x = event_log.gui_index, i = 0; x != event_log.write_index; i++)
        {
            
	  if (i >= 64) break;  // max transfer 64 entries at one time
	  /*    
	  data_buffer[i * sizeof(TYPE_EVENT) + 13] = (event_log.event_data[x].event_number >> 8) & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 14] = event_log.event_data[x].event_number & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 15] = (event_log.event_data[x].event_time >> 24) & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 16] = (event_log.event_data[x].event_time >> 16) & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 17] = (event_log.event_data[x].event_time >> 8) & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 18] = event_log.event_data[x].event_time & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 19] = (event_log.event_data[x].event_id >> 8) & 0xff;
	  data_buffer[i * sizeof(TYPE_EVENT) + 20] = event_log.event_data[x].event_id & 0xff;
      */
	  x++;
	  x &= 0x7F;
        }

      event_log.gui_index = x; //  next entry

      total_bytes = i * sizeof(TYPE_EVENT) + 13;


       
      break;
      
    default:
      break;           
	     
    }
       
  return (total_bytes);

}


unsigned int BuildModbusOutputGeneric(unsigned int msg_bytes,  unsigned char data_id) {
  unsigned int total_bytes;

  total_bytes = msg_bytes + 15;

  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	         // transaction lo byte
  buffer_header[2] = 0;	                                 // protocol hi 
  buffer_header[3] = 0;	                                 // protocol lo 
  buffer_header[4] = ((msg_bytes + 7) >> 8);             // This is the length of data HB
  buffer_header[5] = msg_bytes + 7;                      // This is the length of data LB
  buffer_header[6] = data_id;	                         // Which Block of data are we sending 
  buffer_header[7] = 0x10;                               // function code 
  buffer_header[8] = 0;                                  // ref # hi
  buffer_header[9] = 0;	                                 // ref # lo
  buffer_header[10] = (msg_bytes >> 9);                  // msg length in words hi
  buffer_header[11] = msg_bytes >> 1;                    // msg length in words lo
  buffer_header[12] = total_bytes & 0xff;                // data length in bytes // DPARKER is this used???

  return total_bytes;
}

unsigned int BuildModbusOutputHighSpeedDataLog(void) {
  unsigned int data_bytes;
  static unsigned pulse_index = 0;  // index for eash tracking
  
  data_bytes = HIGH_SPEED_DATA_BUFFER_SIZE * sizeof(ETMCanHighSpeedData) + 2;

  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	         // transaction lo byte
  buffer_header[2] = 0;	                                 // protocol hi 
  buffer_header[3] = 0;	                                 // protocol lo 
  buffer_header[4] = ((data_bytes + 7) >> 8);              // This is the length of data HB
  buffer_header[5] = data_bytes + 7;                       // This is the length of data LB
  buffer_header[6] = MODBUS_WR_PULSE_LOG;                  // 
  buffer_header[7] = 0x10;                                 // function code 
  buffer_header[8] = 0;                                    // ref # hi
  buffer_header[9] = 0;	                                  // ref # lo
  buffer_header[10] = data_bytes >> 9;                     // msg length in words hi
  buffer_header[11] = data_bytes >> 1;                     // msg length in words lo
  buffer_header[12] = data_bytes & 0xff;                   // data length in bytes // DPARKER is this used???
  buffer_header[13] = (pulse_index >> 8) & 0xff;           // pulse index high word
  buffer_header[14] = pulse_index & 0xff;                  // pulse index low word
  
  pulse_index++;  // overflows at 65535
  
  return data_bytes + 13;

}

/****************************************************************************
  Function:
    BuildModbusOutput_read_command()

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
unsigned int BuildModbusOutput_read_command(void)
{ 
  /* modbus header for read:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
     unit id (byte, 0xff), function code (byte, 0x03), reference number(word), word count (byte) */

  buffer_header[0] = (transaction_number >> 8) & 0xff;	  // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	          // transaction lo byte
  buffer_header[2] = 0;	                                  // protocol hi 
  buffer_header[3] = 0;	                                  // protocol lo 
  buffer_header[4] = 0;
  buffer_header[5] = 6;
  buffer_header[6] = MODBUS_RD_COMMAND_DETAIL;
  buffer_header[7] = 0x3;                                 // function code 
  buffer_header[8] = 1;                                   // ref # hi
  buffer_header[9] = MODBUS_RD_COMMAND_DETAIL;            // ref # lo, redundant for now
  buffer_header[10] = 0;                                  // data length in words hi 
  buffer_header[11] = 4;                                  // data length in words lo
  buffer_header[12] = 0;                                  // data length in bytes // DPARKER is this used???
         
  return (15);	// always 15 bytes for read command
}

#define SIZE_BOARD_MIRROR    104
#define SIZE_DEBUG_DATA      80
#define SIZE_HIGH_SPEED_DATA TBD

unsigned int PrepareStandardMessage(void);

unsigned char scheduled_modbus_message_counter = 0;

unsigned int PrepareStandardMessage(void) {
  unsigned int message_size = 0;
  
  scheduled_modbus_message_counter++;
  if (scheduled_modbus_message_counter >= 11) {
    scheduled_modbus_message_counter = 0;
  }
  
  switch (scheduled_modbus_message_counter)
    {
    case 0:
      data_ptr = (unsigned char *)&mirror_hv_lambda;
      modbus_send_index = MODBUS_WR_HVLAMBDA;
      message_size = SIZE_BOARD_MIRROR;
      break;

    case 1:
      data_ptr = (unsigned char *)&mirror_ion_pump;
      modbus_send_index = MODBUS_WR_ION_PUMP;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 2:
      data_ptr = (unsigned char *)&mirror_afc;
      modbus_send_index = MODBUS_WR_AFC;
      message_size = SIZE_BOARD_MIRROR;
      break;
      
    case 3:
      data_ptr = (unsigned char *)&mirror_cooling;
      modbus_send_index = MODBUS_WR_COOLING;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 4:
      data_ptr = (unsigned char *)&mirror_htr_mag;
      modbus_send_index = MODBUS_WR_HTR_MAGNET;
      message_size = SIZE_BOARD_MIRROR;
      break;

    case 5:
      data_ptr = (unsigned char *)&mirror_gun_drv;
      modbus_send_index = MODBUS_WR_GUN_DRIVER;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 6:
      data_ptr = (unsigned char *)&mirror_pulse_mon;
      modbus_send_index = MODBUS_WR_MAGNETRON_CURRENT;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 7:
      data_ptr = (unsigned char *)&mirror_pulse_sync;
      modbus_send_index = MODBUS_WR_PULSE_SYNC;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 8:
      data_ptr = (unsigned char *)&local_data_ecb;
      modbus_send_index = MODBUS_WR_ETHERNET;
      message_size = SIZE_BOARD_MIRROR;
      break;
    
    case 9:
      if (etm_can_active_debugging_board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
	data_ptr = (unsigned char *)&debug_data_ecb;
      } else {
	data_ptr = (unsigned char *)&debug_data_slave_mirror;
      }
      modbus_send_index = MODBUS_WR_DEBUG_DATA;
      message_size = SIZE_DEBUG_DATA;
      break;
      
    case 10:
      // FUTURE USE OF CALIBRATION DATA
      break;

    case 11:
      // EVENT LOG IF NOT EMPTY
      // DPARKER ADD EVENT LOG IN HERE
      //total_bytes = BuildModbusOutputEventLog(modbus_send_index);  //DPARKER figure out how to build event log
      break;

    default: // move to the next for now, ignore some boards
      scheduled_modbus_message_counter = 0;
      break;
    }
  
  return message_size;
}


#define HEADER_LENGTH_CHAR 15

void BuildModbusHeader(unsigned int data_bytes, unsigned int data_id) {
  static unsigned pulse_index = 0;        // index for eash tracking
  static unsigned transaction_number = 0; // Index for each transaction


  buffer_header[0] = (transaction_number >> 8) & 0xff;	    // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	    // transaction lo byte
  buffer_header[2] = 0;	                            // protocol hi - 0 
  buffer_header[3] = 0;	                            // protocol lo - 0
  buffer_header[4] = ((data_bytes + HEADER_LENGTH_CHAR - 8) >> 8);              // This is the length of data remaining in the message
  buffer_header[5] = (data_bytes + HEADER_LENGTH_CHAR - 8);                       // This is the length of data remaining in the message
  buffer_header[6] = data_id;                              // Unit Identifier - What Type of Data is this 
  if (data_id == MODBUS_RD_COMMAND_DETAIL) {
    buffer_header[7] = 0x3;                                // function code 0x03 = Read Multiple Holding Registers, 
  } else {
    buffer_header[7] = 0x10;                               // function code 0x10 = Write Multiple Holding Registers, 
  }
  // DATA STARTS HERE - HOW SHOULD THIS BE FORMATED FOR GENERIC ETM MESSAGES
  buffer_header[8] = 0;                                    // Reserved for pulse index High Word
  buffer_header[9] = 0;	                                   // Reserved for pulse index low word
  // This header data is not sent out
  buffer_header[10] = data_bytes >> 9;                     // msg length in words hi
  buffer_header[11] = data_bytes >> 1;                     // msg length in words lo
  buffer_header[12] = (data_bytes +15) & 0xff;                   // data length in bytes // DPARKER is this used???
  buffer_header[13] = (pulse_index >> 8) & 0xff;           // pulse index high word
  buffer_header[14] = pulse_index & 0xff;                  // pulse index low word


  if (data_id == MODBUS_RD_COMMAND_DETAIL) {
    buffer_header[9] = MODBUS_RD_COMMAND_DETAIL;
    buffer_header[10] = 0;                     // msg length in words hi
    buffer_header[11] = 4;                     // msg length in words lo
    buffer_header[12] = 0;                   // data length in bytes // DPARKER is this used???
  }



  
  if (data_id == MODBUS_WR_PULSE_LOG) {
    pulse_index++;  // overflows at 65535
  }
  transaction_number++;
  
}


ETMModbusTXData ETMModbusApplicationSpecificTXData(void) {
  unsigned int total_bytes = 0;
  unsigned int msg_size_bytes;
  ETMModbusTXData data_to_send;
  unsigned int header_length;
  unsigned char send_data = 0;

  // See if there are any high speed messages to be sent
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
    msg_size_bytes = 0;
    modbus_command_request = 0;
    data_ptr = (unsigned char *)&local_data_ecb;
    send_data = 1;
  } else {
    // Execute regularly scheduled command - No need to check to see if they were recieved we will resend them again soon enough
    if (ETMTickRunOnceEveryNMilliseconds(100, &timer_write_holding_var)) {
      // 100ms has passed - Send the next Message
      //msg_size_bytes = PrepareStandardMessage();
      data_ptr = (unsigned char *)&local_data_ecb;
      modbus_send_index = MODBUS_WR_ETHERNET;
      msg_size_bytes = SIZE_BOARD_MIRROR;
      //total_bytes = BuildModbusOutputGeneric(msg_size_bytes, modbus_send_index);
      send_data = 1;
    }
  }

  header_length = 0;
  if (send_data) {
    header_length = HEADER_LENGTH_CHAR;
    BuildModbusHeader(msg_size_bytes, modbus_send_index);
    //transaction_number++; // don't care about overflow
    if (modbus_send_index != MODBUS_WR_EVENTS) {
      ETMTCPModbusWaitForResponse();  // Event log is not repeatable so no need to wait for response
    }
  }


  data_to_send.header_length = header_length;
  data_to_send.data_length = msg_size_bytes;
  data_to_send.header_ptr = buffer_header;
  data_to_send.data_ptr = data_ptr;
  
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
 
  eth_cal_to_GUI_put_index = 0;
  eth_cal_to_GUI_get_index = 0;
 
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

unsigned int SendCalibrationDataToGUI(unsigned int index, unsigned int scale, unsigned int offset)
{
    
  if (queue_buffer_room(QUEUE_CAL_TO_GUI) > 0)
    {
      eth_cal_to_GUI[eth_cal_to_GUI_put_index].index  = index;
      eth_cal_to_GUI[eth_cal_to_GUI_put_index].scale = scale;
      eth_cal_to_GUI[eth_cal_to_GUI_put_index].offset = offset;
        
      eth_cal_to_GUI_put_index++;
      eth_cal_to_GUI_put_index = eth_cal_to_GUI_put_index & (ETH_CAL_TO_GUI_BUFFER_SIZE - 1);
    
      return (0);
    }
  else
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