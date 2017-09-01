/*********************************************************************
 *
 *  Generic TCP Client Example Application
 *  Module for Microchip TCP/IP Stack
 *   -Implements an example HTTP client and should be used as a basis 
 *	  for creating new TCP client applications
 *	 -Reference: None.  Hopefully AN833 in the future.
 *
 *********************************************************************
 * FileName:        GenericTCPClient.c
 * Dependencies:    TCP, DNS, ARP, Tick
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder     8/01/06	Original
 ********************************************************************/
#define __GENERICTCPCLIENT_C

#define THIS_IS_STACK_APPLICATION

#define IS_MODBUS_MAIN  1


#include "TCPIPStack/TCPIPStack/TCPIPConfig.h"
#include "TCPIPStack/TCPIPStack/TCPIP.h"
#include "TCPIPStack/TCPIPStack/ETM_TICK.h"

#include <p30F6014a.h>
//#include "TCPmodbus.h"
#include "A36507.h"




typedef struct {
  unsigned char *header_ptr;
  unsigned char *data_ptr;
  unsigned int  header_length;
  unsigned int  data_length;
} ETMModbusTXData;


ETMModbusTXData ETMModbusApplicationSpecificTXData(void);
/*
  This returns 
  * header ptr
  * data ptr
  * header length
  * data length
*/

void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]);

unsigned int BuildModbusOutputGeneric(unsigned int msg_bytes,  unsigned char unit_id);

unsigned long led_flash_holding_var;
unsigned long timer_write_holding_var;

unsigned char buffer_header[MAX_RX_SIZE];

unsigned char *data_ptr;
unsigned int header_length;


// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
static unsigned short wOriginalAppConfigChecksum;    // Checksum of the ROM defaults for AppConfig

#define LED_PUT(a)	do{unsigned char vTemp = (a); LEDOP_IO = vTemp&0x1; LEDA_IO = vTemp&0x4; LEDB_IO = vTemp&0x2;} while(0)

void GenericTCPClient(void);
void InitModbusData(void);


ETMEthernetMessageFromGUI    eth_message_from_GUI[ ETH_GUI_MESSAGE_BUFFER_SIZE ];
ETMEthernetCalToGUI          eth_cal_to_GUI[ ETH_CAL_TO_GUI_BUFFER_SIZE ];



//static BYTE         data_buffer[MAX_TX_SIZE];
static BYTE         modbus_send_index = 0;

static BYTE         modbus_refresh_index = 0;
static BYTE         modbus_command_request = 0;  /* how many commands from GUI */

//static BYTE         super_user_mode = 0;
static WORD         transaction_number = 0;

static BYTE         modbus_cmd_need_repeat = 0;  


static BYTE         eth_message_from_GUI_put_index;
static BYTE         eth_message_from_GUI_get_index;

static BYTE         eth_cal_to_GUI_put_index;
static BYTE         eth_cal_to_GUI_get_index;

static BYTE         send_high_speed_data_buffer;  /* bit 0 high for buffer A, bit 1 high for buffer B */


#define QUEUE_MESSAGE_FROM_GUI  1
#define QUEUE_CAL_TO_GUI        2
/****************************************************************************
  Function:
    static BYTE queue_buffer_room(q_index)

  Input:
    index to a queue
  Description:
 	return buffer room left for the queue
  Remarks:
    None
***************************************************************************/
static BYTE queue_buffer_room(BYTE q_index)
{
  BYTE room = 0;
  BYTE put_index;
  BYTE get_index;
  BYTE size;
    
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
    static BYTE is_queue_empty(index)

  Input:
    index to a queue
  Description:
 	return length of the queue
  Remarks:
    None
***************************************************************************/
static BYTE queue_is_empty(BYTE q_index)
{
  BYTE is_empty = 0;
  BYTE put_index;
  BYTE get_index;
  BYTE size;
    
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
static void queue_put_command(BYTE * buffer_ptr)
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
ETMEthernetMessageFromGUI GetNextMessage(void)
{
  ETMEthernetMessageFromGUI command;
    
  if (queue_is_empty(QUEUE_MESSAGE_FROM_GUI) == 0)
    {
      command = eth_message_from_GUI[eth_message_from_GUI_get_index]; 
      eth_message_from_GUI_get_index++;
      eth_message_from_GUI_get_index = eth_message_from_GUI_get_index & (ETH_GUI_MESSAGE_BUFFER_SIZE - 1);
    
    }
  else
    command.index = 0xffff;
    
  return (command);    
        
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
/****************************************************************************
  Function:
		unsigned int SendPulseData(unsigned char buffer_a)

  Input:
        0: buffer_b, non-0: buffer_a
    
  Description:
  Remarks:
  // This will change a flag to indicate pulse data ready
  // It will return 0x0000 if previous data was sent,  or 0xFFFF if it was not (buffer full)
  ***************************************************************************/
void SendPulseData(unsigned int buffer_select) {
  if (buffer_select == SEND_BUFFER_A) {
    send_high_speed_data_buffer = 0x01;
  } else {
    send_high_speed_data_buffer = 0x02;
  }
}
/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
***************************************************************************/
static void InitializeBoard(void)
{    
  // LEDs
  LEDA_TRIS = 0;
  LEDB_TRIS = 0;
  LEDOP_TRIS = 0;
  LED_PUT(0x00);
    


  // UART
  // Deassert all chip select lines so there isn't any problem with 
  // initialization order.  Ex: When ENC28J60 is on SPI2 with Explorer 16, 
  // MAX3232 ROUT2 pin will drive RF12/U2CTS ENC28J60 CS line asserted, 
  // preventing proper 25LC256 EEPROM operation.
#if defined(ENC_CS_TRIS)
  ENC_CS_IO = 1;
  ENC_CS_TRIS = 0;
#endif
}

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
// MAC Address Serialization using a MPLAB PM3 Programmer and 
// Serialized Quick Turn Programming (SQTP). 
// The advantage of using SQTP for programming the MAC Address is it
// allows you to auto-increment the MAC address without recompiling 
// the code for each unit.  To use SQTP, the MAC address must be fixed
// at a specific location in program memory.  Uncomment these two pragmas
// that locate the MAC address at 0x1FFF0.  Syntax below is for MPLAB C 
// Compiler for PIC18 MCUs. Syntax will vary for other compilers.
//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(IPCONFIG* ip_config)
{
    
  // Start out zeroing all AppConfig bytes to ensure all fields are 
  // deterministic for checksum generation
  memset((void*)&AppConfig, 0x00, sizeof(AppConfig));
        
  AppConfig.Flags.bIsDHCPEnabled = 0;
  AppConfig.Flags.bInConfigMode = 0;
  memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
  //        {
  //            _prog_addressT MACAddressAddress;
  //            MACAddressAddress.next = 0x157F8;
  //            _memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
  //        }
  
  AppConfig.MyIPAddr.Val = ip_config->ip_addr;
  //AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
  AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
  AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
  AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
  AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
  AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
  AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;
    
  AppConfig.MyRemIPAddr.Val = ip_config->remote_ip_addr;
  //AppConfig.MyRemIPAddr.Val = MY_DEFAULT_REM_IP_ADDR_BYTE1 | MY_DEFAULT_REM_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_REM_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_REM_IP_ADDR_BYTE4<<24ul;
    
    
  // Load the default NetBIOS Host Name
  memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
  FormatNetBIOSName(AppConfig.NetBIOSName);
    

  // Compute the checksum of the AppConfig defaults as loaded from ROM
  wOriginalAppConfigChecksum = CalcIPChecksum((BYTE*)&AppConfig, sizeof(AppConfig));
}

void TCPmodbusSetIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1) {
  unsigned long temp;
  temp = byte4;
  temp <<= 24;
  AppConfig.MyIPAddr.Val = temp;
  
  temp = byte3;
  temp <<= 16;
  AppConfig.MyIPAddr.Val |= temp;
  
  temp = byte2;
  temp <<= 8;
  AppConfig.MyIPAddr.Val |= temp;
  
  AppConfig.MyIPAddr.Val |= byte1;
}


void TCPmodbusSetRemoteIPAddress(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1) {
  unsigned long temp;
  temp = byte4;
  temp <<= 24;
  AppConfig.MyRemIPAddr.Val = temp;
  
  temp = byte3;
  temp <<= 16;
  AppConfig.MyRemIPAddr.Val |= temp;
  
  temp = byte2;
  temp <<= 8;
  AppConfig.MyRemIPAddr.Val |= temp;
  
  AppConfig.MyRemIPAddr.Val |= byte1;
}

//
// called once for initilization.
//
void TCPmodbus_init(IPCONFIG* ip_config)
{
  // Initialize application specific hardware
  InitializeBoard();


  ETMTickInitialize(20000000, ETM_TICK_USE_TIMER_1);  // DPARKER make this part of the configuration


  // Initialize Stack and application related NV variables into AppConfig.
  InitAppConfig(ip_config);
    
  InitModbusData(); 

  // Initialize core stack layers (MAC, ARP, TCP, UDP) and
  // application modules (HTTP, SNMP, etc.)
  StackInit();


}
//
// Need to call this task periodically
//
void TCPmodbus_task(void)
{

  // Blink LED0 (right most one) every second.
  if(ETMTickRunOnceEveryNMilliseconds(500, &led_flash_holding_var)) {
    LEDOP_IO ^= 1; // DPARKER fix this horribly unsafe operation
  }

  // This task performs normal stack task including checking
  // for incoming packet, type of packet and calling
  // appropriate stack entity to process it.
  StackTask();
        
  
  // This tasks invokes each of the core stack application tasks
  // StackApplications();  // don't need

 
  GenericTCPClient();

  //  ExecuteCommands();

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
    BuildModbusOutput_write_header(unsigned index)

  Description:
    Build the header for modbus command according to total bytes
 
	modbus header for write:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
	unit id (byte, 0xff), function code (byte, 0x10), reference number(word), data word count (word), 
	data byte count(byte), data bytes 
***************************************************************************/
void BuildModbusOutput_write_header(unsigned int total_bytes)
{
  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	 // transaction lo byte
  buffer_header[2] = 0;	// protocol hi 
  buffer_header[3] = 0;	// protocol lo 
  // byte 4 and 5 for length
  buffer_header[4] = ((total_bytes + 7) >> 8) & 0xff;
  buffer_header[5] = (total_bytes + 7) & 0xff;
  buffer_header[6] = modbus_send_index;	// unit Id 

  buffer_header[7] = 0x10; // function code 
  buffer_header[8] = 0;   // ref # hi
  buffer_header[9] = 0;	  // ref # lo

  buffer_header[10] = (total_bytes >> 9) & 0xff;  // data length in words hi, always 0, assume data length < 256
  buffer_header[11] = total_bytes >> 1;     // data length in words lo
  buffer_header[12] = total_bytes & 0xff;   // data length in bytes

}
/****************************************************************************
  Function:
    BuildModbusOutput_write_commands(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/
// DPARKER rewrite this to use point to data instead of data buffer
WORD BuildModbusOutput_write_commands(unsigned char index)
{
  WORD x, i; 
  WORD total_bytes = 0;  // default: no cmd out 
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


unsigned int BuildModbusOutputGeneric(unsigned int msg_bytes,  unsigned char unit_id) {
  unsigned int total_bytes;

  header_length = 13;
  total_bytes = msg_bytes + 13;

  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	         // transaction lo byte
  buffer_header[2] = 0;	                                 // protocol hi 
  buffer_header[3] = 0;	                                 // protocol lo 
  buffer_header[4] = ((msg_bytes + 7) >> 8);               // This is the length of data HB
  buffer_header[5] = msg_bytes + 7;                        // This is the length of data LB
  buffer_header[6] = unit_id;	                         // 
  buffer_header[7] = 0x10;                                 // function code 
  buffer_header[8] = 0;                                    // ref # hi
  buffer_header[9] = 0;	                                 // ref # lo
  buffer_header[10] = (msg_bytes >> 9);                    // msg length in words hi
  buffer_header[11] = msg_bytes >> 1;                      // msg length in words lo
  buffer_header[12] = total_bytes & 0xff;                  // data length in bytes // DPARKER is this used???

  return total_bytes;
}

unsigned int BuildModbusOutputHighSpeedDataLog(void) {
  unsigned int data_bytes;
  static unsigned pulse_index = 0;  // index for eash tracking
  
  data_bytes = HIGH_SPEED_DATA_BUFFER_SIZE * sizeof(ETMCanHighSpeedData) + 2;

  header_length = 15;
  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	         // transaction lo byte
  buffer_header[2] = 0;	                                 // protocol hi 
  buffer_header[3] = 0;	                                 // protocol lo 
  buffer_header[4] = ((data_bytes + 7) >> 8);              // This is the length of data HB
  buffer_header[5] = data_bytes + 7;                       // This is the length of data LB
  buffer_header[6] = MODBUS_WR_PULSE_LOG;                  // 
  buffer_header[7] = 0x10;                                 // function code 
  buffer_header[8] = 0;                                    // ref # hi
  buffer_header[9] = 0;	                                 // ref # lo
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
WORD BuildModbusOutput_read_command(BYTE index, BYTE byte_count)
{ 
  /* modbus header for read:  transaction ID(word), protocol ID(word, 0x0000), length(word, bytes to follow), 
     unit id (byte, 0xff), function code (byte, 0x03), reference number(word), word count (byte) */

  header_length = 12;
  buffer_header[0] = (transaction_number >> 8) & 0xff;	 // transaction hi byte
  buffer_header[1] = transaction_number & 0xff;	 // transaction lo byte
  buffer_header[2] = 0;	// protocol hi 
  buffer_header[3] = 0;	// protocol lo 
  // fill the byte length    
  buffer_header[4] = 0;
  buffer_header[5] = 6;
  buffer_header[6] = index;	// unit Id 

  buffer_header[7] = 0x3; // function code 
  buffer_header[8] = 1;  // ref # hi
  buffer_header[9] = index;  // ref # lo, redundant for now

  buffer_header[10] = 0;  // data length in words hi 
  buffer_header[11] = byte_count >> 1;  // data length in words lo
         
              
  return (12);	// always 12 bytes for read command

}
/****************************************************************************
  Function:
    BuildModbusOutput(void)

  Description:
    Build modbus command, return 0 if we don't want to send anything
 
***************************************************************************/

#define SIZE_BOARD_MIRROR    104
#define SIZE_DEBUG_DATA      80
#define SIZE_HIGH_SPEED_DATA TBD


WORD BuildModbusOutput(void) {
  WORD total_bytes = 0;  // default: no cmd out
  //unsigned char *tx_ptr = 0;
  unsigned int msg_size_bytes;

  header_length = 0;
  
  if (ETMTickRunOnceEveryNMilliseconds(100, &timer_write_holding_var)) {
    if (!modbus_cmd_need_repeat) {
      modbus_refresh_index++;
      if (modbus_refresh_index > MODBUS_COMMAND_REFRESH_TOTAL) modbus_refresh_index = 1;	 // starts from 1
    }
	      
    modbus_send_index = modbus_refresh_index;
    

    if (modbus_send_index >= MODBUS_WR_HVLAMBDA && modbus_send_index <= MODBUS_WR_DEBUG_DATA) {  // write info to the GUI
      switch (modbus_send_index)
	{
	case MODBUS_WR_HVLAMBDA:
	  data_ptr = (unsigned char *)&mirror_hv_lambda;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_ION_PUMP:
	  data_ptr = (unsigned char *)&mirror_ion_pump;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_AFC:
	  data_ptr = (unsigned char *)&mirror_afc;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_COOLING:
	  data_ptr = (unsigned char *)&mirror_cooling;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_HTR_MAGNET:
	  data_ptr = (unsigned char *)&mirror_htr_mag;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_GUN_DRIVER:
	  data_ptr = (unsigned char *)&mirror_gun_drv;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_MAGNETRON_CURRENT:
	  data_ptr = (unsigned char *)&mirror_pulse_mon;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_PULSE_SYNC:
	  data_ptr = (unsigned char *)&mirror_pulse_sync;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_ETHERNET:
	  data_ptr = (unsigned char *)&local_data_ecb;
	  msg_size_bytes = SIZE_BOARD_MIRROR;
	  break;
	case MODBUS_WR_DEBUG_DATA:
	  if (etm_can_active_debugging_board_id == ETM_CAN_ADDR_ETHERNET_BOARD) {
	    data_ptr = (unsigned char *)&debug_data_ecb;
	  } else {
	    data_ptr = (unsigned char *)&debug_data_slave_mirror;
	  }
	  msg_size_bytes = SIZE_DEBUG_DATA;
	  break;

	default: // move to the next for now, ignore some boards
	  break;
	}
      total_bytes = BuildModbusOutputGeneric(msg_size_bytes, modbus_send_index);
    } else {	 
      // special command for rd or write info
      switch (modbus_send_index)
	{
	case MODBUS_WR_EVENTS:
	  //total_bytes = BuildModbusOutput_write_commands(modbus_send_index);
	  break;
          
	default:
	  break;
	}
    }
  } else {  // time to send queue commands
    modbus_send_index = 0;
    if (send_high_speed_data_buffer) {
      total_bytes = BuildModbusOutputHighSpeedDataLog();
      if (send_high_speed_data_buffer & 0x01) {
	data_ptr = (unsigned char *)&high_speed_data_buffer_a[0];
      } else {
	data_ptr = (unsigned char *)&high_speed_data_buffer_b[0];
      } 
      msg_size_bytes = HIGH_SPEED_DATA_BUFFER_SIZE * sizeof(ETMCanHighSpeedData);
      send_high_speed_data_buffer = 0;
    } else if (modbus_command_request) {
      modbus_send_index = MODBUS_RD_COMMAND_DETAIL;
      total_bytes = BuildModbusOutput_read_command(modbus_send_index, 8);
      modbus_command_request = 0; 
    }
    
    switch (modbus_send_index)
      {
      case MODBUS_RD_COMMAND_DETAIL:

	break;
      default:
	break;
      }
  }
  
  if (total_bytes) {
    transaction_number++; // don't care about overflow
    modbus_cmd_need_repeat = 1; // clear when there is response
  }
  return (total_bytes);
}
/*****************************************************************************
  Function:
	void GenericTCPClient(void)

  Summary:
	Implements a simple HTTP client (over TCP).

  Description:
	This function implements a simple HTTP client, which operates over TCP.  
	The function is called periodically by the stack, and waits for BUTTON1 
	to be pressed.  When the button is pressed, the application opens a TCP
	connection to an Internet search engine, performs a search for the word
	"Microchip" on "microchip.com", and prints the resulting HTML page to
	the UART.
	
	This example can be used as a model for many TCP and HTTP client
	applications.

  Precondition:
	TCP is initialized.

  Parameters:
	None

  Returns:
  	None
***************************************************************************/
void GenericTCPClient(void) {
  WORD				w, len;
  
  ETMModbusTXData tx_data;
  
  
  
  static DWORD		Timer;
  static TCP_SOCKET	MySocket = INVALID_SOCKET;
  static enum _GenericTCPExampleState {
    SM_HOME = 0,
    SM_SOCKET_OBTAINED,
    SM_PROCESS_RESPONSE,
    SM_DISCONNECT,
    SM_DONE
  } GenericTCPExampleState = SM_DONE;

  switch(GenericTCPExampleState) 
    {
    case SM_HOME:
      // Connect a socket to the remote TCP server, 192.168.66.15
      MySocket = TCPOpen(AppConfig.MyRemIPAddr.Val, TCP_OPEN_IP_ADDRESS, 502, TCP_PURPOSE_TCP_MODBUS_CLIENT);
      
      // Abort operation if no TCP socket of type TCP_PURPOSE_GENERIC_TCP_CLIENT is available
      // If this ever happens, you need to go add one to TCPIPConfig.h
      if(MySocket == INVALID_SOCKET)
	break;
      
      GenericTCPExampleState = SM_SOCKET_OBTAINED;
      Timer = ETMTickGet();
      break;

    case SM_SOCKET_OBTAINED:
      // Wait for the remote server to accept our connection request
      if(!TCPIsConnected(MySocket)) {
	// Time out if too much time is spent in this state
	if(ETMTickGreaterThanNMilliseconds(5000, Timer)) {
	  // Close the socket so it can be used by other modules
	  TCPDisconnect(MySocket);
	  MySocket = INVALID_SOCKET;
	  GenericTCPExampleState = SM_HOME;
	}
	break;
      }
      
      Timer = ETMTickGet();
      
      // Make certain the socket can be written to
      //if (TCPIsPutReady(MySocket) < MAX_TX_SIZE) break;  // DPARKER fix this

      //len = BuildModbusOutput();
      //if (len == 0) break;  
      //if (header_length == 0) break;  // don't want to send anything for now, stay in this state      

      tx_data = ETMModbusApplicationSpecificTXData();

      if ((tx_data.header_length + tx_data.data_length) > TCPIsPutReady(MySocket)) {
	// The socket can not be written to
	break;
      }
      
      if (tx_data.header_length == 0) {
	// don't want to send anything for now, stay in this state
	break;
      }

      TCPPutArray(MySocket, tx_data.header_ptr, tx_data.header_length);
      TCPPutArray(MySocket, tx_data.data_ptr, tx_data.data_length);
      
      // Send the packet
      TCPFlush(MySocket);
      GenericTCPExampleState = SM_PROCESS_RESPONSE;
      break;

    case SM_PROCESS_RESPONSE:
      // Check to see if the remote node has disconnected from us or sent us any application data
      if(!TCPIsConnected(MySocket)) {
	GenericTCPExampleState = SM_DISCONNECT;
	// Do not break;  We might still have data in the TCP RX FIFO waiting for us
      }
      
      // Get count of RX bytes waiting
      w = TCPIsGetReady(MySocket);	
      
      if (w) {
	if (w > (MAX_RX_SIZE-1)) {
	  w = (MAX_RX_SIZE-1);
	}
	
	len = TCPGetArray(MySocket, buffer_header, w);
	w -= len;
	
	ETMModbusApplicationSpecificRXData(buffer_header);
	
	modbus_cmd_need_repeat = 0;
	GenericTCPExampleState = SM_SOCKET_OBTAINED; // repeat sending
	
      } else {
	// Time out if too much time is spent in this state
	if(ETMTickGreaterThanNMilliseconds(1000, Timer)) {
	  // Close the socket so it can be used by other modules
	  TCPDisconnect(MySocket);
	  MySocket = INVALID_SOCKET;
	  GenericTCPExampleState = SM_HOME;
	}
      }
      
      break;
      
    case SM_DISCONNECT:
      // Close the socket so it can be used by other modules
      // For this application, we wish to stay connected, but this state will still get entered if the remote server decides to disconnect
      TCPDisconnect(MySocket);
      MySocket = INVALID_SOCKET;
      GenericTCPExampleState = SM_DONE;
      break;
      
    case SM_DONE:
      GenericTCPExampleState = SM_HOME;
      break;
    }
}


// ------------------------------------ ////



















ETMModbusTXData ETMModbusApplicationSpecificTXData(void) {
  unsigned int    total_length;
  ETMModbusTXData data_to_send;
  
  total_length = BuildModbusOutput();
  data_to_send.header_length = header_length;
  data_to_send.data_length = total_length - header_length;
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
