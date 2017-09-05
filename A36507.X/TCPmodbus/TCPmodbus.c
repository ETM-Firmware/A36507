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

//#define THIS_IS_STACK_APPLICATION


#include <xc.h>
#include "TCPIPStack/TCPIPStack/TCPIPConfig.h"
#include "TCPIPStack/TCPIPStack/TCPIP.h"
#include "TCPIPStack/TCPIPStack/ETM_TICK.h"
#include "TCPmodbus.h"





static void InitializeBoard(void);
static void InitAppConfig(IPCONFIG* ip_config);
void TCPmodbus_init(IPCONFIG* ip_config);
void TCPmodbus_task(void);
void GenericTCPClient(void);






ETMModbusTXData ETMModbusApplicationSpecificTXData(void);
void ETMModbusApplicationSpecificRXData(unsigned char data_RX[]);

unsigned long led_flash_holding_var;

unsigned char rx_data[MAX_RX_SIZE];

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;
static unsigned short wOriginalAppConfigChecksum;    // Checksum of the ROM defaults for AppConfig

#define LED_PUT(a)	do{unsigned char vTemp = (a); LEDOP_IO = vTemp&0x1; LEDA_IO = vTemp&0x4; LEDB_IO = vTemp&0x2;} while(0)

void GenericTCPClient(void);
void InitModbusData(void);

unsigned char         modbus_cmd_need_repeat = 0;  







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
	
	len = TCPGetArray(MySocket, rx_data, w);
	w -= len;
	
	ETMModbusApplicationSpecificRXData(rx_data);
	
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

