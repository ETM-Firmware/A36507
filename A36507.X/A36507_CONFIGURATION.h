#ifndef __A36507_CONFIGURATION_H
#define __A36507_CONFIGURATION_H


// Required Parameters
#define __USE_CAN_1
#define __USE_EXTERNAL_EEPROM               
#define FCY_CLK                             20000000
#define ETM_CAN_MY_ADDRESS                  ETM_CAN_ADDR_ETHERNET_BOARD
#define PIN_CAN_OPERATION_LED               _LATG13
#define ETM_CAN_INTERRUPT_PRIORITY          4  

#define ETM_CAN_AGILE_ID_HIGH               0
#define ETM_CAN_AGILE_ID_LOW                36507
#define ETM_CAN_AGILE_DASH                  0
#define ETM_CAN_AGILE_REV                   'A'                   
#define ETM_CAN_SERIAL_NUMBER               100 // DPARKER need to figure out how to set this in FLASH when programming



//#define __IGNORE_ION_PUMP_MODULE
//#define __IGNORE_AFC_MODULE
//#define __IGNORE_GUN_DRIVER_MODULE
//#define __IGNORE_COOLING_INTERFACE_MODULE
//#define __IGNORE_HEATER_MAGNET_MODULE
//#define __IGNORE_HV_LAMBDA_MODULE
//#define __IGNORE_PULSE_CURRENT_MODULE
//#define __IGNORE_PULSE_SYNC_MODULE
#define __IGNORE_TCU


#endif
