#ifndef __A36507_H
#define __A36507_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <uart.h>

#include "ETM.h"
#include "TCPmodbus.h"
#include "P1395_CAN_MASTER.h"


/*
  Hardware Module Resource Usage

  CAN1 - Can module
  CAN2 - Reserved in case we need CAN 2

  Timer1 - Used by Ethernet Module

  Timer4 - Used to time CAN transmits - This is configured by ETM CAN module
  Timer5 - Used as timeout on status update receives - This is configured by ETM CAN module

  Timer2 - Used for Ethernet Board 10ms timing 

  UART1 - Reserved for TCU Communication
  UART2 - Reserved for Serial GUI


 */









// ----------------- IO PIN CONFIGURATION -------------------- //
/*
  All unused pins will be set to outputs and logic zero
  LAT values default to 0 at startup so they do not need to be manually set
*/



// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RA9  (Accidentally left grounded)
  RG0  (Unused Can Pin)
  RG1  (Unused Can Pin)
  RG14 (Reset Detect)
  RB14 (Analog Input)
  RB15 (Analog Input)
  
  Pins that are overidden by a hardware module and should be left as inputs during port configuration
  RB0 PROGRAM
  RB1 PROGRAM

  RF0 CAN 1
  RF1 CAN 1
  RF2 UART 1
  RF3 UART 1
  RF4 UART 2
  RF5 UART 2
  RF6 SPI 1
  RF7 SPI 1
  RF8 SPI 1

  RG2 I2C
  RG3 I2C

  Pins that are configured by other software modules and should be left as inputs during port configuration
  RA14 (Ethernet Module Interrupt Input)
  RA15 (Ethernet Module Reset Output)
  RD14 (Ethernet Module Clock Input)
  RD15 (Ethernet Module CS Output)
  

*/

#define A36507_TRISA_VALUE 0b1100001000000000 
#define A36507_TRISB_VALUE 0b0110000000000011 
#define A36507_TRISC_VALUE 0b0000000000000000 
#define A36507_TRISD_VALUE 0b1100000000000000 
#define A36507_TRISF_VALUE 0b0000000111111111 
#define A36507_TRISG_VALUE 0b0100000000001111


#define PIN_OUT_ETM_UART_1_DE                 _LATD7
#define PIN_OUT_ETM_UART_2_DE                 _LATD6
#define OLL_UART_TX_DRIVER_ENABLE             1

#define PIN_IN_ETM_RESET_DETECT               _RG14
#define PIN_OUT_ETM_RESET_DETECT              _LATG14
#define TRIS_PIN_ETM_RESET_RETECT             _TRISG14

#define PIN_OUT_ETM_LED_OPERATIONAL_GREEN     _LATA7
#define PIN_OUT_ETM_LED_TEST_POINT_A_RED      _LATG12
#define PIN_OUT_ETM_LED_TEST_POINT_B_GREEN    _LATG13
#define OLL_LED_ON                            0

#define PIN_OUT_TP_13                         _LATC15
#define PIN_OUT_TP_14                         _LATB7
#define PIN_OUT_TP_15                         _LATB8
#define PIN_OUT_TP_16                         _LATB9


// --------------- CONFIGURE TMR2 MODULE ----------------------- //
#define T2CON_VALUE                    (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define PR2_PERIOD_US                  10000   // 10mS
#define PR2_VALUE_10_MILLISECONDS      ((FCY_CLK/1000000)*PR2_PERIOD_US/8)



// ------------------------ CONFIGURE ADC MODULE ------------------- //
#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_AVDD_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_16 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_10Tcy)

#define ADPCFG_SETTING          (ENABLE_AN13_ANA & ENABLE_AN14_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 & SKIP_SCAN_AN6 &  SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN15)

#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN13 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN14 & ADC_CH0_NEG_SAMPLEB_VREFN)








// ---------------------- FAULTS/WARNINGS ------------------------ //
#define FAULT_A36507_CAN_TIMEOUT              0b0000 0000 0000 0001
#define FAULT_A36507_CAN_ETHERNET_TIMEOUT     0b0000 0000 0000 0010
//#define FAULT_A36507_     




typedef struct {
  AnalogInput analog_input_5v_mon;                    // 1mV per LSB
  AnalogInput analog_input_3v3_mon;                   // 1mV per LSB

  unsigned int control_state;
  //unsigned int thyratron_warmup_counter_seconds;
  //unsigned int magnetron_heater_warmup_counter_seconds;
  //unsigned int gun_driver_heater_warmup_counter_seconds;

  //unsigned int millisecond_counter;
  unsigned int warmup_timer_stage;
  unsigned int warmup_done;
  unsigned int gun_heater_holdoff_timer;

  unsigned long last_recorded_warmup_seconds;
  unsigned int  thyratron_warmup_remaining;
  unsigned int  magnetron_warmup_remaining;
  unsigned int  gun_warmup_remaining;

  //unsigned long system_powered_seconds;
  //unsigned long system_hv_on_seconds;
  //unsigned long system_xray_on_seconds;

  RTC_TIME time_now;
  //unsigned long time_seconds_now;
  

  //unsigned int send_pulse_sync_config;
  unsigned int drive_up_timer;

  //unsigned int average_output_power_watts;
  unsigned int event_log_counter;
  
  unsigned int startup_counter;


  unsigned int no_connect_count_ion_pump_board;
  unsigned int no_connect_count_magnetron_current_board;
  unsigned int no_connect_count_pulse_sync_board;
  unsigned int no_connect_count_hv_lambda_board;
  unsigned int no_connect_count_afc_board;
  unsigned int no_connect_count_cooling_interface_board;
  unsigned int no_connect_count_heater_magnet_board;
  unsigned int no_connect_count_gun_driver_board;

  unsigned int buffer_a_ready_to_send;
  unsigned int buffer_b_ready_to_send;
  unsigned int buffer_a_sent;
  unsigned int buffer_b_sent;

  unsigned int reset_requested;
  
  //unsigned int personality_select_from_pulse_sync;

  unsigned int drive_up_fault_counter;
  unsigned int high_voltage_on_fault_counter;
  unsigned int reset_hold_timer;

  unsigned int system_serial_number;
  unsigned int most_recent_ref_detector_reading;

  unsigned int eeprom_failure;

  unsigned int timer_flash;
  unsigned int timer_flash_counter;

  unsigned int start_power_cycle_test;

  unsigned int power_cycle_timer;

  unsigned int debug_cal_set_request;
  unsigned int debug_cal_set_local;
  unsigned int debug_cal_set_can;
  unsigned int debug_cal_read_request;


} A36507GlobalVars;

//#define thyratron_warmup_counter_seconds                     local_data_ecb.log_data[4]
//#define magnetron_heater_warmup_counter_seconds              local_data_ecb.log_data[5]
//#define gun_driver_heater_warmup_counter_seconds             local_data_ecb.log_data[6]
#define system_powered_seconds                               (*(unsigned long*)&local_data_ecb.log_data[8])
#define system_hv_on_seconds                                 (*(unsigned long*)&local_data_ecb.log_data[10])
#define system_xray_on_seconds                               (*(unsigned long*)&local_data_ecb.log_data[12])
#define average_output_power_watts                           local_data_ecb.log_data[14]
#define personality_select_from_pulse_sync                   local_data_ecb.log_data[15]
#define x_ray_on_time_set_point                              (*(unsigned long*)&local_data_ecb.log_data[20])
#define x_ray_on_time_counter                                (*(unsigned long*)&local_data_ecb.log_data[22])
#define power_cycle_faults                                   local_data_ecb.local_data[0]
#define power_cycle_counter                                  local_data_ecb.local_data[1]




extern A36507GlobalVars global_data_A36507;


#define _FAULT_X_RAY_ON_LOGIC_ERROR                     _FAULT_0
#define _FAULT_REPEATED_DRIVE_UP_FAULT                  _FAULT_1
#define _FAULT_REPEATED_HV_ON_FAULT                     _FAULT_2
#define _FAULT_EEPROM_FAILURE                           _FAULT_3

// DPAKRER  - Need to evaluate how these are used under new control system
#define _STATUS_PERSONALITY_LOADED                      _LOGGED_0
#define _STATUS_DRIVE_UP_TIMEOUT                        _LOGGED_1

#define _STATUS_LAST_RESET_WAS_POWER_CYCLE              _NOT_LOGGED_0




// These are computed from Filament Lookup Table worksheet
// https://docs.google.com/spreadsheets/d/18de5OHQ0gJUx2U1b8VjYTvutx2ACGTG0XtN_QEIc9WI/

#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095 0xFFF,0xFD6,0xFD6,0xFAE,0xFAE,0xF85,0xF5C,0xF33,0xF0A,0xF0A,0xEE1,0xEB8,0xE8F,0xE66,0xE3D,0xE14,0xDEB,0xDC2,0xD70,0xD47,0xD1E,0xCF5,0xCA3,0xC7A,0xC51,0xBFF,0xBD6,0xB85,0xB5C,0xB0A,0xAB8,0xA8F,0xA3D,0x9EB,0x9C2,0x970,0x91E,0x8CC,0x87A,0x828,0x7D7,0x785,0x733,0x6E1,0x68F,0x63D,0x5EB,0x599,0x51E,0x4CC,0x47A,0x3FF,0x3AE,0x35C,0x2E1,0x28F,0x214,0x199,0x147,0xCC,0x7A,0x0,0x0,0x0 

#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG7095_V2 0xFFF,0xFF1,0xFE2,0xFD2,0xFC2,0xFB1,0xFA0,0xF8D,0xF7B,0xF67,0xF53,0xF3F,0xF29,0xF13,0xEFD,0xEE6,0xECE,0xEB6,0xE9D,0xE83,0xE69,0xE4E,0xE33,0xE16,0xDFA,0xDDC,0xDBE,0xDA0,0xD81,0xD61,0xD40,0xD1F,0xCFE,0xCDB,0xCB8,0xC95,0xC71,0xC4C,0xC26,0xC00,0xBDA,0xBB2,0xB8A,0xB62,0xB39,0xB0F,0xAE4,0xAB9,0xA8E,0xA62,0xA35,0xA07,0x9D9,0x9AA,0x97B,0x94B,0x91A,0x8E9,0x8B7,0x885,0x851,0x81E,0x7E9,0x7B4

#define FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193 0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xFFF,0xF85,0xF0A,0xE8F,0xE14,0xD70,0xCF5,0xC7A,0xBFF,0xB85,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0


#define EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC               0x000
#define EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA                 0x001
#define EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV                   0x002
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1          0x003
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2          0x004
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3          0x005
#define EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4          0x006
#define EEPROM_PAGE_ON_TIME                                 0x007
#define EEPROM_PAGE_HEATER_TIMERS                           0x008
// EEPROM PAGES reserved for future use                     9->F


#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HTR_MAG_AFC               0x020
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HV_LAMBDA                 0x021
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_GUN_DRV                   0x022
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_1          0x023
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_2          0x024
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_3          0x025
#define EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_4          0x026




#define STATE_STARTUP                                0x10
#define STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC   0x12
#define STATE_WAITING_FOR_INITIALIZATION             0x15
#define STATE_WARMUP                                 0x20
#define STATE_STANDBY                                0x30
#define STATE_DRIVE_UP                               0x40
#define STATE_READY                                  0x50
#define STATE_XRAY_ON                                0x60
#define STATE_X_RAY_TIME_EXCEEDED                    0x70


#define STATE_FAULT_HOLD                             0x80
#define STATE_FAULT_RESET_HOLD                       0x86
#define STATE_FAULT_LATCH_DECISION                   0x8A
//#define STATE_FAULT_RESET                            0x90
#define STATE_FAULT_SYSTEM                           0xA0
#define STATE_FAULT_WARMUP                           0xB0
#define STATE_FAULT_STANDBY                          0xC0
#define STATE_POWER_CYCLE_TEST                       0xD1





#define EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT                      0x0000
#define EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_HIGH_ENERGY          0x0001
#define EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_LOW_ENERGY           0x000C
#define EEPROM_REGISTER_AFC_HOME_POSITION                           0x0005
#define EEPROM_REGISTER_AFC_OFFSET                                  0x0009
#define EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_HIGH_ENERGY         0x000A
#define EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_LOW_ENERGY          0x000B

#define EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT                0x0010
#define EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT                 0x0011
#define EEPROM_REGISTER_REMOTE_IP_ADDRESS                           0x0018
#define EEPROM_REGISTER_IP_ADDRESS                                  0x001A
#define EEPROM_REGISTER_X_RAY_ON_TIME                               0x001C
#define EEPROM_REGISTER_EEPROM_OK_CHECK                             0x001E
#define EEPROM_REGISTER_TOP_LEVEL_SERIAL_NUMBER                     0x001F

#define EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE                         0x0020
#define EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP                      0x0021
#define EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP                       0x0022
#define EEPROM_REGISTER_GUN_DRV_CATHODE                             0x0023


#define _SYNC_CONTROL_PULSE_SYNC_NDT_X_RAY_ON_LED                   etm_can_master_sync_message.sync_0_control_word.sync_8_unused

#ifdef __NDT_LINAC
// Redfine the warmup and X RAY led pins
#undef _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED
#undef _SYNC_CONTROL_PULSE_SYNC_NDT_X_RAY_ON_LED
#define _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED                         etm_can_master_sync_message.sync_0_control_word.sync_8_unused
#define _SYNC_CONTROL_PULSE_SYNC_NDT_X_RAY_ON_LED                   etm_can_master_sync_message.sync_0_control_word.sync_A_pulse_sync_warmup_led_on

#endif


#endif
