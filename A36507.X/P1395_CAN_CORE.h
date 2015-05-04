#ifndef __P1395_CAN_CORE_H
#define __P1395_CAN_CORE_H

// ---------------------- STATUS REGISTER DEFFENITIONS   ------------------------- //

typedef struct {
  unsigned control_0_not_ready:1;
  unsigned control_1_can_status:1;
  unsigned control_2_not_configured:1;
  unsigned control_3_self_check_error:1;
  unsigned control_4_unused:1;
  unsigned control_5_unused:1;
  unsigned control_6_unused:1;
  unsigned control_7_ecb_can_not_active:1;
  
  unsigned status_0:1;
  unsigned status_1:1;
  unsigned status_2:1;
  unsigned status_3:1;
  unsigned status_4:1;
  unsigned status_5:1;
  unsigned status_6:1;
  unsigned status_7:1;
} ETMCanStatusRegisterStatusBits;

typedef struct {
  unsigned fault_0:1;
  unsigned fault_1:1;
  unsigned fault_2:1;
  unsigned fault_3:1;
  unsigned fault_4:1;
  unsigned fault_5:1;
  unsigned fault_6:1;
  unsigned fault_7:1;
  unsigned fault_8:1;
  unsigned fault_9:1;
  unsigned fault_A:1;
  unsigned fault_B:1;
  unsigned fault_C:1;
  unsigned fault_D:1;
  unsigned fault_E:1;
  unsigned fault_F:1;
} ETMCanStatusRegisterFaultBits;


typedef struct {
  ETMCanStatusRegisterStatusBits status_bits;  // 16 bits
  ETMCanStatusRegisterFaultBits  fault_bits;   // 16 bits
  unsigned int data_word_A;
  unsigned int data_word_B;

  unsigned int unused_A;                       // DPARKER Remove this when you figure out how
  unsigned int unused_B;                       // DPARKER Remove this when you figure out how
} ETMCanStatusRegister;

extern ETMCanStatusRegister  etm_can_status_register;

#define _CONTROL_NOT_READY            etm_can_status_register.status_bits.control_0_not_ready
#define _CONTROL_CAN_COM_LOSS         etm_can_status_register.status_bits.control_1_can_status
#define _CONTROL_NOT_CONFIGURED       etm_can_status_register.status_bits.control_2_not_configured
#define _CONTROL_SELF_CHECK_ERROR     etm_can_status_register.status_bits.control_3_self_check_error


#define _STATUS_0                     etm_can_status_register.status_bits.status_0
#define _STATUS_1                     etm_can_status_register.status_bits.status_1
#define _STATUS_2                     etm_can_status_register.status_bits.status_2
#define _STATUS_3                     etm_can_status_register.status_bits.status_3
#define _STATUS_4                     etm_can_status_register.status_bits.status_4
#define _STATUS_5                     etm_can_status_register.status_bits.status_5
#define _STATUS_6                     etm_can_status_register.status_bits.status_6
#define _STATUS_7                     etm_can_status_register.status_bits.status_7

#define _FAULT_0                      etm_can_status_register.fault_bits.fault_0
#define _FAULT_1                      etm_can_status_register.fault_bits.fault_1
#define _FAULT_2                      etm_can_status_register.fault_bits.fault_2
#define _FAULT_3                      etm_can_status_register.fault_bits.fault_3
#define _FAULT_4                      etm_can_status_register.fault_bits.fault_4
#define _FAULT_5                      etm_can_status_register.fault_bits.fault_5
#define _FAULT_6                      etm_can_status_register.fault_bits.fault_6
#define _FAULT_7                      etm_can_status_register.fault_bits.fault_7
#define _FAULT_8                      etm_can_status_register.fault_bits.fault_8
#define _FAULT_9                      etm_can_status_register.fault_bits.fault_9
#define _FAULT_A                      etm_can_status_register.fault_bits.fault_A
#define _FAULT_B                      etm_can_status_register.fault_bits.fault_B
#define _FAULT_C                      etm_can_status_register.fault_bits.fault_C
#define _FAULT_D                      etm_can_status_register.fault_bits.fault_D
#define _FAULT_E                      etm_can_status_register.fault_bits.fault_E
#define _FAULT_F                      etm_can_status_register.fault_bits.fault_F


#define _FAULT_REGISTER               *(unsigned int*)&etm_can_status_register.fault_bits
#define _CONTROL_REGISTER             *(unsigned int*)&etm_can_status_register.status_bits









// -----------------------  DEBUG REGISTERS DEFFENTITIONS --------------------- //


typedef struct {
  unsigned st_5V_over_voltage:1;
  unsigned st_5V_under_voltage:1;
  unsigned st_15V_over_voltage:1;
  unsigned st_15V_under_voltage:1;
  unsigned st_N15V_over_voltage:1;
  unsigned st_N15V_under_voltage:1;
  unsigned st_ADC_over_voltage:1;
  unsigned st_ADC_under_voltage:1;
  unsigned st_ADC_EXT:1;
  unsigned st__EEPROM:1;
  unsigned st_DAC:1;
  unsigned st_spare_4:1;
  unsigned st_spare_3:1;
  unsigned st_spare_2:1;
  unsigned st_spare_1:1;
  unsigned st_spare_0:1;
  
} ETMCanSelfTestRegister;

// DPARKER - FIGURE OUT BETTER USE Of FIRST 8 words of debug register
typedef struct {
  // Log Message 0x0 
  unsigned int i2c_bus_error_count;
  unsigned int spi_bus_error_count;
  unsigned int can_bus_error_count;
  unsigned int scale_error_count;
  // Log Message 0x1
  unsigned int reset_count;
  ETMCanSelfTestRegister self_test_results;
  unsigned int reserved_1;   // This is the RCON register
  unsigned int reserved_0;
  // Log Message 0x2
  unsigned int debug_0;
  unsigned int debug_1;
  unsigned int debug_2;
  unsigned int debug_3;
  // Log Message 0x3
  unsigned int debug_4;
  unsigned int debug_5;
  unsigned int debug_6;
  unsigned int debug_7;
  // Log Message 0x4
  unsigned int debug_8;
  unsigned int debug_9;
  unsigned int debug_A;
  unsigned int debug_B;
  // Log Message 0x5
  unsigned int debug_C;
  unsigned int debug_D;
  unsigned int debug_E;
  unsigned int debug_F;
} ETMCanSystemDebugData;

extern ETMCanSystemDebugData local_debug_data;  











// -------------- AGILE CONFIGURATION DEFFENITION --------------------- //

typedef struct {
  // Log Message 0x6
  unsigned int  agile_number_high_word;
  unsigned int  agile_number_low_word;
  unsigned int  agile_dash;
  unsigned int  agile_rev_ascii;
  // Log Message 0x7
  unsigned int  serial_number;
  unsigned int  firmware_branch;
  unsigned int  firmware_major_rev;
  unsigned int  firmware_minor_rev;
} ETMCanAgileConfig;

extern ETMCanAgileConfig     etm_can_my_configuration;







// -------------------------  SYNC MESSAGE DEFFENITIONS ----------------------- //

typedef struct {
  unsigned sync_0_reset_enable:1;
  unsigned sync_1_high_speed_logging_enabled:1;
  unsigned sync_2_pulse_sync_disable_hv:1;
  unsigned sync_3_pulse_sync_disable_xray:1;
  unsigned sync_4_cooling_fault:1;
  unsigned sync_5_unused:1;
  unsigned sync_6_unused:1;
  unsigned sync_7_unused:1;

  unsigned sync_8_unused:1;
  unsigned sync_9_unused:1;
  unsigned sync_A_unused:1;
  unsigned sync_B_unused:1;
  unsigned sync_C_unused:1;
  unsigned sync_D_unused:1;
  unsigned sync_E_unused:1;
  unsigned sync_F_clear_debug_data:1;
} ETMCanSyncControlWord;


typedef struct {
  ETMCanSyncControlWord sync_0_control_word;
  unsigned int sync_1;
  unsigned int sync_2;
  unsigned int sync_3;
} ETMCanSyncMessage;

extern ETMCanSyncMessage     etm_can_sync_message;


#define _SYNC_CONTROL_RESET_ENABLE            etm_can_sync_message.sync_0_control_word.sync_0_reset_enable
#define _SYNC_CONTROL_HIGH_SPEED_LOGGING      etm_can_sync_message.sync_0_control_word.sync_1_high_speed_logging_enabled
#define _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV   etm_can_sync_message.sync_0_control_word.sync_2_pulse_sync_disable_hv
#define _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY etm_can_sync_message.sync_0_control_word.sync_3_pulse_sync_disable_xray
#define _SYNC_CONTROL_COOLING_FAULT           etm_can_sync_message.sync_0_control_word.sync_4_cooling_fault
#define _SYNC_CONTROL_CLEAR_DEBUG_DATA        etm_can_sync_message.sync_0_control_word.sync_F_clear_debug_data

#define _SYNC_CONTROL_WORD                 *(unsigned int*)&etm_can_sync_message.sync_0_control_word



// Public Variables
extern unsigned int etm_can_next_pulse_level;  // This value will get updated in RAM as when a next pulse level command is received
extern unsigned int etm_can_next_pulse_count;  // This value will get updated in RAM as when a next pulse level command is received
#define ETM_CAN_HIGH_ENERGY           1
#define ETM_CAN_LOW_ENERGY            0


#endif
