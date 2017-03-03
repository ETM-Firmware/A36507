#ifndef __A36507_CONFIG_H
#define __A36507_CONFIG_H



#define __SYSTEM_CONFIGURATION_2_5_MEV

#define DRIVE_UP_TIMEOUT                     1000 // 10 Seconds
#define MINIMUM_FAULT_HOLD_TIME              50   // .5 Second
#define FAULT_RESET_HOLD_TIME                200  // 2 Second
#define MAX_DRIVE_UP_FAULTS                  5    // Attempts to bring up high voltage before latching fault
#define MAX_HV_ON_FAULTS                     10   // Attempts to reset faults with HV on (but Xray Off) before latching 

#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes

#define GUN_HEATER_HOLDOFF_AT_STARTUP        500   // 5 seconds
#define GUN_HEATER_ADDITONAL_HOLDOFF_COLD   2500   // 25 seconds

// Page 0
#define DEFAULT_MAGNETRON_HEATER_CURRENT                           8000
#define DEFAULT_ELECTROMAGNET_CURRENT                             15200
#define DEFAULT_ELECTROMAGNET_CURRENT_LOW                         15200
#define DEFAULT_AFC_HOME_POSITION                                 23000
#define DEFAULT_AFT_CONTROL_VOLTAGE                                2000
#define DEFAULT_AFT_CONTROL_VOLTAGE_LOW                            2000


// Page 1
#define DEFAULT_HV_LAMBDA_SET_POINT                               14000
#define DEFAULT_HV_LAMBDA_SET_POINT_LOW                           14000
#define DEFAULT_REMOTE_IP_ADDRESS                            0x0F46A8C0  // 192.168.70.15
#define DEFAULT_IP_ADDRESS                                   0x6346A8C0  // 192.168.70.99
#define DEFAULT_TOP_LEVEL_SERIAL_NUMBER                               1
#define DEFAULT_EEPROM_OK                                        0xACAC


// Page 2
#define DEFAULT_GUN_DRIVER_HEATER_CURRENT                          1450
#define DEFAULT_GUN_DRIVER_PULSE_TOP                                840
#define DEFAULT_GUN_DRIVER_PULSE_TOP_LOW                            840
#define DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE                        20000


// Page 3,4,5,6
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_3                         25
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_2                         50
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_1                         75
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_0                         100
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_3_LOW                     25
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_2_LOW                     50
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_1_LOW                     75
#define DEFAULT_PULSE_SYNC_PULSE_START_VAL_0_LOW                     100

#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_3                          200
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_2                          150
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_1                          125
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_0                          105
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_3_LOW                      200
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_2_LOW                      150
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_1_LOW                      125
#define DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_0_LOW                      105

#define DEFAULT_PULSE_SYNC_THYRATRON_DELAY                            0
#define DEFAULT_PULSE_SYNC_THYRATRON_DELAY_LOW                        0
#define DEFAULT_PULSE_SYNC_HV_LAMBDA_TRIGGER_DELAY                    0
#define DEFAULT_PULSE_SYNC_HV_LAMBDA_TRIGGER_DELAY_LOW                0
#define DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY                         150                        
#define DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY_LOW                     150
#define DEFAULT_PULSE_SYNC_MAGNETRON_SAMPLE_DELAY                   150
#define DEFAULT_PULSE_SYNC_MAGNETRON_SAMPLE_DELAY_LOW               150


#define DEFAULT_UNUSED_EEPROM                                         0


const unsigned int eeprom_default_values_htr_mag_afc[16] = {DEFAULT_MAGNETRON_HEATER_CURRENT,
							    DEFAULT_ELECTROMAGNET_CURRENT,
							    DEFAULT_ELECTROMAGNET_CURRENT,
							    DEFAULT_ELECTROMAGNET_CURRENT,
							    DEFAULT_ELECTROMAGNET_CURRENT,
							    DEFAULT_AFC_HOME_POSITION,
							    DEFAULT_AFC_HOME_POSITION,
							    DEFAULT_AFC_HOME_POSITION,
							    DEFAULT_AFC_HOME_POSITION,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_AFT_CONTROL_VOLTAGE,
							    DEFAULT_AFT_CONTROL_VOLTAGE_LOW,
							    DEFAULT_ELECTROMAGNET_CURRENT_LOW,
							    DEFAULT_ELECTROMAGNET_CURRENT_LOW,
							    DEFAULT_ELECTROMAGNET_CURRENT_LOW,
							    DEFAULT_ELECTROMAGNET_CURRENT_LOW};


const unsigned int eeprom_default_values_hv_lambda[16]   = {DEFAULT_HV_LAMBDA_SET_POINT,
							    DEFAULT_HV_LAMBDA_SET_POINT_LOW,
							    DEFAULT_HV_LAMBDA_SET_POINT,
							    DEFAULT_HV_LAMBDA_SET_POINT_LOW,
							    DEFAULT_HV_LAMBDA_SET_POINT,
							    DEFAULT_HV_LAMBDA_SET_POINT_LOW,
							    DEFAULT_HV_LAMBDA_SET_POINT,
							    DEFAULT_HV_LAMBDA_SET_POINT_LOW,
							    0x0F46,
							    0xA8C0,
							    0x6346,
							    0xA8C0,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_EEPROM_OK,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_TOP_LEVEL_SERIAL_NUMBER};

const unsigned int eeprom_default_values_gun_driver[16]  = {DEFAULT_GUN_DRIVER_HEATER_CURRENT,
							    DEFAULT_GUN_DRIVER_PULSE_TOP,
							    DEFAULT_GUN_DRIVER_PULSE_TOP_LOW,
							    DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE,
							    DEFAULT_GUN_DRIVER_PULSE_TOP,
							    DEFAULT_GUN_DRIVER_PULSE_TOP_LOW,
							    DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE,
							    DEFAULT_GUN_DRIVER_PULSE_TOP,
							    DEFAULT_GUN_DRIVER_PULSE_TOP_LOW,
							    DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE,
							    DEFAULT_GUN_DRIVER_PULSE_TOP,
							    DEFAULT_GUN_DRIVER_PULSE_TOP_LOW,
							    DEFAULT_GUN_DRIVER_CATHODE_VOLTAGE,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};

const unsigned int eeprom_default_values_p_sync_per_1[16]= {((DEFAULT_PULSE_SYNC_PULSE_START_VAL_2 << 8) + DEFAULT_PULSE_SYNC_PULSE_START_VAL_3),
							    ((DEFAULT_PULSE_SYNC_PULSE_START_VAL_0 << 8) + DEFAULT_PULSE_SYNC_PULSE_START_VAL_1),
							    ((DEFAULT_PULSE_SYNC_HV_LAMBDA_TRIGGER_DELAY << 8) + DEFAULT_PULSE_SYNC_THYRATRON_DELAY),
							    DEFAULT_UNUSED_EEPROM,
							    ((DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_2 << 8) + DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_3),
							    ((DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_0 << 8) + DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_1),
							    ((DEFAULT_PULSE_SYNC_MAGNETRON_SAMPLE_DELAY << 8) + DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY),
							    DEFAULT_UNUSED_EEPROM,
							    ((DEFAULT_PULSE_SYNC_PULSE_START_VAL_2_LOW << 8) + DEFAULT_PULSE_SYNC_PULSE_START_VAL_3_LOW),
							    ((DEFAULT_PULSE_SYNC_PULSE_START_VAL_0_LOW << 8) + DEFAULT_PULSE_SYNC_PULSE_START_VAL_1_LOW),
							    ((DEFAULT_PULSE_SYNC_HV_LAMBDA_TRIGGER_DELAY_LOW << 8) + DEFAULT_PULSE_SYNC_THYRATRON_DELAY_LOW),
							    DEFAULT_UNUSED_EEPROM,	
							    ((DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_2_LOW << 8) + DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_3_LOW),
							    ((DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_0_LOW << 8) + DEFAULT_PULSE_SYNC_PULSE_STOP_VAL_1_LOW),
							    ((DEFAULT_PULSE_SYNC_MAGNETRON_SAMPLE_DELAY_LOW << 8) + DEFAULT_PULSE_SYNC_AFC_SAMPLE_DELAY_LOW),
							    DEFAULT_UNUSED_EEPROM};






#define MAX_SF6_REFILL_PULSES_IN_BOTTLE                             2100



#endif
