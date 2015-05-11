#ifndef __A36507_CONFIG_H
#define __A36507_CONFIG_H



#define __SYSTEM_CONFIGURATION_2_5_MEV

#define DRIVE_UP_TIMEOUT                     1000  // 10 Seconds

#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes









// Default Control Parameters to load into EEPROM

#define DEFAULT_MAGNITRON_HEATER_CURRENT                           9000
#define DEFAULT_MAGNET_CURRENT_PER_1                              16000
#define DEFAULT_MAGNET_CURRENT_PER_2                              16000
#define DEFAULT_MAGNET_CURRENT_PER_3                              16000
#define DEFAULT_MAGNET_CURRENT_PER_4                              16000
#define DEFAULT_AFC_HOME_PER_1                                    19200
#define DEFAULT_AFC_HOME_PER_2                                    19200
#define DEFAULT_AFC_HOME_PER_3                                    19200
#define DEFAULT_AFC_HOME_PER_4                                    19200


#define DEFAULT_HV_LAMBDA_HIGH_PER_1                              15000
#define DEFAULT_HV_LAMBDA_LOW_PER_1                               15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_2                              15000
#define DEFAULT_HV_LAMBDA_LOW_PER_2                               15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_3                              15000
#define DEFAULT_HV_LAMBDA_LOW_PER_3                               15000
#define DEFAULT_HV_LAMBDA_HIGH_PER_4                              15000
#define DEFAULT_HV_LAMBDA_LOW_PER_4                               15000


#define DEFAULT_GUN_DRV_HEATER_VOLT                                6300
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_1                       1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_1                        1800
#define DEFAULT_GUN_DRV_CATHODE_PER_1                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_2                       1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_2                        1800
#define DEFAULT_GUN_DRV_CATHODE_PER_2                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_3                       1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_3                        1800
#define DEFAULT_GUN_DRV_CATHODE_PER_3                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_4                       1800
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_4                        1800
#define DEFAULT_GUN_DRV_CATHODE_PER_4                             20000


#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_A                0x06
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_B                0x05
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_C                0x04
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_D                0x03
#define DEFAULT_P_SYNC_HIGH_THYRATRON_DELAY_PER_1                  0x02
#define DEFAULT_P_SYNC_HIGH_DOSE_SAMPLE_DELAY_PER_1                0x01

#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_A                0x16
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_B                0x15
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_C                0x14
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_D                0x13
#define DEFAULT_P_SYNC_HIGH_AFC_SAMPLE_DELAY_PER_1                 0x12
#define DEFAULT_P_SYNC_HIGH_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1   0x11

#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_A                 0x26
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_B                 0x25
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_C                 0x24
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_D                 0x23
#define DEFAULT_P_SYNC_LOW_THYRATRON_DELAY_PER_1                   0x22
#define DEFAULT_P_SYNC_LOW_DOSE_SAMPLE_DELAY_PER_1                 0x21

#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_A                 0x36
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_B                 0x35
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_C                 0x34
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_D                 0x33
#define DEFAULT_P_SYNC_LOW_AFC_SAMPLE_DELAY_PER_1                  0x32
#define DEFAULT_P_SYNC_LOW_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1    0x31

#define DEFAULT_UNUSED_EEPROM                                         0




#endif
