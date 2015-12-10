#ifndef __A36507_CONFIG_H
#define __A36507_CONFIG_H



#define __SYSTEM_CONFIGURATION_2_5_MEV

#define DRIVE_UP_TIMEOUT                     1000  // 10 Seconds

#define MAGNETRON_HEATER_WARM_UP_TIME        300   // 5 minutes
#define THYRATRON_WARM_UP_TIME               60 // DPARKER 900   // 15 minutes
#define GUN_DRIVER_HEATER_WARM_UP_TIME       300   // 5 minutes









// Default Control Parameters to load into EEPROM For 2.5 MeV System

#define DEFAULT_MAGNITRON_HEATER_CURRENT                           8000
#define DEFAULT_MAGNET_CURRENT_PER_1                              15200
#define DEFAULT_MAGNET_CURRENT_PER_2                              15200
#define DEFAULT_MAGNET_CURRENT_PER_3                              15200
#define DEFAULT_MAGNET_CURRENT_PER_4                              15200
#define DEFAULT_AFC_HOME_PER_1                                    23000
#define DEFAULT_AFC_HOME_PER_2                                    23000
#define DEFAULT_AFC_HOME_PER_3                                    23000
#define DEFAULT_AFC_HOME_PER_4                                    23000


#define DEFAULT_HV_LAMBDA_HIGH_PER_1                              14000
#define DEFAULT_HV_LAMBDA_LOW_PER_1                               14000
#define DEFAULT_HV_LAMBDA_HIGH_PER_2                              14000
#define DEFAULT_HV_LAMBDA_LOW_PER_2                               14000
#define DEFAULT_HV_LAMBDA_HIGH_PER_3                              14000
#define DEFAULT_HV_LAMBDA_LOW_PER_3                               14000
#define DEFAULT_HV_LAMBDA_HIGH_PER_4                              14000
#define DEFAULT_HV_LAMBDA_LOW_PER_4                               14000


#define DEFAULT_GUN_DRV_HEATER_VOLT                                6300
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_1                        840
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_1                         840
#define DEFAULT_GUN_DRV_CATHODE_PER_1                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_2                        840
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_2                         840
#define DEFAULT_GUN_DRV_CATHODE_PER_2                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_3                        840
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_3                         840
#define DEFAULT_GUN_DRV_CATHODE_PER_3                             20000
#define DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_4                        840
#define DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_4                         840
#define DEFAULT_GUN_DRV_CATHODE_PER_4                             20000


#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_A                  76
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_B                  76
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_C                  76
#define DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_D                  76
#define DEFAULT_P_SYNC_HIGH_THYRATRON_DELAY_PER_1                     0
#define DEFAULT_P_SYNC_HIGH_DOSE_SAMPLE_DELAY_PER_1                 100

#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_A                 196
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_B                 196
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_C                 196
#define DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_D                 196
#define DEFAULT_P_SYNC_HIGH_AFC_SAMPLE_DELAY_PER_1                  150
#define DEFAULT_P_SYNC_HIGH_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1    175

#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_A                   76
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_B                   76
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_C                   76
#define DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_D                   76
#define DEFAULT_P_SYNC_LOW_THYRATRON_DELAY_PER_1                      0
#define DEFAULT_P_SYNC_LOW_DOSE_SAMPLE_DELAY_PER_1                  100

#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_A                  196
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_B                  196
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_C                  196
#define DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_D                  196
#define DEFAULT_P_SYNC_LOW_AFC_SAMPLE_DELAY_PER_1                   150
#define DEFAULT_P_SYNC_LOW_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1     175

#define DEFAULT_UNUSED_EEPROM                                         0




#endif
