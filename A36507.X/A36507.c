#include "A36507.h"
#include "FIRMWARE_VERSION.h"
#include "A36507_CONFIG.h"

#define FCY_CLK  20000000


// ------------------ PROCESSOR CONFIGURATION ------------------------//
_FOSC(ECIO_PLL16 & CSW_FSCM_OFF);                                         // 5Mhz External Osc created 20Mhz FCY
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);                                    // 8 Second watchdog timer 
_FBORPOR(PWRT_4 & NONE & PBOR_OFF & MCLR_EN);                             // 4ms Power up timer, Low Voltage Reset disabled
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);  // 
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);         //
_FGS(CODE_PROT_OFF);                                                      //
_FICD(PGD);                                                               //


const unsigned int FilamentLookUpTable[64] = {FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193};

const unsigned int eeprom_default_values_htr_mag_afc[16] = {DEFAULT_MAGNITRON_HEATER_CURRENT,
							    DEFAULT_MAGNET_CURRENT_PER_1,
							    DEFAULT_MAGNET_CURRENT_PER_2,
							    DEFAULT_MAGNET_CURRENT_PER_3,
							    DEFAULT_MAGNET_CURRENT_PER_4,
							    DEFAULT_AFC_HOME_PER_1,
							    DEFAULT_AFC_HOME_PER_2,
							    DEFAULT_AFC_HOME_PER_3,
							    DEFAULT_AFC_HOME_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};

const unsigned int eeprom_default_values_hv_lambda[16]   = {DEFAULT_HV_LAMBDA_HIGH_PER_1,
							    DEFAULT_HV_LAMBDA_LOW_PER_1,
							    DEFAULT_HV_LAMBDA_HIGH_PER_2,
							    DEFAULT_HV_LAMBDA_LOW_PER_2,
							    DEFAULT_HV_LAMBDA_HIGH_PER_3,
							    DEFAULT_HV_LAMBDA_LOW_PER_3,
							    DEFAULT_HV_LAMBDA_HIGH_PER_4,
							    DEFAULT_HV_LAMBDA_LOW_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};

const unsigned int eeprom_default_values_gun_driver[16]  = {DEFAULT_GUN_DRV_HEATER_VOLT,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_1,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_1,
							    DEFAULT_GUN_DRV_CATHODE_PER_1,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_2,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_2,
							    DEFAULT_GUN_DRV_CATHODE_PER_2,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_3,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_3,
							    DEFAULT_GUN_DRV_CATHODE_PER_3,
							    DEFAULT_GUN_DRV_HIGH_PULSE_TOP_PER_4,
							    DEFAULT_GUN_DRV_LOW_PULSE_TOP_PER_4,
							    DEFAULT_GUN_DRV_CATHODE_PER_4,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};

const unsigned int eeprom_default_values_p_sync_per_1[16]= {((DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_HIGH_GRID_DELAY_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_HIGH_DOSE_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_HIGH_THYRATRON_DELAY_PER_1),
							    ((DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_HIGH_GRID_WIDTH_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_HIGH_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_HIGH_AFC_SAMPLE_DELAY_PER_1),
							    ((DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_LOW_GRID_DELAY_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_LOW_DOSE_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_LOW_THYRATRON_DELAY_PER_1),	
							    ((DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_B << 8) + DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_A),
							    ((DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_D << 8) + DEFAULT_P_SYNC_LOW_GRID_WIDTH_PER_1_DOSE_C),
							    ((DEFAULT_P_SYNC_LOW_MAGNETRON_CURRENT_SAMPLE_DELAY_PER_1 << 8) + DEFAULT_P_SYNC_LOW_AFC_SAMPLE_DELAY_PER_1),
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM,
							    DEFAULT_UNUSED_EEPROM};



// ---------------------- Control Functions ---------------------------- //
void DoStateMachine(void);
/*
  This runs the state machine.
  Typical loop time is a couple hundred uS
  This can be extended significantly by spi/i2c communication if waiting for those functions to complete
*/

unsigned int CheckHVOffFault(void); 
/* 
   This will return TRUE (NOT ZERO) if there is a condition that is considered a "fault" when HV is off
   For example a Magnet Over Current would be a fault
   A Lambda not powered would not be a fault (because the lambda is not supposed to be powered when HV is off)
*/

unsigned int CheckFault(void);
/*
  This will return TRUE (NOT ZERO) if there is any fault
*/

unsigned int CheckSystemFault(void);
/*
  This will return TRUE (NOT ZERO) if there is any fault that causes the system to shut itself down till power cycled
  At this time there are no faults that meet this criteria so it always returns 0
*/

unsigned int CheckAllModulesConfigured(void);
/*
  This checks if all modules are connected and configured
  If any module is not connected or not configured it will return TRUE (NOT ZERO)
  This is used to leave the initialization state at startup but is also checked
  while running to look for a missing module
*/

void DoA36507(void);
/*
  This is called every time through the control loop.
  Some tasks are executed every time this function is called
  Some tasks are executed once every 10ms  - those tasks inside "if (_T2IF)"
*/

void ExecuteEthernetCommand(unsigned int personality); 
/*
  This looks to see if there any ethernet commands to execute
  If a command is available this executes that command
  The personality is the personality of the module
  // DPARKER - should not need to include personality
  // DPARKER - need to think about this a bit more. Personality should be stored in RAM
  // DPARKER - what to do if no personality is set??? 
*/

unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage); 
/*
  This caluclates the energy per pulse based on a lambda voltage
  This is just .5*C*V^2 (where C is the PFN capacitance)
*/

void UpdateHeaterScale(void);
/*
  This function grabs the highest lambda set point (from mode A or mode B) and calulates the energy per pulse from that voltage
  Energy per pulse is multiplied by PRF (from Pulse Sync board)
  The energy is used to look up the heater scale factor from the FilamentLookUpTable
  The heater set poing scaled is then set by multipling the set point by the scale factor
*/




// -------------------- STARTUP Helper Functions ---------------------- //
void InitializeA36507(void);
/*
  This initialized the processor and all of it's internal/external hardware and software modules
  It should only be called once
*/

void CalculateHeaterWarmupTimers(void);
/*
  This reads the last time a heater was warm from EEPROM and caluclates how long it needs to be warmed up for
*/

void ReadSystemConfigurationFromEEProm(unsigned int personality);
/*
  This loads the configuration for a given personality from the external EEPROM
*/

void FlashLeds(void);
/*
  LED Helper function
  This is called while the ECB is initializing and causes the LED to flash in the "initializing" pattern
*/




// -------------------------------- Ethernet Interface helper Functions ---------------------------- //
void ZeroSystemPoweredTime(void);
/*
  Sets the system powered, hv on, and xray on second counter to Zero.
*/

void LoadDefaultSystemCalibrationToEEProm(void);
/*
  Loads the default system calibration parmaeters from flash program memmory into the EEPROM
  This is used to restore "factory defaults" if all goes to hell
  DPARKER - need to add pulse sync personalities 2,3,4 
  DPARKER - need to be added to the etherenet interface
*/

void CalculatePulseSyncParams(unsigned char start, unsigned char stop);


void LogModuleFault(unsigned int board_address);
P1395BoardBits board_not_configured_latch;
P1395BoardBits board_not_ready_latch;


// ------------------- Global Variables ------------------------------ //
A36507GlobalVars global_data_A36507;         
/* 
   This is the Data structure for the global variables
   Every variable that is not a structure or on the stack should be within this structure
   This places variables at a fixed place in memory relative to each other which we depend on in some cases
   It also find that this makes it easier to work with debugger
*/


// -------------------- Local Structures ----------------------------- //
RTC_DS3231 U6_DS3231;                        
/* 
   This is the Data structure for the Real Time Clock
   See DS3231.h For more information
*/




int main(void) {
  
  global_data_A36507.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {

    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.control_state = STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    if (_STATUS_LAST_RESET_WAS_POWER_CYCLE) {
      //_SYNC_CONTROL_CLEAR_DEBUG_DATA = 1; // DPAKER changed for debugging
      _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    }
    break;

  case STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _STATUS_PERSONALITY_LOADED = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC);
    while (global_data_A36507.control_state == STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC) {
      DoA36507();
      FlashLeds();

      // DPARKER HOW IS MODULE TYPE BEING RECEIVED FROM PULSE SYNC
      // I SUGGEST 4 bits of the NOT LOGGED REGISTER
#ifdef __IGNORE_PULSE_SYNC_MODULE
      global_data_A36507.personality_select_from_pulse_sync = 255;
#endif
      if (global_data_A36507.personality_select_from_pulse_sync != 0) {
	// a personality has been received from pulse sync board
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
	SendToEventLog(LOG_ID_PERSONALITY_RECEIVED);

#ifndef __SYSTEM_CONFIGURATION_2_5_MEV
	if (global_data_A36507.personality_select_from_pulse_sync >= 5) {
	  global_data_A36507.control_state = STATE_FAULT_SYSTEM;
	  SendToEventLog(LOG_ID_PERSONALITY_ERROR_6_4);
	}
#endif
	
#ifndef __SYSTEM_CONFIGURATION_6_4_MEV
	if (global_data_A36507.personality_select_from_pulse_sync != 255) {
	  global_data_A36507.control_state = STATE_FAULT_SYSTEM;
	  SendToEventLog(LOG_ID_PERSONALITY_ERROR_2_5);
	}
#endif
      }
    }

  case STATE_WAITING_FOR_INITIALIZATION:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _STATUS_PERSONALITY_LOADED = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    ReadSystemConfigurationFromEEProm(global_data_A36507.personality_select_from_pulse_sync);
    // Calculate all of the warmup counters based on previous warmup completed
    CalculateHeaterWarmupTimers();
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION);
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      FlashLeds();
      if (CheckAllModulesConfigured() && (global_data_A36507.startup_counter >= 300)) {
      	global_data_A36507.control_state = STATE_WARMUP;
	SendToEventLog(LOG_ID_ALL_MODULES_CONFIGURED);
      }
    }
    break;
    

  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _FAULT_REGISTER = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_WARMUP);
    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();
      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_WARMUP_DONE);
      }
      if (CheckSystemFault()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;
    
        
  case STATE_STANDBY:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
     while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (!_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
	SendToEventLog(LOG_ID_CUSTOMER_HV_ON);
      }
      if (CheckHVOffFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
     }
    break;


  case STATE_DRIVE_UP:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    global_data_A36507.drive_up_timer = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      // Check to see if the HV Lambda is ready, if it is check all faults and move to ready or fault hold
      if (!_HV_LAMBDA_NOT_READY) {
	if (CheckFault()) {
	  global_data_A36507.control_state = STATE_FAULT_HOLD;
	} else {
	  global_data_A36507.control_state = STATE_READY;
	  SendToEventLog(LOG_ID_DRIVEUP_COMPLETE);
	}
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF);
      }
      if (global_data_A36507.drive_up_timer >= DRIVE_UP_TIMEOUT) {
	_FAULT_DRIVE_UP_TIMEOUT = 1;
	global_data_A36507.control_state = STATE_FAULT_HOLD;
	SendToEventLog(LOG_ID_DRIVE_UP_TIMEOUT);
      }
      if (CheckHVOffFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;
    

  case STATE_READY:
    // Enable XRAYs to Pulse Sync Board
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    SendToEventLog(LOG_ID_ENTERED_STATE_READY);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 1;
    while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF == 0) {
	global_data_A36507.control_state = STATE_XRAY_ON;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_ON);
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF);
      }
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_XRAY_ON:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;    
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
	global_data_A36507.control_state = STATE_READY;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_OFF);
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_CUSTOMER_HV_OFF);
      }
      if (CheckFault()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_FAULT_HOLD:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_FAULT_RESET;
	SendToEventLog(LOG_ID_CUSTOMER_XRAY_OFF);
      }
    }
    break;


  case STATE_FAULT_RESET:
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_RESET) {
      DoA36507();
      _FAULT_REGISTER = 0; // DPARKER IS THIS RIGHT????


      if (CheckHVOffFault() == 0) {
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
	SendToEventLog(LOG_ID_HV_OFF_FAULTS_CLEAR);
      }
      if (CheckSystemFault()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;

    
  case STATE_FAULT_SYSTEM:
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (1) {
      DoA36507();
    }
    break;
    
    
  default:
    global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    break;
  }
}

/*
  Logging Data

  * State Changes                - State changes are logged in the state machine
  * Board Connection Status      - Boards disconnecting and reconnecting is logged by the Can Master Module
  * Board Ready/Not Ready Status - This logged as part of Do10msTicToc
  * Board Faults                 - If a board is faulted ()



*/



unsigned int CheckHVOffFault(void) {
  unsigned int fault = 0;
  
  if (!CheckAllModulesConfigured()) {
    fault = 1;
  }
  
  if (_HEATER_MAGNET_NOT_READY) {
    if (!board_not_ready_latch.heater_magnet_board) {
      // There is a new Heater Magnet not ready event
      SendToEventLog(LOG_ID_NOT_READY_HEATER_MAGNET);
      LogModuleFault(ETM_CAN_ADDR_HEATER_MAGNET_BOARD);
      board_not_ready_latch.heater_magnet_board = 1;
    }
    _FAULT_HTR_MAG_NOT_OPERATE = 1;
#ifndef __IGNORE_HEATER_MAGNET_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.heater_magnet_board) {
      // The heater magnet is reporting that it is ready
      SendToEventLog(LOG_ID_READY_HEATER_MAGNET);
      board_not_ready_latch.heater_magnet_board = 0;
    }
  }
  

  if (_GUN_HEATER_OFF) {
    if (!_FAULT_GUN_HEATER_OFF) {
      // The gun heater has just turned off
      SendToEventLog(LOG_ID_GUN_DRIVER_BOARD_HEATER_OFF);
    }
    _FAULT_GUN_HEATER_OFF = 1;
#ifndef __IGNORE_GUN_DRIVER_MODULE
    fault = 1;
#endif
  } else {
    if (_FAULT_GUN_HEATER_OFF) {
      // The gun has just turned back on 
      SendToEventLog(LOG_ID_GUN_DRIVER_BOARD_HEATER_ON);
    }
    _FAULT_GUN_HEATER_OFF = 0;
  }

  return fault;
}


unsigned int CheckFault(void) {
  unsigned int fault = 0;
  
  if (CheckHVOffFault()) {
    fault = 1;
  }

  // Update the fault status of each of the boards.

  if (_HV_LAMBDA_NOT_READY) {
    if (!board_not_ready_latch.hv_lambda_board) {
      // There is a NEW Lambda fault.
      SendToEventLog(LOG_ID_NOT_READY_HV_LAMBDA_BOARD);
      LogModuleFault(ETM_CAN_ADDR_HV_LAMBDA_BOARD);
      board_not_ready_latch.hv_lambda_board = 1;
    }
    _FAULT_HV_LAMBDA_NOT_OPERATE = 1;
#ifndef __IGNORE_HV_LAMBDA_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.hv_lambda_board) {
      SendToEventLog(LOG_ID_READY_HV_LAMBDA_BOARD);
      board_not_ready_latch.hv_lambda_board = 0;
    }
  }
  
  
  if (_ION_PUMP_NOT_READY) {
    if (!board_not_ready_latch.ion_pump_board) {
      // There is a NEW Ion Pump Fault
      SendToEventLog(LOG_ID_NOT_READY_ION_PUMP_BOARD);
      LogModuleFault(ETM_CAN_ADDR_ION_PUMP_BOARD);
      board_not_ready_latch.ion_pump_board = 1;
    }
    _FAULT_ION_PUMP_NOT_OPERATE = 1;
#ifndef __IGNORE_ION_PUMP_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.ion_pump_board) {
      SendToEventLog(LOG_ID_READY_ION_PUMP_BOARD);
      board_not_ready_latch.ion_pump_board = 0;
    }
  }  


  if (_AFC_NOT_READY) {
    if (!board_not_ready_latch.afc_board) {
      // There is a NEW AFC Fault
      SendToEventLog(LOG_ID_NOT_READY_AFC_BOARD);
      LogModuleFault(ETM_CAN_ADDR_AFC_CONTROL_BOARD);
      board_not_ready_latch.afc_board = 1;
    }
    _FAULT_AFC_NOT_OPERATE = 1;
#ifndef __IGNORE_AFC_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.afc_board) {
      SendToEventLog(LOG_ID_READY_AFC_BOARD);
      board_not_ready_latch.afc_board = 0;
    }
  }  


  if (_COOLING_NOT_READY) {
    if (!board_not_ready_latch.cooling_interface_board) {
      // There is a NEW COOLING Fault
      SendToEventLog(LOG_ID_NOT_READY_COOLING_INTERFACE_BOARD);
      LogModuleFault(ETM_CAN_ADDR_COOLING_INTERFACE_BOARD);
      board_not_ready_latch.cooling_interface_board = 1;
    }
    _FAULT_COOLING_NOT_OPERATE = 1;
#ifndef __IGNORE_COOLING_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.cooling_interface_board) {
      SendToEventLog(LOG_ID_READY_COOLING_INTERFACE_BOARD);
      board_not_ready_latch.cooling_interface_board = 0;
    }
  }  


  if (_GUN_DRIVER_NOT_READY) {
    if (!board_not_ready_latch.gun_driver_board) {
      // There is a NEW COOLING Fault
      SendToEventLog(LOG_ID_NOT_READY_GUN_DRIVER_BOARD);
      LogModuleFault(ETM_CAN_ADDR_GUN_DRIVER_BOARD);
      board_not_ready_latch.gun_driver_board = 1;
    }
    _FAULT_GUN_DVR_NOT_OPERATE = 1;
#ifndef __IGNORE_GUN_DRIVER_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.gun_driver_board) {
      SendToEventLog(LOG_ID_READY_GUN_DRIVER_BOARD);
      board_not_ready_latch.gun_driver_board = 0;
    }
  }  


  if (_PULSE_SYNC_NOT_READY) {
    if (!board_not_ready_latch.pulse_sync_board) {
      // There is a NEW COOLING Fault
      SendToEventLog(LOG_ID_NOT_READY_PULSE_SYNC_BOARD);
      LogModuleFault(ETM_CAN_ADDR_PULSE_SYNC_BOARD);
      board_not_ready_latch.pulse_sync_board = 1;
    }
    _FAULT_PULSE_SYNC_NOT_OPERATE = 1;
#ifndef __IGNORE_PULSE_SYNC_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.pulse_sync_board) {
      SendToEventLog(LOG_ID_READY_PULSE_SYNC_BOARD);
      board_not_ready_latch.pulse_sync_board = 0;
    }
  }  


  if (_PULSE_MON_NOT_READY) {
    if (!board_not_ready_latch.magnetron_current_board) {
      // There is a NEW COOLING Fault
      SendToEventLog(LOG_ID_NOT_READY_PULSE_MONITOR_BOARD);
      LogModuleFault(ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD);
      board_not_ready_latch.magnetron_current_board = 1;
    }
    _FAULT_PULSE_MON_NOT_OPERATE = 1;
#ifndef __IGNORE_PULSE_CURRENT_MODULE
    fault = 1;
#endif
  } else {
    if (board_not_ready_latch.magnetron_current_board) {
      SendToEventLog(LOG_ID_READY_PULSE_MONITOR_BOARD);
      board_not_ready_latch.magnetron_current_board = 0;
    }
  }  
  
  return fault;
}


unsigned int CheckSystemFault(void) {
  // DPARKER SYSTEM FAULT NOT IMPLIMENTED
  return 0;
}


unsigned int CheckAllModulesConfigured(void) {
  unsigned int system_configured;
  
  system_configured = 1;

  if (board_com_fault.hv_lambda_board) {
#ifndef __IGNORE_HV_LAMBDA_MODULE
    system_configured = 0;
#endif
  }
  
  if (_HV_LAMBDA_NOT_CONFIGURED) {
#ifndef __IGNORE_HV_LAMBDA_MODULE
    system_configured = 0;
#endif
    if (!board_not_configured_latch.hv_lambda_board) {
      SendToEventLog(LOG_ID_NOT_CONFIGURED_HV_LAMBDA_BOARD);
    }
    board_not_configured_latch.hv_lambda_board = 1;
  } else {
    if (board_not_configured_latch.hv_lambda_board) {
      SendToEventLog(LOG_ID_CONFIGURED_HV_LAMBDA_BOARD);
    }
    board_not_configured_latch.hv_lambda_board = 0;
  } 
  
  
  if (board_com_fault.ion_pump_board) {
#ifndef __IGNORE_ION_PUMP_MODULE
    system_configured = 0;
#endif
  }
  
  if (_ION_PUMP_NOT_CONFIGURED) {
#ifndef __IGNORE_ION_PUMP_MODULE
    system_configured = 0;
#endif
    if (!board_not_configured_latch.ion_pump_board) {
      SendToEventLog(LOG_ID_NOT_CONFIGURED_ION_PUMP_BOARD);
    }
    board_not_configured_latch.ion_pump_board = 1;
  } else {
    if (board_not_configured_latch.ion_pump_board) {
      SendToEventLog(LOG_ID_CONFIGURED_ION_PUMP_BOARD);
    }
    board_not_configured_latch.ion_pump_board = 0;
  }


  if (board_com_fault.afc_board) {
#ifndef __IGNORE_AFC_MODULE
    system_configured = 0;
#endif  
  }
    
  if (_AFC_NOT_CONFIGURED) {
#ifndef __IGNORE_AFC_MODULE
    system_configured = 0;
#endif  
    if (!board_not_configured_latch.afc_board) {
      SendToEventLog(LOG_ID_NOT_CONFIGURED_AFC_BOARD);
    }
    board_not_configured_latch.afc_board = 1;
  } else {
    if (board_not_configured_latch.afc_board) {
      SendToEventLog(LOG_ID_CONFIGURED_AFC_BOARD);
    }
    board_not_configured_latch.afc_board = 0;
  }




#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if ((board_com_fault.cooling_interface_board) || (_COOLING_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if ((board_com_fault.heater_magnet_board) || (_HEATER_MAGNET_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_GUN_DRIVER_MODULE
  if ((board_com_fault.gun_driver_board) || (_GUN_DRIVER_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if ((board_com_fault.magnetron_current_board) || (_PULSE_MON_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if ((board_com_fault.pulse_sync_board) || (_PULSE_SYNC_NOT_CONFIGURED)) {
    system_configured = 0;
  }
#endif

  return system_configured;

}


#define SEND_BUFFER_A            1
#define SEND_BUFFER_B            0


#define DEBUG_ETMMODBUS



void DoA36507(void) {
#ifdef DEBUG_ETMMODBUS
  static MODBUS_RESP_SMALL etmmodbus_test[4];
  static unsigned etmmodbus_index = 0;
  static unsigned etmmodbus_relay_set_index = 0;
  static unsigned etmmodbus_relay_toggle = 0;
#endif 
  //_SYNC_CONTROL_WORD                 = 0xF0F0;
  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = global_data_A36507.control_state;
  etm_can_master_sync_message.sync_2 = 0x0123;
  etm_can_master_sync_message.sync_3 = 0x4567;

  ETMCanMasterDoCan();
  TCPmodbus_task();
  ExecuteEthernetCommand(1);  // DPARKER This is using personality 1, should read from pulse sync

#ifndef __IGNORE_TCU
  ETMmodbus_task();
#endif

  if ((global_data_A36507.buffer_a_ready_to_send) & (!global_data_A36507.buffer_a_sent)) {
    SendPulseData(SEND_BUFFER_A);
    global_data_A36507.buffer_a_sent = 1;
  }

  if ((global_data_A36507.buffer_b_ready_to_send) & (!global_data_A36507.buffer_b_sent)) {
    SendPulseData(SEND_BUFFER_B);
    global_data_A36507.buffer_b_sent = 1;
  }


  // Check to see if cooling is present
  _SYNC_CONTROL_COOLING_FAULT = 0;
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (board_com_fault.cooling_interface_board) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
    _FAULT_COOLING_NOT_CONNECTED = 1;
  }
  if (_COOLING_NOT_READY) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
    _FAULT_COOLING_NOT_READY = 1;
  }
#endif
  /*
  debug_data_ecb.debug_0 = global_data_A36507.control_state;
  debug_data_ecb.debug_1 = global_data_A36507.event_log_counter;
  debug_data_ecb.debug_2 = (unsigned int)(global_data_A36507.time_seconds_now >> 16);
  debug_data_ecb.debug_3 = (unsigned int)(global_data_A36507.time_seconds_now & 0x0000FFFF);
  debug_data_ecb.debug_4 = global_data_A36507.no_connect_count_ion_pump_board;
  debug_data_ecb.debug_5 = global_data_A36507.no_connect_count_magnetron_current_board;
  debug_data_ecb.debug_6 = global_data_A36507.no_connect_count_pulse_sync_board;
  debug_data_ecb.debug_7 = global_data_A36507.no_connect_count_hv_lambda_board;
  debug_data_ecb.debug_8 = global_data_A36507.no_connect_count_afc_board;
  debug_data_ecb.debug_9 = global_data_A36507.no_connect_count_cooling_interface_board;
  debug_data_ecb.debug_A = global_data_A36507.no_connect_count_heater_magnet_board;
  debug_data_ecb.debug_B = global_data_A36507.no_connect_count_gun_driver_board;
  debug_data_ecb.debug_C = etm_can_next_pulse_level;
  debug_data_ecb.debug_D = etm_can_next_pulse_count;

  */

  local_data_ecb.log_data[0] = global_data_A36507.control_state;
  *(unsigned long*)&local_data_ecb.log_data[1] = global_data_A36507.time_seconds_now;
  local_data_ecb.log_data[3] = ETMCanMasterGetPulsePRF();



  //mirror_pulse_mon.log_data[4] = ;
  //mirror_pulse_mon.log_data[5] = ;
  //mirror_pulse_mon.log_data[6] = ;
  mirror_pulse_mon.log_data[7] = _SYNC_CONTROL_WORD;

  *(unsigned long*)&local_data_ecb.log_data[8] = global_data_A36507.system_powered_seconds;
  *(unsigned long*)&local_data_ecb.log_data[10] = global_data_A36507.system_hv_on_seconds;
  
  *(unsigned long*)&local_data_ecb.log_data[12] = global_data_A36507.system_xray_on_seconds;
  local_data_ecb.log_data[14] = 0;
  local_data_ecb.log_data[15] = *(unsigned int*)&board_com_fault;


  local_data_ecb.local_data[0] = 300;
  local_data_ecb.local_data[1] = 301;
  local_data_ecb.local_data[2] = 302;
  local_data_ecb.local_data[3] = 303;

  local_data_ecb.local_data[4] = 304;
  local_data_ecb.local_data[5] = 305;
  local_data_ecb.local_data[6] = 306;
  local_data_ecb.local_data[7] = 307;

  local_data_ecb.local_data[8] = 308;
  local_data_ecb.local_data[9] = 309;
  local_data_ecb.local_data[10] = 310;
  local_data_ecb.local_data[11] = 311;

  local_data_ecb.local_data[12] = 312;
  local_data_ecb.local_data[13] = 313;
  local_data_ecb.local_data[14] = 314;
  local_data_ecb.local_data[15] = 315;



  debug_data_ecb.debug_reg[0]  = 0; 
  debug_data_ecb.debug_reg[1]  = 1; 
  debug_data_ecb.debug_reg[2]  = 2; 
  debug_data_ecb.debug_reg[3]  = 3; 

  debug_data_ecb.debug_reg[4]  = 4; 
  debug_data_ecb.debug_reg[5]  = 5; 
  debug_data_ecb.debug_reg[6]  = 6; 
  debug_data_ecb.debug_reg[7]  = 7; 

  debug_data_ecb.debug_reg[8]  = 8; 
  debug_data_ecb.debug_reg[9]  = 9; 
  debug_data_ecb.debug_reg[10] = 10; 
  debug_data_ecb.debug_reg[11] = 11; 

  debug_data_ecb.debug_reg[12] = 12; 
  debug_data_ecb.debug_reg[13] = 13; 
  debug_data_ecb.debug_reg[14] = 14; 
  debug_data_ecb.debug_reg[15] = 15; 


  if (_T2IF) {
    // 10ms Timer has expired
    _T2IF = 0;
    global_data_A36507.millisecond_counter += 10;
    
#ifndef __IGNORE_TCU
    if (ETMmodbus_timer_10ms < 60000) ETMmodbus_timer_10ms++;
    
#ifdef DEBUG_ETMMODBUS
    if (global_data_A36507.millisecond_counter == 500) {
      switch (etmmodbus_index) 
	{
	case 0:
	  ETMModbusReadCoilsSmall(0x177d, 3, &etmmodbus_test[etmmodbus_index]);  // T14-T16
	  break;
	case 1:
	  ETMModbusReadCoilsSmall(0x17e0, 10, &etmmodbus_test[etmmodbus_index]);	// T113-T122
	  break;
	case 2:
	  ETMModbusReadHoldingRegistersSmall(0xfa6, 1, &etmmodbus_test[etmmodbus_index]);	// R1007, temp read
	  break;
	case 3:
	  ETMModbusWriteSingleCoil(0x177d + etmmodbus_relay_set_index, etmmodbus_relay_toggle, &etmmodbus_test[etmmodbus_index]);
	  etmmodbus_relay_toggle = etmmodbus_relay_toggle? 0: 1;
	  if (etmmodbus_relay_toggle == 0) {
	    etmmodbus_relay_set_index += 1;
	    if (etmmodbus_relay_set_index > 2) etmmodbus_relay_set_index = 0;
	  }
	  break;
	  default:
	    break;
	}
      etmmodbus_index	+= 1;
      if (etmmodbus_index >= 4) etmmodbus_index = 0;
      
      // check readbacks 
      if (etmmodbus_test[0].done) {
	if (etmmodbus_test[0].done == ETMMODBUS_RESPONSE_OK) {
	  //local_debug_data.debug_E &= 0xfff0;
	}
	//local_debug_data.debug_E |= etmmodbus_test[0].data & 0x0f;
	etmmodbus_test[0].done = 0;
      }
      if (etmmodbus_test[1].done) {
	if (etmmodbus_test[1].done == ETMMODBUS_RESPONSE_OK) {
	  //local_debug_data.debug_E &= 0x000f;
	}
	//local_debug_data.debug_E |= (etmmodbus_test[1].data << 4) & 0xfff0;
	etmmodbus_test[1].done = 0;
      }
      if (etmmodbus_test[2].done) {
	if (etmmodbus_test[2].done == ETMMODBUS_RESPONSE_OK) {
	  //local_debug_data.debug_F = etmmodbus_test[2].data;
	}
	etmmodbus_test[2].done = 0;
      }
    }
#endif    
#endif
    
    // Copy data from global variable strucutre to strucutre that gets sent to GUI

    //DPARKER figure out some way to do this
    /*
    etm_can_ethernet_board_data.mirror_sync_0_control_word = *(unsigned int*)&etm_can_sync_message.sync_0_control_word;
    etm_can_ethernet_board_data.mirror_control_state = global_data_A36507.control_state;
    etm_can_ethernet_board_data.mirror_system_powered_seconds = global_data_A36507.system_powered_seconds;
    etm_can_ethernet_board_data.mirror_system_hv_on_seconds = global_data_A36507.system_hv_on_seconds;
    etm_can_ethernet_board_data.mirror_system_xray_on_seconds = global_data_A36507.system_xray_on_seconds;
    etm_can_ethernet_board_data.mirror_time_seconds_now = global_data_A36507.time_seconds_now;
    etm_can_ethernet_board_data.mirror_average_output_power_watts = global_data_A36507.average_output_power_watts;
    etm_can_ethernet_board_data.mirror_thyratron_warmup_counter_seconds = global_data_A36507.thyratron_warmup_counter_seconds;
    etm_can_ethernet_board_data.mirror_magnetron_heater_warmup_counter_seconds = global_data_A36507.magnetron_heater_warmup_counter_seconds;
    etm_can_ethernet_board_data.mirror_gun_driver_heater_warmup_counter_seconds = global_data_A36507.gun_driver_heater_warmup_counter_seconds;
    etm_can_ethernet_board_data.mirror_board_com_fault = *(unsigned int*)&board_com_fault;
    */
    if (global_data_A36507.control_state == STATE_DRIVE_UP) {
      global_data_A36507.drive_up_timer++;
    }

    if ((global_data_A36507.control_state == STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC) || (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION)) {
      global_data_A36507.startup_counter++;
    }

    // DPARKER Check for cooling fault, and set the sync bit message as appropriate

    
    // Update the heater current based on Output Power
    UpdateHeaterScale();



    /*
      The following tasks require use of the i2c bus which can hold the processor for a lot of time
      Need to schedule them at different point of a 1 second period
    */

    // Run at 1 second interval
    if (global_data_A36507.millisecond_counter >= 1000) {
      global_data_A36507.millisecond_counter = 0;
    }

    // Run once a second at 0 milliseconds
    if (global_data_A36507.millisecond_counter == 0) {
      // Read Date/Time from RTC and update the warmup up counters
      ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
      global_data_A36507.time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

      // Update the warmup counters
      if (thyratron_warmup_counter_seconds > 0) {
	thyratron_warmup_counter_seconds--;
      } else {
	global_data_A36507.thyratron_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
      }
      
      if ((!board_com_fault.heater_magnet_board) && (!_HEATER_MAGNET_NOT_READY)) {
	// The Magnetron heater is on
	if (magnetron_heater_warmup_counter_seconds > 0) {
	  magnetron_heater_warmup_counter_seconds--;
	} else {
	  global_data_A36507.magnetron_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
	}
      } else {
	magnetron_heater_warmup_counter_seconds += 2;
	if (magnetron_heater_warmup_counter_seconds >= MAGNETRON_HEATER_WARM_UP_TIME) {
	  magnetron_heater_warmup_counter_seconds = MAGNETRON_HEATER_WARM_UP_TIME;
	}
      }
	
      if (!board_com_fault.gun_driver_board && !_GUN_HEATER_OFF) {
	// The gun heater is on
	if (gun_driver_heater_warmup_counter_seconds > 0) {
	  gun_driver_heater_warmup_counter_seconds--;
	} else {
	  global_data_A36507.gun_driver_heater_last_warm_seconds = global_data_A36507.time_seconds_now;
	}
      } else {
	gun_driver_heater_warmup_counter_seconds += 2;
	if (gun_driver_heater_warmup_counter_seconds >= GUN_DRIVER_HEATER_WARM_UP_TIME) {
	  gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_HEATER_WARM_UP_TIME;
	}
      }
      // Check for warmup done
      
#ifdef __IGNORE_HEATER_MAGNET_MODULE
      magnetron_heater_warmup_counter_seconds = 0;
#endif
      
#ifdef __IGNORE_GUN_DRIVER_MODULE
      gun_driver_heater_warmup_counter_seconds = 0;
#endif
      
      if ((thyratron_warmup_counter_seconds) || (magnetron_heater_warmup_counter_seconds) || (gun_driver_heater_warmup_counter_seconds)) {
	global_data_A36507.warmup_done = 0;
      } else {
	global_data_A36507.warmup_done = 1;
      }
    } // End of tasks that happen when millisecond = 0
    

    // Run once a second at 250 milliseconds
    if (global_data_A36507.millisecond_counter == 250) {
      // Write Warmup Done Timers to EEPROM
      ETMEEPromWritePage(EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
    } // End of tasks that happen when millisecond = 250
    

    // Run once a second at 500 milliseconds
    if (global_data_A36507.millisecond_counter == 500) {
      // Write Seconds on Counters to EEPROM
      global_data_A36507.system_powered_seconds++;
      
      if (global_data_A36507.control_state == STATE_READY) {
	global_data_A36507.system_hv_on_seconds++;
      }
      
      if (global_data_A36507.control_state == STATE_XRAY_ON) {
	global_data_A36507.system_hv_on_seconds++;
	global_data_A36507.system_xray_on_seconds++;
      }
      ETMEEPromWritePage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);
    } // End of tasks that happen when millisecond = 500
  
  } // End of 10ms Tasks
}


// DPARKER - This will change if we use a new PFN with a different capacitance
unsigned int CalculatePulseEnergyMilliJoules(unsigned int lambda_voltage) {
  unsigned long power_milli_joule;
  unsigned int return_data;

  /*
    The Pulse Energy is Calculated for Each Pulse
    The Pulse Energy is then multiplied by the PRF to generate the power.
    The filament heater voltage is generated from the power.

    Power = 1/2 * C * V^2
    C = 90nF
    In Floating Point Math
    power(milli_joule) = .5 * 90e-9 * V^2 * 1000

    power_milli_joule = .5 * 90e-9 * V^2 * 1000
                      = v^2/22222.22
		      = v*v / 2^6 / 347.22
		      = v*v / 2^6 * 47 / 2^14 (.4% fixed point error)
		      
  */
  power_milli_joule = lambda_voltage;
  power_milli_joule *= lambda_voltage;
  power_milli_joule >>= 6;
  power_milli_joule *= 47;
  power_milli_joule >>= 14;

  if (power_milli_joule >= 0xFFFF) {
    power_milli_joule = 0xFFFF;
  }
  power_milli_joule &= 0xFFFF;

  return_data = power_milli_joule;

  return return_data;
}


void UpdateHeaterScale() {
  unsigned long temp32;
  unsigned int temp16;

  // Load the energy per pulse into temp32
  // Use the higher of High/Low Energy set point
  if (local_hv_lambda_high_en_set_point > local_hv_lambda_low_en_set_point) {
    temp32 = CalculatePulseEnergyMilliJoules(local_hv_lambda_high_en_set_point);
  } else {
    temp32 = CalculatePulseEnergyMilliJoules(local_hv_lambda_low_en_set_point);
  }
  
  // Multiply the Energy per Pulse times the PRF (in deci-Hz)
  temp32 *= ETMCanMasterGetPulsePRF();
  if (global_data_A36507.control_state != STATE_XRAY_ON) {
    // Set the power to zero if we are not in the X-RAY ON state
    temp32 = 0;
  }

  temp32 >>= 6;
  temp32 *= 13;
  temp32 >>= 11;  // Temp32 is now Magnetron Power (in Watts)
  
  global_data_A36507.average_output_power_watts = temp32;
  temp16 = global_data_A36507.average_output_power_watts;

  temp16 >>= 7; // Convert to index for our rolloff table
  if (temp16 >= 0x3F) {
    // Prevent Rollover of the index
    // This is a maximum magnitron power of 8064 Watts
    // If the Magnritron power is greater thatn 8064 it will rolloff as if the power was 8064 watts
    // This would happen at a lambda voltage of 21.2 KV which is well above the maximum voltage of the Lambda
    temp16 = 0x3F;
  }
  
  local_heater_current_scaled_set_point = ETMScaleFactor16(local_heater_current_full_set_point,
							   FilamentLookUpTable[temp16],
							   0);
}


void InitializeA36507(void) {
  unsigned int loop_counter;




  _FAULT_REGISTER      = 0;
  _CONTROL_REGISTER    = 0;
  _WARNING_REGISTER    = 0;
  _NOT_LOGGED_REGISTER = 0;

  // Set the not connected bits for all boards
  board_com_fault.hv_lambda_board         = 1;
  board_com_fault.ion_pump_board          = 1;
  board_com_fault.afc_board               = 1;
  board_com_fault.cooling_interface_board = 1;
  board_com_fault.heater_magnet_board     = 1;
  board_com_fault.gun_driver_board        = 1;
  board_com_fault.magnetron_current_board = 1;
  board_com_fault.pulse_sync_board        = 1;

    
  // Check it reset was a result of full power cycle
  _STATUS_LAST_RESET_WAS_POWER_CYCLE = 0;
  if (PIN_IN_ETM_RESET_DETECT) {
    // The power was off for more than a couple hundered milliseconds
    // All values in RAM are random.
    _STATUS_LAST_RESET_WAS_POWER_CYCLE = 1;
  }


  // Initialize all I/O Registers
  TRISA = A36507_TRISA_VALUE;
  TRISB = A36507_TRISB_VALUE;
  TRISC = A36507_TRISC_VALUE;
  TRISD = A36507_TRISD_VALUE;
  TRISF = A36507_TRISF_VALUE;
  TRISG = A36507_TRISG_VALUE;


  // Initialize the reset detect circuit
  TRIS_PIN_ETM_RESET_RETECT = 0;  // Pin is an output
  PIN_OUT_ETM_RESET_DETECT = 0;   // Pin is low so that reset detect capacitor is charged 

  // Initialize TMR2
  PR2   = PR2_VALUE_10_MILLISECONDS;
  TMR2  = 0;
  _T2IF = 0;
  T2CON = T2CON_VALUE;

  // manually clock out I2C CLK
  // DPARKER make this a generic reset I2C Function
  
  _TRISG2 = 0; // g2 is output
  for (loop_counter = 0; loop_counter <= 100; loop_counter++) {
    _LATG2 = 0;
    __delay32(25);
    _LATG2 = 1;
    __delay32(25);
  }

  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD, EEPROM_I2C_ADDRESS_0, 1);
  ConfigureDS3231(&U6_DS3231, I2C_PORT, RTC_DEFAULT_CONFIG, FCY_CLK, ETM_I2C_400K_BAUD);

#define AGILE_REV 77
#define SERIAL_NUMBER 100 

  // Initialize the Can module
  ETMCanMasterInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_ETHERNET_BOARD, _PIN_RG13, 4);
  ETMCanMasterLoadConfiguration(36507, 0, AGILE_REV, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, SERIAL_NUMBER);
  
  // Initialize TCPmodbus Module
  TCPmodbus_init();

#ifndef __IGNORE_TCU
  // Initialize ETMmodbus Module
  ETMmodbus_init();
#endif

  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCSSL = ADCSSL_SETTING;

  _ADIF = 0;
  _ADON = 1;

  // Wait for data to be read
  while (_ADIF == 0);

  global_data_A36507.analog_input_5v_mon.filtered_adc_reading  = ADCBUF0 + ADCBUF2 + ADCBUF4 + ADCBUF6 + ADCBUF8 + ADCBUFA + ADCBUFC + ADCBUFE;
  global_data_A36507.analog_input_3v3_mon.filtered_adc_reading = ADCBUF1 + ADCBUF3 + ADCBUF5 + ADCBUF7 + ADCBUF9 + ADCBUFB + ADCBUFD + ADCBUFF;

  global_data_A36507.analog_input_5v_mon.filtered_adc_reading  <<= 1;
  global_data_A36507.analog_input_3v3_mon.filtered_adc_reading <<= 1;  

  ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_5v_mon);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36507.analog_input_3v3_mon);

  
  _ADON = 0;

  // Load System powered time from EEPROM
  ETMEEPromReadPage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&global_data_A36507.system_powered_seconds);
}


void CalculateHeaterWarmupTimers(void) {
  unsigned long difference;
  // Read the warmup timers stored in EEPROM
  ETMEEPromReadPage(EEPROM_PAGE_HEATER_TIMERS, 6, (unsigned int*)&global_data_A36507.magnetron_heater_last_warm_seconds);
  ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  global_data_A36507.time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

  // Calculate new magnetron heater warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.magnetron_heater_last_warm_seconds;
  if (difference > (MAGNETRON_HEATER_WARM_UP_TIME >> 1)) {
    magnetron_heater_warmup_counter_seconds = MAGNETRON_HEATER_WARM_UP_TIME;    
  } else {
    magnetron_heater_warmup_counter_seconds = (difference << 1);
  }

  // Calculate new thyratron warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.thyratron_heater_last_warm_seconds;
  if (difference > (THYRATRON_WARM_UP_TIME >> 1)) {
    thyratron_warmup_counter_seconds = THYRATRON_WARM_UP_TIME;    
  } else {
    thyratron_warmup_counter_seconds = (difference << 1);
  }
  
  // Calculate new gun driver heater warm up time remaining
  difference = global_data_A36507.time_seconds_now - global_data_A36507.gun_driver_heater_last_warm_seconds;
  if (difference > (GUN_DRIVER_HEATER_WARM_UP_TIME >> 1)) {
    gun_driver_heater_warmup_counter_seconds = GUN_DRIVER_HEATER_WARM_UP_TIME;
  } else {
    gun_driver_heater_warmup_counter_seconds = (difference << 1);
  }
}


void ReadSystemConfigurationFromEEProm(unsigned int personality) {
  if (personality >= 5) {
    personality = 1;
  }
  if (personality) {
    personality--;  // Personality is now a register offset
  }
  
  // Load data for HV Lambda
  local_hv_lambda_low_en_set_point    = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT + (2*personality)));
  local_hv_lambda_high_en_set_point   = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT + (2*personality)));

  // Load data for AFC
  local_afc_home_position             = ETMEEPromReadWord((EEPROM_REGISTER_AFC_HOME_POSITION + personality));
  local_afc_aft_control_voltage       = ETMEEPromReadWord(EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE);
  // etm_can_afc_mirror.afc_offset = ETMEEPromReadWord(EEPROM_REGISTER_AFC_OFFSET);

  
  // Load Data for Heater/Magnet Supply
  local_heater_current_full_set_point = ETMEEPromReadWord(EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT);
  local_magnet_current_set_point      = ETMEEPromReadWord((EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT + personality));
  
  // Load data for Gun Driver
  local_gun_drv_heater_v_set_point    = ETMEEPromReadWord(EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE);
  local_gun_drv_high_en_pulse_top_v   = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP + (3*personality)));
  local_gun_drv_low_en_pulse_top_v    = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP + (3*personality)));
  local_gun_drv_cathode_set_point     = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_CATHODE + (3*personality)));

  // Load data for Pulse Sync
  ETMEEPromReadPage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1 + personality), 12, (unsigned int*)&local_pulse_sync_timing_reg_0_word_0);
}


void FlashLeds(void) {
  switch (((global_data_A36507.startup_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = !OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_OUT_ETM_LED_OPERATIONAL_GREEN = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_A_RED = OLL_LED_ON;
    PIN_OUT_ETM_LED_TEST_POINT_B_GREEN = OLL_LED_ON;
    break;
  }
}


void ZeroSystemPoweredTime(void) {
  // These values will be written to EEPROM sometime in the next second.
  global_data_A36507.system_powered_seconds = 0;
  global_data_A36507.system_hv_on_seconds = 0;
  global_data_A36507.system_xray_on_seconds = 0;
}


void LoadDefaultSystemCalibrationToEEProm(void) {
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC, 16, (unsigned int*)&eeprom_default_values_htr_mag_afc);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA, 16, (unsigned int*)&eeprom_default_values_hv_lambda);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV, 16, (unsigned int*)&eeprom_default_values_gun_driver);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, (unsigned int*)&eeprom_default_values_p_sync_per_1);
}



#define REGISTER_HEATER_CURRENT_AT_STANDBY                                                 0x0000
#define REGISTER_ELECTROMAGNET_CURRENT                                                     0x0001
#define REGISTER_HOME_POSITION                                                             0x0005
#define REGISTER_AFC_OFFSET                                                                0x0009
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE                                                   0x000A
#define REGISTER_HIGH_ENERGY_SET_POINT                                                     0x0010
#define REGISTER_LOW_ENERGY_SET_POINT                                                      0x0011
#define REGISTER_GUN_DRIVER_HEATER_VOLTAGE                                                 0x0020
#define REGISTER_GUN_DRIVER_HIGH_ENERGY_PULSE_TOP_VOLTAGE                                  0x0021
#define REGISTER_GUN_DRIVER_LOW_ENERGY_PULSE_TOP_VOLTAGE                                   0x0022
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE                                                0x0023

#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_A_B                               0x0030
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_C_D                               0x0031
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_HIGH_ENERGY               0x0032

#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_A_B                               0x0033
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_C_D                               0x0034
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_HIGH_ENERGY                          0x0035

#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_A_B                                0x0036
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_C_D                                0x0037
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_LOW_ENERGY                0x0038

#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_A_B                                0x0039
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_C_D                                0x003A
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_LOW_ENERGY                           0x003B

#define REGISTER_CMD_AFC_SELECT_AFC_MODE                                                   0x5081
#define REGISTER_CMD_AFC_SELECT_MANUAL_MODE                                                0x5082
#define REGISTER_CMD_AFC_MANUAL_TARGET_POSITION                                            0x5083
#define REGISTER_CMD_AFC_MANUAL_MOVE                                                       0x5084

#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES                            0x6082
#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT          0x6083
#define REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE                            0x6084
#define REGISTER_SPECIAL_ECB_LOAD_DEFAULT_SETTINGS_TO_EEPROM_AND_REBOOT                    0xE080
#define REGISTER_SPECIAL_ECB_RESET_ARC_AND_PULSE_COUNT                                     0xE081
#define REGISTER_SPECIAL_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON                           0xE082
#define REGISTER_SPECIAL_ECB_RESET_SLAVE                                                   0xE083
#define REGISTER_SPECIAL_ECB_SEND_SLAVE_RELOAD_EEPROM_WITH_DEFAULTS                        0xE084



#define REGISTER_DEBUG_TOGGLE_RESET                                                        0xEF00
#define REGISTER_DEBUG_TOGGLE_HIGH_SPEED_LOGGING                                           0xEF01
#define REGISTER_DEBUG_TOGGLE_HV_ENABLE                                                    0xEF02
#define REGISTER_DEBUG_TOGGLE_XRAY_ENABLE                                                  0xEF03
#define REGISTER_DEBUG_TOGGLE_COOLING_FAULT                                                0xEF04
#define REGISTER_DEBUG_TOGGLE_RESET_DEBUG                                                  0xEF05
#define REGISTER_DEBUG_ENABLE_HIGH_SPEED_LOGGING                                           0xEF06
#define REGISTER_DEBUG_DISABLE_HIGH_SPEED_LOGGING                                          0xEF07

#define REGISTER_SPECIAL_SET_TIME                                                          0xEF08

#define REGISTER_SPECIAL_2_5_SET_GRID_START                                                0xEF40
#define REGISTER_SPECIAL_2_5_SET_GRID_STOP                                                 0xEF41
#define REGISTER_SPECIAL_2_5_SET_PFN_DELAY                                                 0xEF42
#define REGISTER_SPECIAL_2_5_SET_DOSE_SAMPLE_DELAY                                         0xEF43  // Unused for the 2.5 - NOT IMPLIMENTED
#define REGISTER_SPECIAL_2_5_SET_AFC_SAMPLE_DELAY                                          0xEF44
#define REGISTER_SPECIAL_2_5_SET_MAGNETRON_CURRENT_SAMPLE_DELAY                            0xEF45
#define REGISTER_SPECIAL_2_5_SET_HV_LAMBDA_VOLTAGE                                         0xEF46
#define REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_START                                        0xEF47
#define REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_STOP                                         0xEF48



void ExecuteEthernetCommand(unsigned int personality) {
  ETMEthernetMessageFromGUI next_message;
  unsigned int eeprom_register;
  unsigned int temp;
  unsigned int temp_array[12];


  //unsigned long temp_long;
  //RTC_TIME set_time;


  // DPARKER what happens if this is called before personality has been read??? 
  // Easy to solve in the state machine, just don't call until state when the personality is known

  if (personality >= 5) {
    personality = 1;
  }
  if (personality) {
    personality--;  // Personality is now a register offset
  }
  next_message = GetNextMessage();
  if (next_message.index == 0xFFFF) {
    // there was no message
    return;
  }
  
  if ((next_message.index & 0x0F00) == 0x0100) {
    // this is a calibration set message, route to appropriate board
    // DPARKER only allow when customer has not commanded high voltage on
    SendCalibrationSetPointToSlave(next_message.index, next_message.data_1, next_message.data_0);
  } else if ((next_message.index & 0x0F00) == 0x0900) {
    // this is a calibration requestion message, route to appropriate board
    // When the response is received, the data will be transfered to the GUI
    // DPARKER only allow when customer has not commanded high voltage on
    ReadCalibrationSetPointFromSlave(next_message.index);
  } else {
    // This message needs to be processsed by the ethernet control board
    switch (next_message.index) {
    case REGISTER_HEATER_CURRENT_AT_STANDBY:
      local_heater_current_full_set_point = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_ELECTROMAGNET_CURRENT:
      local_magnet_current_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_HOME_POSITION:
      local_afc_home_position = next_message.data_2;
      eeprom_register = next_message.index + personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_AFC_OFFSET:
      //etm_can_afc_mirror.afc_offset = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;
      
    case REGISTER_AFC_AFT_CONTROL_VOLTAGE:
      local_afc_aft_control_voltage = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_HIGH_ENERGY_SET_POINT:
      local_hv_lambda_high_en_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      // DPARKER figure out how this is going to work - voltage or current programming
      break;

    case REGISTER_LOW_ENERGY_SET_POINT:
      local_hv_lambda_low_en_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      // DPARKER figure out how this is going to work - voltage or current programming
      break;

    case REGISTER_GUN_DRIVER_HEATER_VOLTAGE:
      local_gun_drv_heater_v_set_point = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_HIGH_ENERGY_PULSE_TOP_VOLTAGE:
      local_gun_drv_high_en_pulse_top_v = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_LOW_ENERGY_PULSE_TOP_VOLTAGE:
      local_gun_drv_low_en_pulse_top_v = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_GUN_DRIVER_CATHODE_VOLTAGE:
      local_gun_drv_cathode_set_point = next_message.data_2;
      eeprom_register = next_message.index + personality * 3;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_A_B:
      local_pulse_sync_timing_reg_0_word_0 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_C_D:
      local_pulse_sync_timing_reg_0_word_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_HIGH_ENERGY:
      local_pulse_sync_timing_reg_0_word_2 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_A_B:
      local_pulse_sync_timing_reg_1_word_0 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_C_D:
      local_pulse_sync_timing_reg_1_word_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_HIGH_ENERGY:
      local_pulse_sync_timing_reg_1_word_2 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_A_B:
      local_pulse_sync_timing_reg_2_word_0 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_C_D:
      local_pulse_sync_timing_reg_2_word_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_LOW_ENERGY:
      local_pulse_sync_timing_reg_2_word_2 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_A_B:
      local_pulse_sync_timing_reg_3_word_0 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_C_D:
      local_pulse_sync_timing_reg_3_word_1 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_LOW_ENERGY:
      local_pulse_sync_timing_reg_3_word_2 = next_message.data_2;
      eeprom_register = next_message.index + personality * 0x10;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_CMD_AFC_SELECT_AFC_MODE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SELECT_AFC_MODE,
			  0,
			  0,
			  0);
      break;

    case REGISTER_CMD_AFC_SELECT_MANUAL_MODE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SELECT_MANUAL_MODE,
			  0,
			  0,
			  0);
      break;

    case REGISTER_CMD_AFC_MANUAL_TARGET_POSITION:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_AFC_CONTROL_BOARD << 2)),
			  ETM_CAN_REGISTER_AFC_CMD_SET_MANUAL_TARGET_POSITION,
			  0,
			  0,
			  next_message.data_2);
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_SF6_PULSE_LIMIT_OVERRIDE,
			  0,
			  0,
			  0);
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_SF6_LEAK_LIMIT_OVERRIDE,
			  0,
			  0,
			  next_message.data_2);
      break;

    case REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_COOLING_INTERFACE_BOARD << 2)),
			  ETM_CAN_REGISTER_COOLING_CMD_RESET_BOTTLE_COUNT,
			  0,
			  0,
			  next_message.data_2);

    case REGISTER_SPECIAL_ECB_RESET_ARC_AND_PULSE_COUNT:
      // DPARKER the command to do this is not yet part of the CAN library
      // This will require an extension of the can library to impliment
      break;

    case REGISTER_SPECIAL_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON:
      ZeroSystemPoweredTime();      
      break;

    case REGISTER_SPECIAL_ECB_LOAD_DEFAULT_SETTINGS_TO_EEPROM_AND_REBOOT:
      // DPARKER only allow when customer has not commanded high voltage on
      LoadDefaultSystemCalibrationToEEProm();
      __delay32(1000000);
      __asm__ ("Reset");
      break;

    case REGISTER_SPECIAL_ECB_SEND_SLAVE_RELOAD_EEPROM_WITH_DEFAULTS:
      // DPARKER only allow when customer has not commanded high voltage on
      SendSlaveLoadDefaultEEpromData(next_message.data_2);
      break;
    
    case REGISTER_SPECIAL_ECB_RESET_SLAVE:
      SendSlaveReset(next_message.data_2);
      break;

    /*
    case REGISTER_SPECIAL_SEND_ALL_CAL_DATA_TO_GUI:
      // DPARKER only allow when customer has not commanded high voltage on
      SendSlaveUploadAllCalData(next_message.data_2);
      break;
    */

    case REGISTER_DEBUG_TOGGLE_RESET:
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_SYNC_CONTROL_RESET_ENABLE = 0;
      } else {
	_SYNC_CONTROL_RESET_ENABLE = 1;
      }
      break;

    case REGISTER_SPECIAL_2_5_SET_GRID_START:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      temp = (temp << 8) + temp;
      temp_array[0] = temp;
      temp_array[1] = temp;
      temp_array[6] = temp;
      temp_array[7] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      local_pulse_sync_timing_reg_0_word_0 = temp;
      local_pulse_sync_timing_reg_0_word_1 = temp;
      local_pulse_sync_timing_reg_2_word_0 = temp;
      local_pulse_sync_timing_reg_2_word_1 = temp;
      break;

    case REGISTER_SPECIAL_2_5_SET_GRID_STOP:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      temp = (temp << 8) + temp;
      temp_array[3] = temp;
      temp_array[4] = temp;
      temp_array[9] = temp;
      temp_array[10] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      local_pulse_sync_timing_reg_1_word_0 = temp;
      local_pulse_sync_timing_reg_1_word_1 = temp;
      local_pulse_sync_timing_reg_3_word_0 = temp;
      local_pulse_sync_timing_reg_3_word_1 = temp;
      break;

      // DPARKER BRING ALL OF THESE BACK
      /*
    case REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_START:
      CalculatePulseSyncParams(next_message.data_2, etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_3);
      break;

    case REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_STOP:
      CalculatePulseSyncParams(etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_3, next_message.data_2);
      break;


    case REGISTER_SPECIAL_2_5_SET_PFN_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      temp <<= 8;
      temp += (temp_array[2] & 0x00FF);
      temp_array[2] = temp;
      temp_array[8] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_pfn_delay_high = temp;
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_pfn_delay_low = temp;
      break;

    case REGISTER_SPECIAL_2_5_SET_AFC_SAMPLE_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      temp <<= 8;
      temp += (temp_array[5] & 0x00FF);
      temp_array[5] = temp;
      temp_array[11] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_high = temp;
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_low = temp;
      break;

    case REGISTER_SPECIAL_2_5_SET_MAGNETRON_CURRENT_SAMPLE_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      temp += (temp_array[5] & 0xFF00);
      temp_array[5] = temp;
      temp_array[11] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 12, &temp_array[0]);
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_high = temp;
      *(unsigned int*)&etm_can_pulse_sync_mirror.psync_afc_delay_low = temp;
      break;
      */
    case REGISTER_SPECIAL_2_5_SET_HV_LAMBDA_VOLTAGE:
      local_hv_lambda_low_en_set_point  = next_message.data_2;     
      local_hv_lambda_high_en_set_point = next_message.data_2;

      eeprom_register = REGISTER_HIGH_ENERGY_SET_POINT + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
 
      eeprom_register = REGISTER_LOW_ENERGY_SET_POINT + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;


    case REGISTER_DEBUG_ENABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      break;

    case REGISTER_DEBUG_DISABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      break;
      
    case REGISTER_DEBUG_TOGGLE_HIGH_SPEED_LOGGING:
      if (_SYNC_CONTROL_HIGH_SPEED_LOGGING) {
	_SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      } else {
	_SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_HV_ENABLE:
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
      } else {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_XRAY_ENABLE:
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY) {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
      } else {
	_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_COOLING_FAULT:
      if (_SYNC_CONTROL_COOLING_FAULT) {
	_SYNC_CONTROL_COOLING_FAULT = 0;
      } else {
	_SYNC_CONTROL_COOLING_FAULT = 1;
      }
      break;

    case REGISTER_DEBUG_TOGGLE_RESET_DEBUG:
      if (_SYNC_CONTROL_CLEAR_DEBUG_DATA) {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
      } else {
	_SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
      }
      break;

    }
  }
}

#define HALF_TMIN   3   // 60nS 

void CalculatePulseSyncParams(unsigned char start, unsigned char stop) {
  // DPARKER BRING THIS BACK
  /*
  unsigned char start_max;
  unsigned char start_med;
  unsigned char start_small;
  unsigned char start_min;


  unsigned char stop_max;
  unsigned char stop_med;
  unsigned char stop_small;
  unsigned char stop_min;

  unsigned int temp;


  if (stop > (start + 2*HALF_TMIN)) { 
    start_max = start;
    stop_max  = stop;
    
    temp      = start;
    temp     += stop;
    temp    >>= 1;    // temp is now equal to the mid point
    
    start_min = temp - HALF_TMIN;
    stop_min  = temp + HALF_TMIN;
    
    temp = (start_min - start_max)/3;  // temp is now the medium offset
    start_med = start_max + temp;
    stop_med  = stop_max - temp;
    
    temp *= 2;  // temp is now the small offset
    start_small = start_max + temp;
    stop_small  = stop_max - temp;
    
    etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_3 = start_max;
    etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_2 = start_med;
    etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_1 = start_small;
    etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_0 = start_min;
    
    etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_3 = start_max;
    etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_2 = start_med;
    etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_1 = start_small;
    etm_can_pulse_sync_mirror.psync_grid_delay_low_intensity_0 = start_min;
    
    etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_3 = stop_max;      
    etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_2 = stop_med;      
    etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_1 = stop_small;      
    etm_can_pulse_sync_mirror.psync_grid_width_high_intensity_0 = stop_min;      
    
    etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_3 = stop_max;      
    etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_2 = stop_med;      
    etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_1 = stop_small;      
    etm_can_pulse_sync_mirror.psync_grid_width_low_intensity_0 = stop_min;      
  }
  
  ETMEEPromWritePage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1 + 0), 12, (unsigned int*)&etm_can_pulse_sync_mirror.psync_grid_delay_high_intensity_2);
  // DPARKER need to update for multiple personalities

  */
}


void LogBoardReadyStatus(void) {


}



// DPARKER need to update all the logging functionality

void LogModuleFault(unsigned int board_address) {
  /* 
     Faults are numbered as = 0x10bf
     b = Board Address as defined in P1395_CAN_CORE.h
     f = Fault bit that is set (0 -> F)
  */
  /*
  ETMCanStatusRegisterFaultBits fault_bits;


  switch (board_address) 
    {
    case ETM_CAN_ADDR_ION_PUMP_BOARD:
      fault_bits = etm_can_ion_pump_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD:
      fault_bits = etm_can_magnetron_current_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_PULSE_SYNC_BOARD:
      fault_bits = etm_can_pulse_sync_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_HV_LAMBDA_BOARD:
      fault_bits = etm_can_hv_lambda_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_AFC_CONTROL_BOARD:
      fault_bits = etm_can_afc_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_COOLING_INTERFACE_BOARD:
      fault_bits = etm_can_afc_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_HEATER_MAGNET_BOARD:
      fault_bits = etm_can_heater_magnet_mirror.status_data.fault_bits;
      break;

    case ETM_CAN_ADDR_GUN_DRIVER_BOARD:
      fault_bits = etm_can_gun_driver_mirror.status_data.fault_bits;
      break;
      
    default:
      break;
    }
  

  //SendToEventLog(0x1100 + (board_address << 4));

  if (fault_bits.fault_0) {
    SendToEventLog(0x1000 + (board_address << 4) + 0);
  }
  if (fault_bits.fault_1) {
    SendToEventLog(0x1000 + (board_address << 4) + 1);
  }
  if (fault_bits.fault_2) {
    SendToEventLog(0x1000 + (board_address << 4) + 2);
  }
  if (fault_bits.fault_3) {
    SendToEventLog(0x1000 + (board_address << 4) + 3);
  }
  if (fault_bits.fault_4) {
    SendToEventLog(0x1000 + (board_address << 4) + 4);
  }
  if (fault_bits.fault_5) {
    SendToEventLog(0x1000 + (board_address << 4) + 5);
  }
  if (fault_bits.fault_6) {
    SendToEventLog(0x1000 + (board_address << 4) + 6);
  }
  if (fault_bits.fault_7) {
    SendToEventLog(0x1000 + (board_address << 4) + 7);
  }
  if (fault_bits.fault_8) {
    SendToEventLog(0x1000 + (board_address << 4) + 8);
  }
  if (fault_bits.fault_9) {
    SendToEventLog(0x1000 + (board_address << 4) + 9);
  }
  if (fault_bits.fault_A) {
    SendToEventLog(0x1000 + (board_address << 4) + 10);
  }
  if (fault_bits.fault_B) {
    SendToEventLog(0x1000 + (board_address << 4) + 11);
  }
  if (fault_bits.fault_C) {
    SendToEventLog(0x1000 + (board_address << 4) + 12);
  }
  if (fault_bits.fault_D) {
    SendToEventLog(0x1000 + (board_address << 4) + 13);
  }
  if (fault_bits.fault_E) {
    SendToEventLog(0x1000 + (board_address << 4) + 14);
  }
  if (fault_bits.fault_F) {
    SendToEventLog(0x1000 + (board_address << 4) + 15);
  }
  */
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  
  Nop();
  Nop();
  __asm__ ("Reset");
}



