#include "A36507.h"
#include "FIRMWARE_VERSION.h"
#include "A36507_CONFIG.h"

unsigned int test_uart_data_recieved;
unsigned int test_ref_det_recieved;
unsigned int test_ref_det_good_message;



void CRCTest(void);

unsigned int LookForDoseMessageFromReferenceDetector(void);

void WriteConfigToMirror(void);
void ReadConfigFromMirror(void);

BUFFERBYTE64 uart1_input_buffer;


unsigned int a_ready;
unsigned int a_sent;
unsigned int b_ready;
unsigned int b_sent;



// ------------------ PROCESSOR CONFIGURATION ------------------------//
_FOSC(ECIO_PLL16 & CSW_FSCM_OFF);                                         // 5Mhz External Osc created 20Mhz FCY
_FWDT(WDT_OFF & WDTPSA_512 & WDTPSB_8);                                    // 8 Second watchdog timer 
_FBORPOR(PWRT_4 & NONE & PBOR_OFF & MCLR_EN);                             // 4ms Power up timer, Low Voltage Reset disabled
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);  // 
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);         //
_FGS(CODE_PROT_OFF);                                                      //
_FICD(PGD);                                                               //


const unsigned int FilamentLookUpTable[64] = {FILAMENT_LOOK_UP_TABLE_VALUES_FOR_MG5193};



// ---------------------- Control Functions ---------------------------- //
void DoStateMachine(void);
/*
  This runs the state machine.
  Typical loop time is a couple hundred uS
  This can be extended significantly by spi/i2c communication if waiting for those functions to complete
*/

unsigned int CheckWarmupFailure(void);
unsigned int CheckWarmupFault(void);
unsigned int CheckConfigurationFault(void);
unsigned int CheckStandbyFault(void);
unsigned int CheckFaultLatching(void);
unsigned int CheckHVOnFault(void);
unsigned int CheckCoolingFault(void);
unsigned int CheckGunHeaterOffFault(void);


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
*/

void CalculatePulseSyncParams(unsigned char start, unsigned char stop);


//void LogModuleFault(unsigned int board_address);
//P1395BoardBits board_not_configured_latch;
//P1395BoardBits board_not_ready_latch;


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
  
  CRCTest();
  Nop();
  Nop();
  Nop();
  Nop();

  global_data_A36507.control_state = STATE_STARTUP;
  
  while (1) {
    DoStateMachine();
  }
}








void DoStateMachine(void) {
  
  switch (global_data_A36507.control_state) {

    
  case STATE_STARTUP:
    InitializeA36507();
    global_data_A36507.gun_heater_holdoff_timer = 0;
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 1;
    global_data_A36507.control_state = STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC;
    SendToEventLog(LOG_ID_ENTERED_STATE_STARTUP);
    if (_STATUS_LAST_RESET_WAS_POWER_CYCLE) {
      _SYNC_CONTROL_CLEAR_DEBUG_DATA = 1;
    }
    break;

  case STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    _STATUS_PERSONALITY_LOADED = 0;
    global_data_A36507.startup_counter = 0;
    while (global_data_A36507.control_state == STATE_WAIT_FOR_PERSONALITY_FROM_PULSE_SYNC) {
      DoA36507();
      FlashLeds();

      if (_PULSE_SYNC_PERSONALITY_READY) {
	personality_select_from_pulse_sync = _PULSE_SYNC_PERSONALITY_VALUE;
	global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
	SendToEventLog(LOG_ID_PERSONALITY_RECEIVED);
      }
      
#ifdef __IGNORE_PULSE_SYNC_MODULE
      personality_select_from_pulse_sync = 0;
      global_data_A36507.control_state = STATE_WAITING_FOR_INITIALIZATION;
#endif

#define PERSONALITY_ERROR_READING  0x000F

      if (personality_select_from_pulse_sync == PERSONALITY_ERROR_READING) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
	SendToEventLog(LOG_ID_PERSONALITY_ERROR);
      }
    }

  case STATE_WAITING_FOR_INITIALIZATION:
    SendToEventLog(LOG_ID_ENTERED_STATE_WAITING_FOR_INITIALIZATION);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    _STATUS_PERSONALITY_LOADED = 1;
    personality_loaded = 1;
    ReadSystemConfigurationFromEEProm(personality_select_from_pulse_sync);
    if (global_data_A36507.eeprom_failure) {
      global_data_A36507.control_state = STATE_FAULT_SYSTEM;
    }
    CalculateHeaterWarmupTimers();     // Calculate all of the warmup counters based on previous warmup completed
    while (global_data_A36507.control_state == STATE_WAITING_FOR_INITIALIZATION) {
      DoA36507();
      FlashLeds();
      if ((!CheckConfigurationFault()) && (global_data_A36507.startup_counter >= 300)) {
      	global_data_A36507.control_state = STATE_WARMUP;
	SendToEventLog(LOG_ID_ALL_MODULES_CONFIGURED);
      }
    }
    break;
    

  case STATE_WARMUP:
    // Note that the warmup timers start counting in "Waiting for Initialization"
    SendToEventLog(LOG_ID_ENTERED_STATE_WARMUP);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_WARMUP) {
      DoA36507();
      if (global_data_A36507.warmup_done) {
	global_data_A36507.control_state = STATE_STANDBY;
	SendToEventLog(LOG_ID_WARMUP_DONE);
      }
      if (CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }
    }
    break;


  case STATE_FAULT_WARMUP:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_WARMUP);
    _SYNC_CONTROL_CLEAR_DEBUG_DATA = 0;
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_WARMUP) {
      DoA36507();
      if (!CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_WARMUP;
      }
      if (CheckWarmupFailure()) {
	global_data_A36507.control_state = STATE_FAULT_SYSTEM;
      }
    }
    break;
    

  case STATE_FAULT_SYSTEM:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_SYSTEM);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (1) {
      DoA36507();
    }
    break;

    
  case STATE_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_STANDBY);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
     while (global_data_A36507.control_state == STATE_STANDBY) {
      DoA36507();
      if (!_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
      }
     }
    break;


  case STATE_FAULT_STANDBY:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_STANDBY);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_FAULT_STANDBY) {
      DoA36507();
      if (!CheckStandbyFault()) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (_FAULT_X_RAY_ON_LOGIC_ERROR) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
      if (CheckWarmupFault()) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }
      if ((global_data_A36507.thyratron_warmup_remaining > 0) ||
	  (global_data_A36507.magnetron_warmup_remaining > 0) ||
	  (global_data_A36507.gun_warmup_remaining > 0)) {
	global_data_A36507.control_state = STATE_FAULT_WARMUP;
      }

     }
    break;


  case STATE_DRIVE_UP:
    SendToEventLog(LOG_ID_ENTERED_STATE_DRIVE_UP);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    while (global_data_A36507.control_state == STATE_DRIVE_UP) {
      DoA36507();
      if (!CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_STANDBY;
      }
      if (CheckStandbyFault()) {
	global_data_A36507.drive_up_fault_counter++;
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
      }
    }
    break;
    

  case STATE_READY:
    SendToEventLog(LOG_ID_ENTERED_STATE_READY);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 1;
    global_data_A36507.drive_up_fault_counter = 0;
    _STATUS_DRIVE_UP_TIMEOUT = 0;
     while (global_data_A36507.control_state == STATE_READY) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF == 0) {
	global_data_A36507.control_state = STATE_XRAY_ON;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_DRIVE_UP;
      }
      if (CheckHVOnFault()) {
	global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
	global_data_A36507.high_voltage_on_fault_counter++;
      }
    }
    break;


  case STATE_XRAY_ON:
    SendToEventLog(LOG_ID_ENTERED_STATE_XRAY_ON);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 0;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.high_voltage_on_fault_counter = 0;
    while (global_data_A36507.control_state == STATE_XRAY_ON) {
      DoA36507();
      if (_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	global_data_A36507.control_state = STATE_READY;
      }
      if (CheckHVOnFault()) {
	//global_data_A36507.control_state = STATE_FAULT_LATCH_DECISION;
	global_data_A36507.control_state = STATE_FAULT_HOLD;  // Why would you ever need to make this decision if X-Rays were on
      }
    }
    break;


  case STATE_FAULT_LATCH_DECISION:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_LATCH_DECISION);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
    global_data_A36507.reset_hold_timer = 0;
    while (global_data_A36507.control_state == STATE_FAULT_LATCH_DECISION) {
      DoA36507();
      if (global_data_A36507.reset_hold_timer > MINIMUM_FAULT_HOLD_TIME) { 
	// Need to wait in this state for X_RAY_ON status to propigate from the pulse sync board
	global_data_A36507.control_state = STATE_FAULT_RESET_HOLD;
      }
      if (CheckFaultLatching()) {
	global_data_A36507.control_state = STATE_FAULT_HOLD;
      }
    }
    break;


  case STATE_FAULT_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_HOLD);
    _SYNC_CONTROL_RESET_ENABLE = 0;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
      while (global_data_A36507.control_state == STATE_FAULT_HOLD) {
      DoA36507();
      if (global_data_A36507.reset_requested) {
	global_data_A36507.control_state = STATE_FAULT_RESET_HOLD;
      }
     }
    break;
    

  case STATE_FAULT_RESET_HOLD:
    SendToEventLog(LOG_ID_ENTERED_STATE_FAULT_RESET_HOLD);
    _SYNC_CONTROL_RESET_ENABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_HV = 1;
    _SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY = 1;
    _SYNC_CONTROL_SYSTEM_HV_DISABLE = 1;
    _SYNC_CONTROL_PULSE_SYNC_FAULT_LED = 1;
    _SYNC_CONTROL_PULSE_SYNC_WARMUP_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_STANDBY_LED = 0;
    _SYNC_CONTROL_PULSE_SYNC_READY_LED = 0;
    global_data_A36507.reset_requested = 0;
    global_data_A36507.reset_hold_timer = 0;
    while (global_data_A36507.control_state == STATE_FAULT_RESET_HOLD) {
      DoA36507();
      if (global_data_A36507.reset_hold_timer > FAULT_RESET_HOLD_TIME) { 
	global_data_A36507.control_state = STATE_FAULT_STANDBY;
      }
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

unsigned int CheckWarmupFailure(void) {
  // DPARKER need to add this
  // Look for failure of magnetron or gun heater
  return 0;
}

unsigned int CheckWarmupFault(void) {
  // What can go wrong in warmup???

  // The gun driver can report that the heater is not on
  // The heater/magnet board can report that the heater is not on
  // We can loose communication with Gun Driver, Heater Magnet, or Ion Pump Board
  // The Gun Driver, Heater Magnet reports that it is not configured
  // The Ion Pump can detect too much current and shut down the gun heater  (We don't need to check this, because the gun heater will turn off)
  
  // There is nothing the user can do about any of these.  The can just get a status that something is not right
  // If the fault is unrecoverable(Gun Heater or Magnetron Heater) they can power cycle the system


#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (!_HEATER_MAGNET_HEATER_OK) {
    return 1;
  }
  if (!board_com_ok.heater_magnet_board) {
    return 1;
  }
  if (_HEATER_MAGNET_NOT_CONFIGURED) {
    return 1;
  }
#endif


#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (!board_com_ok.gun_driver_board) {
    return 1;
  }
  if (_GUN_DRIVER_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_ION_PUMP_MODULE
  if (!board_com_ok.ion_pump_board) {
    return 1;
  }
#endif

  return 0;
}


unsigned int CheckConfigurationFault(void) {
#ifndef __IGNORE_HV_LAMBDA_MODULE
  if (!board_com_ok.hv_lambda_board || _HV_LAMBDA_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  if (!board_com_ok.ion_pump_board || _ION_PUMP_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_AFC_MODULE
  if (!board_com_ok.afc_board || _AFC_NOT_CONFIGURED) {
    return 1;
  }
#endif  

#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (!board_com_ok.cooling_interface_board || _COOLING_NOT_CONFIGURED) {
    return 1;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (!board_com_ok.heater_magnet_board || _HEATER_MAGNET_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (!board_com_ok.gun_driver_board || _GUN_DRIVER_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if (!board_com_ok.magnetron_current_board || _PULSE_MON_NOT_CONFIGURED) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if (!board_com_ok.pulse_sync_board || _PULSE_SYNC_NOT_CONFIGURED) {
    return 1;
  }
#endif
  
  return 0;
}


// DPARKER
unsigned int CheckFaultLatching(void) {
  if (_FAULT_REPEATED_DRIVE_UP_FAULT || _FAULT_REPEATED_HV_ON_FAULT || _FAULT_X_RAY_ON_LOGIC_ERROR) {
    return 1;
  }
  if (_PULSE_MON_FALSE_TRIGGER) {
    // If a false trigger is detected we must hold the fault
    return 1;
  }
  if (_PULSE_SYNC_FAULT_X_RAY_MISMATCH) {
    return 1;
  }
  if (!_PULSE_SYNC_CUSTOMER_XRAY_OFF) {
    // The fault happened when X-Rays were on, need to latch the fault
    return 1;
  }


  return 0;
}


unsigned int CheckStandbyFault(void) {
  unsigned int faults = 0;

  /*
    Loose Connection with any board - This is covered by CheckAllModulesConfigured
    Any Board Reports not configured - This is covered by CheckAllModulesConfigured
    Any Board Reports a fault
  */

  if (_FAULT_REGISTER) {
    return 1;
  }
  
  if (CheckConfigurationFault()) {
    return 1;
  }
  
#ifndef __IGNORE_HV_LAMBDA_MODULE
  faults |= _HV_LAMBDA_FAULT_REGISTER; 
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  faults |= _ION_PUMP_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_AFC_MODULE
  faults |= _AFC_FAULT_REGISTER;
#endif  
  
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  faults |= _COOLING_FAULT_REGISTER;
#endif  
  
#ifndef __IGNORE_HEATER_MAGNET_MODULE
  faults |= _HEATER_MAGNET_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  faults |= _GUN_DRIVER_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_PULSE_CURRENT_MODULE
  faults |= _PULSE_MON_FAULT_REGISTER;
#endif
  
#ifndef __IGNORE_PULSE_SYNC_MODULE
  faults |= _PULSE_SYNC_FAULT_REGISTER;
#endif
  
  if (faults) {
    return 1;
  }
  
  if (global_data_A36507.drive_up_timer > DRIVE_UP_TIMEOUT) {
    _STATUS_DRIVE_UP_TIMEOUT = 1;
    SendToEventLog(LOG_ID_DRIVE_UP_TIMEOUT);
    return 1;
  }

  return 0;
}


unsigned int CheckCoolingFault(void) {
  // Check to see if cooling is present
#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (!board_com_ok.cooling_interface_board) {
    return 1;
  }
  if (!_COOLING_FLOW_OK) {
    return 1;
  }
#endif
  return 0;
}


unsigned int CheckGunHeaterOffFault(void) {
  // Check to see if there is an active over current condition in the ion pump

  //#ifndef __IGNORE_ION_PUMP_MODULE
  if (_ION_PUMP_OVER_CURRENT_ACTIVE) {
    global_data_A36507.gun_heater_holdoff_timer = 0;
    return 1;
  }
  if (!board_com_ok.ion_pump_board) {
    return 1;
  }
  if (_ION_PUMP_NOT_CONFIGURED) {
    return 1;
  }
  if (global_data_A36507.gun_heater_holdoff_timer < GUN_HEATER_HOLDOFF_AT_STARTUP) {
    return 1;
  }
  if (global_data_A36507.thyratron_warmup_remaining > global_data_A36507.gun_warmup_remaining) {
    if (global_data_A36507.gun_heater_holdoff_timer < (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
      return 1;
    }
  }

  return 0;
}


unsigned int CheckHVOnFault(void) {
  /*
    CheckStandbyFault();
    Any Board Reports Not Ready
  */

  if (_FAULT_REGISTER) {
    return 1;
  }
  
  if (CheckStandbyFault()) {
    return 1;
  }
  
#ifndef __IGNORE_HV_LAMBDA_MODULE
  if (_HV_LAMBDA_NOT_READY) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_ION_PUMP_MODULE  
  if (_ION_PUMP_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_AFC_MODULE
  if (_AFC_NOT_READY) {
    return 1;
  }
#endif  

#ifndef __IGNORE_COOLING_INTERFACE_MODULE
  if (_COOLING_NOT_READY) {
    return 1;
  }
#endif  

#ifndef __IGNORE_HEATER_MAGNET_MODULE
  if (_HEATER_MAGNET_NOT_READY) {
    return 1;
  }
#endif
  
#ifndef __IGNORE_GUN_DRIVER_MODULE
  if (_GUN_DRIVER_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_CURRENT_MODULE
  if (_PULSE_MON_NOT_READY) {
    return 1;
  }
#endif

#ifndef __IGNORE_PULSE_SYNC_MODULE
  if (_PULSE_SYNC_NOT_READY) {
    return 1;
  }
#endif

  return 0;
}


/*
#define DEBUG_ETMMODBUS

#ifdef DEBUG_ETMMODBUS
  //static MODBUS_RESP_SMALL etmmodbus_test[4];
  //static unsigned etmmodbus_index = 0;
  //static unsigned etmmodbus_relay_set_index = 0;
  //static unsigned etmmodbus_relay_toggle = 0;
#endif 
  //_SYNC_CONTROL_WORD                 = 0xF0F0;
  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = global_data_A36507.control_state;
  etm_can_master_sync_message.sync_2 = 0x0123;
  etm_can_master_sync_message.sync_3 = 0x4567;

  ETMCanMasterDoCan();
  TCPmodbus_task();
  ExecuteEthernetCommand(personality_select_from_pulse_sync);

#ifndef __IGNORE_TCU
  ETMmodbus_task();
#endif


#ifndef __IGNORE_TCU
    if (ETMmodbus_timer_10ms < 60000) ETMmodbus_timer_10ms++;
    
#ifdef DEBUG_ETMMODBUS
    if (can_master_millisecond_counter == 500) {
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
    


*/





void UpdateDebugData(void) {
  debug_data_ecb.debug_reg[0]  = global_data_A36507.high_voltage_on_fault_counter; 
  debug_data_ecb.debug_reg[1]  = global_data_A36507.drive_up_fault_counter;
  if (_STATUS_LAST_RESET_WAS_POWER_CYCLE) {
    debug_data_ecb.debug_reg[2]  = 1;
  } else {
    debug_data_ecb.debug_reg[2]  = 0;
  }
  debug_data_ecb.debug_reg[3]  = test_ref_det_recieved; 
  
  debug_data_ecb.debug_reg[4]  = test_ref_det_good_message; 
  debug_data_ecb.debug_reg[5]  = global_data_A36507.most_recent_ref_detector_reading; 
  debug_data_ecb.debug_reg[6]  = test_uart_data_recieved; 
  debug_data_ecb.debug_reg[7]  = 7; 

  debug_data_ecb.debug_reg[8]  = 8; 
  debug_data_ecb.debug_reg[9]  = 9; 
  debug_data_ecb.debug_reg[10] = 10; 
  debug_data_ecb.debug_reg[11] = 11; 

  debug_data_ecb.debug_reg[12] = 12; 
  debug_data_ecb.debug_reg[13] = 0;
  debug_data_ecb.debug_reg[14] = 0;
  debug_data_ecb.debug_reg[15] = 0;
}


void DoA36507(void) {
  unsigned int fast_log_index;


  etm_can_master_sync_message.sync_1_ecb_state_for_fault_logic = global_data_A36507.control_state;
  etm_can_master_sync_message.sync_2 = 0x0123;
  etm_can_master_sync_message.sync_3 = 0x4567;

  ETMCanMasterDoCan();
  //TCPmodbus_task();
  ETMLinacModbusUpdate();
  ExecuteEthernetCommand(personality_select_from_pulse_sync);
  if (LookForDoseMessageFromReferenceDetector()) {
    // the dose data was updated with a new value from the reference detector
    fast_log_index = etm_can_master_next_pulse_count - 1;
    if (fast_log_index & 0x0010) {
      high_speed_data_buffer_a[(fast_log_index & 0x000F)].ionpump_readback_high_energy_target_current_reading = global_data_A36507.most_recent_ref_detector_reading;
    } else {
      high_speed_data_buffer_b[(fast_log_index & 0x000F)].ionpump_readback_high_energy_target_current_reading = global_data_A36507.most_recent_ref_detector_reading;
    }
  }


  if (global_data_A36507.eeprom_failure) {
    _FAULT_EEPROM_FAILURE = 1;
  }


  // Figure out if the customer has enabled XRAYs before they should have
  // If so set a fault that can only be cleared with a reset command
  if (!_PULSE_SYNC_CUSTOMER_XRAY_OFF) { 
    if ((global_data_A36507.control_state == STATE_WARMUP) ||
	(global_data_A36507.control_state == STATE_FAULT_WARMUP) ||
	(global_data_A36507.control_state == STATE_FAULT_STANDBY)) { 
      // Customer Enabled XRAYs when not ready
      _FAULT_X_RAY_ON_LOGIC_ERROR = 1;
    }
    if ((global_data_A36507.control_state == STATE_STANDBY) || 
	(global_data_A36507.control_state == STATE_FAULT_HOLD)) {
      
      if (_PULSE_SYNC_CUSTOMER_HV_OFF) {
	// Customer Enabled XRAYS, but not High Voltage durring one of the standby states
	_FAULT_X_RAY_ON_LOGIC_ERROR = 1;
      }
    }
  }  
    
  if (global_data_A36507.drive_up_fault_counter > MAX_DRIVE_UP_FAULTS) {
    _FAULT_REPEATED_DRIVE_UP_FAULT = 1;
  }
  
  if (global_data_A36507.high_voltage_on_fault_counter > MAX_HV_ON_FAULTS) {
    _FAULT_REPEATED_HV_ON_FAULT = 1;
  }
  
  // Update the cooling fault sync bit
  if (CheckCoolingFault()) {
    _SYNC_CONTROL_COOLING_FAULT = 1;
  } else {
    _SYNC_CONTROL_COOLING_FAULT = 0;
  }

  // Update the Gun Driver Heater Enable sync bit
  // DPARKER need to update the libraries to use the Gun Heater Disable Bit
  if (CheckGunHeaterOffFault()) {
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 1;
  } else {
    _SYNC_CONTROL_GUN_DRIVER_DISABLE_HTR = 0;
  }

  // Update the SYNC message with the control state
  // DPARKER as far as I know, this is never used - consider removing
  ETMCanMasterSetSyncState(global_data_A36507.control_state);


  // Load log_data Memory for types that can not be mapped directly into memory
  local_data_ecb.log_data[0] = global_data_A36507.control_state;
  local_data_ecb.log_data[3] = ETMCanMasterGetPulsePRF();
  local_data_ecb.log_data[4] = global_data_A36507.thyratron_warmup_remaining;
  local_data_ecb.log_data[5] = global_data_A36507.magnetron_warmup_remaining;
  local_data_ecb.log_data[6] = global_data_A36507.gun_warmup_remaining;
  local_data_ecb.log_data[7] = _SYNC_CONTROL_WORD;
  local_data_ecb.log_data[16] = *(unsigned int*)&board_com_ok;
  local_data_ecb.log_data[17] = global_data_A36507.most_recent_ref_detector_reading;
  local_data_ecb.log_data[19] = global_data_A36507.system_serial_number;
  mirror_cooling.local_data[0] = MAX_SF6_REFILL_PULSES_IN_BOTTLE;

  UpdateDebugData();  // Load the customized debugging data into the debugging registers


  // 10ms Timer has expired -- run periodic checks and updates
  if (_T2IF) {
    _T2IF = 0;
    if (global_data_A36507.control_state == STATE_DRIVE_UP) {
      global_data_A36507.drive_up_timer++;
    } else {
      global_data_A36507.drive_up_timer = 0;
    }
    global_data_A36507.startup_counter++;
    global_data_A36507.reset_hold_timer++;
    
    // Update the heater current based on Output Power
    UpdateHeaterScale();

    if (global_data_A36507.gun_heater_holdoff_timer <= (GUN_HEATER_HOLDOFF_AT_STARTUP + GUN_HEATER_ADDITONAL_HOLDOFF_COLD)) {
      global_data_A36507.gun_heater_holdoff_timer++;
    }



    /*
      The following tasks require use of the i2c bus which can hold the processor for a lot of time
      Need to schedule them at different point of a 1 second period
    */
    can_master_millisecond_counter += 10;

    // Run at 1 second interval
    if (can_master_millisecond_counter >= 1000) {
      can_master_millisecond_counter = 0;
    }

    // Run once a second at 0 milliseconds
    if (can_master_millisecond_counter == 0) {
      // Read Date/Time from RTC and update the warmup up counters

      /*
	DPARKER - need to update the way that the timers work
	What we should do . . .
	At time Zero, read the current time and update the warmup counters just like we do now
	At Time 250, write the current time (2 words) and the remaining time for each timer (3 words total)

	When you startup and calculate the warmup time, you can treat each independently . . .
	warmup time = old warmup time remaining + 2*(current time - stored time) - Capping the value at max warmup time of course
      */

      ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
      mem_time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

      // Update the warmup counters
      if (!_PULSE_SYNC_PFN_FAN_FAULT) {
	if (global_data_A36507.thyratron_warmup_remaining > 0) {
	  global_data_A36507.thyratron_warmup_remaining--;
	}
      } else {
	global_data_A36507.thyratron_warmup_remaining += 2;
      }
      if (global_data_A36507.thyratron_warmup_remaining >= THYRATRON_WARM_UP_TIME) {
	global_data_A36507.thyratron_warmup_remaining = THYRATRON_WARM_UP_TIME;
      }
      
      
      if ((board_com_ok.heater_magnet_board) && (_HEATER_MAGNET_HEATER_OK)) {
	// The Magnetron heater is on
	if (global_data_A36507.magnetron_warmup_remaining > 0) {
	  global_data_A36507.magnetron_warmup_remaining--;
	}
      } else {
	global_data_A36507.magnetron_warmup_remaining += 2;
      }
      if (global_data_A36507.magnetron_warmup_remaining >= MAGNETRON_HEATER_WARM_UP_TIME) {
	global_data_A36507.magnetron_warmup_remaining = MAGNETRON_HEATER_WARM_UP_TIME;
      }

      
      if (board_com_ok.gun_driver_board && _GUN_DRIVER_HEATER_RAMP_COMPLETE) {
	// The gun heater is on
	if (global_data_A36507.gun_warmup_remaining > 0) {
	  global_data_A36507.gun_warmup_remaining--;
	}
      } else {
	global_data_A36507.gun_warmup_remaining += 2;
      }
      if (global_data_A36507.gun_warmup_remaining >= GUN_DRIVER_HEATER_WARM_UP_TIME) {
	global_data_A36507.gun_warmup_remaining = GUN_DRIVER_HEATER_WARM_UP_TIME;
      }
      
      
#ifdef __IGNORE_HEATER_MAGNET_MODULE
      global_data_A36507.magnetron_warmup_remaining = 0;
#endif
      
#ifdef __IGNORE_GUN_DRIVER_MODULE
      global_data_A36507.gun_warmup_remaining = 0;
#endif
      
      if ((global_data_A36507.thyratron_warmup_remaining) || (global_data_A36507.magnetron_warmup_remaining) || (global_data_A36507.gun_warmup_remaining)) {
	global_data_A36507.warmup_done = 0;
      } else {
	global_data_A36507.warmup_done = 1;
      }
    } // End of tasks that happen when millisecond = 0
    

    // Run once a second at 250 milliseconds
    if (can_master_millisecond_counter == 250) {
      // Write Warmup Done Timers to EEPROM
      global_data_A36507.last_recorded_warmup_seconds = mem_time_seconds_now;
      ETMEEPromWritePage(EEPROM_PAGE_HEATER_TIMERS, 5, (unsigned int*)&global_data_A36507.last_recorded_warmup_seconds);
    } // End of tasks that happen when millisecond = 250
    

    // Run once a second at 500 milliseconds
    if (can_master_millisecond_counter == 500) {
      // Write Seconds on Counters to EEPROM
      system_powered_seconds++;
      
      if (global_data_A36507.control_state == STATE_READY) {
	system_hv_on_seconds++;
      }
      
      if (global_data_A36507.control_state == STATE_XRAY_ON) {
	system_hv_on_seconds++;
	system_xray_on_seconds++;
      }
      ETMEEPromWritePage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&system_powered_seconds);
    } // End of tasks that happen when millisecond = 500
  
  } // End of 10ms Tasks
}


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
  
  average_output_power_watts = temp32;
  temp16 = average_output_power_watts;

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
  *(unsigned int*)&board_com_ok = 0x0000;
    
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

  // manually clock out I2C CLK to clear any connected processors that may have been stuck on a reset  
  _TRISG2 = 0; // g2 is output
  for (loop_counter = 0; loop_counter <= 100; loop_counter++) {
    _LATG2 = 0;
    __delay32(25);
    _LATG2 = 1;
    __delay32(25);
  }
  ETMEEPromUseExternal();
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, ETM_I2C_400K_BAUD, EEPROM_I2C_ADDRESS_0, 1);
  ConfigureDS3231(&U6_DS3231, I2C_PORT, RTC_DEFAULT_CONFIG, FCY_CLK, ETM_I2C_400K_BAUD);
  
  // Read the current time
  ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  mem_time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);
  
  // Initialize the Can module
  ETMCanMasterInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_ETHERNET_BOARD, _PIN_RG13, 4);
  ETMCanMasterLoadConfiguration(36507, 251, ETMEEPromReadWord(0x0181), FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV, ETMEEPromReadWord(0x0180));
  global_data_A36507.system_serial_number = ETMEEPromReadWord(EEPROM_REGISTER_TOP_LEVEL_SERIAL_NUMBER);



  // Initialize TCPmodbus Module
#if 0
  ip_config.remote_ip_addr   = ETMEEPromReadWord(EEPROM_REGISTER_REMOTE_IP_ADDRESS);
  ip_config.remote_ip_addr <<= 16;
  ip_config.remote_ip_addr  += ETMEEPromReadWord(EEPROM_REGISTER_REMOTE_IP_ADDRESS + 1);
  ip_config.ip_addr          = ETMEEPromReadWord(EEPROM_REGISTER_IP_ADDRESS);
  ip_config.ip_addr        <<= 16;
  ip_config.ip_addr         += ETMEEPromReadWord(EEPROM_REGISTER_IP_ADDRESS + 1);

  Nop();
  Nop();

  //ip_config.remote_ip_addr = 0x0F46A8C0;  // 192.168.70.15
  //ip_config.ip_addr        = 0x6346A8C0;  // 192.168.70.99


  if ((ip_config.remote_ip_addr == 0xFFFFFFFF) || (ip_config.remote_ip_addr == 0x00000000)) {
    ip_config.remote_ip_addr = DEFAULT_REMOTE_IP_ADDRESS;
  }
  if ((ip_config.ip_addr == 0xFFFFFFFF) || (ip_config.ip_addr == 0x00000000)) {
    ip_config.ip_addr = DEFAULT_IP_ADDRESS;
  }

  TCPmodbus_init(&ip_config);
#endif
  
  ETMLinacModbusInitialize();


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
  ETMEEPromReadPage(EEPROM_PAGE_ON_TIME, 6, (unsigned int*)&system_powered_seconds);

#define UART1_BAUDRATE             112000        // 113K Baud Rate
#define A36507_U1MODE_VALUE        (UART_DIS & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT)
#define A36507_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define A36507_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)


  U1MODE = A36507_U1MODE_VALUE;
  U1BRG = A36507_U1BRG_VALUE;
  U1STA = A36507_U1STA_VALUE;
  
  BufferByte64Initialize(&uart1_input_buffer);

  _U1RXIF = 0;
  _U1RXIE = 1;
  _U1RXIP = 6;
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on

}
 
 
void CalculateHeaterWarmupTimers(void) {
  unsigned long difference;
  // Read the warmup timers stored in EEPROM
  ETMEEPromReadPage(EEPROM_PAGE_HEATER_TIMERS, 5, (unsigned int*)&global_data_A36507.last_recorded_warmup_seconds);
  ReadDateAndTime(&U6_DS3231, &global_data_A36507.time_now);
  mem_time_seconds_now = RTCDateToSeconds(&global_data_A36507.time_now);

  // Calculate new warm up time remaining
  difference = mem_time_seconds_now - global_data_A36507.last_recorded_warmup_seconds;
  if (difference >= 0x0E00) {
    difference = 0x0E00;
  }
  difference *= 2;

  global_data_A36507.thyratron_warmup_remaining += difference;
  global_data_A36507.magnetron_warmup_remaining += difference;
  global_data_A36507.gun_warmup_remaining += difference;

}


void ReadSystemConfigurationFromEEProm(unsigned int personality) {
  if (personality >= 3) {
    personality = 0;
  }
  // Personality is a register offset
  
  // Check the status of the EEPROM
  if (ETMEEPromReadWord(EEPROM_REGISTER_EEPROM_OK_CHECK) != 0xACAC) {
    // DPARKER consider doing more error checking here
    LoadDefaultSystemCalibrationToEEProm();
    __delay32(1000000);
  }
  
  global_data_A36507.eeprom_failure = 0;
  if (ETMEEPromReadWord(EEPROM_REGISTER_EEPROM_OK_CHECK) != 0xACAC) {
    // There is a EEPROM failure
    global_data_A36507.eeprom_failure = 1;
  }

  // Load data for HV Lambda
  local_hv_lambda_low_en_set_point    = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_LOW_ENERGY_SET_POINT + (2*personality)));
  local_hv_lambda_high_en_set_point   = ETMEEPromReadWord((EEPROM_REGISTER_LAMBDA_HIGH_ENERGY_SET_POINT + (2*personality)));

  // Load data for AFC
  local_afc_home_position                    = ETMEEPromReadWord((EEPROM_REGISTER_AFC_HOME_POSITION + personality));
  local_afc_aft_control_voltage_high_energy  = ETMEEPromReadWord(EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_HIGH_ENERGY);
  local_afc_aft_control_voltage_low_energy   = ETMEEPromReadWord(EEPROM_REGISTER_AFC_AFT_CONTROL_VOLTAGE_LOW_ENERGY);
  // etm_can_afc_mirror.afc_offset = ETMEEPromReadWord(EEPROM_REGISTER_AFC_OFFSET);

  // Load Data for Heater/Magnet Supply
  local_heater_current_full_set_point        = ETMEEPromReadWord(EEPROM_REGISTER_HTR_MAG_HEATER_CURRENT);
  local_magnet_current_set_point_high_energy = ETMEEPromReadWord((EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_HIGH_ENERGY + personality));
  local_magnet_current_set_point_low_energy  = ETMEEPromReadWord((EEPROM_REGISTER_HTR_MAG_MAGNET_CURRENT_LOW_ENERGY + personality));

  // Load data for Gun Driver
  local_gun_drv_heater_v_set_point    = ETMEEPromReadWord(EEPROM_REGISTER_GUN_DRV_HTR_VOLTAGE);
  local_gun_drv_high_en_pulse_top_v   = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_HIGH_PULSE_TOP + (3*personality)));
  local_gun_drv_low_en_pulse_top_v    = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_LOW_PULSE_TOP + (3*personality)));
  local_gun_drv_cathode_set_point     = ETMEEPromReadWord((EEPROM_REGISTER_GUN_DRV_CATHODE + (3*personality)));

  // Load data for Pulse Sync
  ETMEEPromReadPage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1 + personality), 16, (unsigned int*)&mirror_pulse_sync.local_data[0]);
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
  system_powered_seconds = 0;
  system_hv_on_seconds = 0;
  system_xray_on_seconds = 0;
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

#define REGISTER_HEATER_CURRENT_AT_STANDBY 0x0000
#define REGISTER_ELECTROMAGNET_CURRENT_HIGH_ENERGY 0x0001
#define REGISTER_ELECTROMAGNET_CURRENT_LOW_ENERGY 0x000C
#define REGISTER_HOME_POSITION 0x0005
#define REGISTER_AFC_OFFSET 0x0009
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE_HIGH_ENERGY 0x000A
#define REGISTER_AFC_AFT_CONTROL_VOLTAGE_LOW_ENERGY 0x000B

#define REGISTER_HIGH_ENERGY_SET_POINT 0x0010
#define REGISTER_LOW_ENERGY_SET_POINT 0x0011
#define REGISTER_REMOTE_IP_ADDRESS 0x0018
#define REGISTER_IP_ADDRESS 0x001A
#define REGISTER_ECB_SYSTEM_SERIAL_NUMBER 0x001F

#define REGISTER_GUN_DRIVER_HEATER_VOLTAGE 0x0020
#define REGISTER_GUN_DRIVER_HIGH_ENERGY_PULSE_TOP_VOLTAGE 0x0021
#define REGISTER_GUN_DRIVER_LOW_ENERGY_PULSE_TOP_VOLTAGE 0x0022
#define REGISTER_GUN_DRIVER_CATHODE_VOLTAGE 0x0023

#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_A_B 0x0030
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_HIGH_ENERGY_C_D 0x0031
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_HIGH_ENERGY 0x0032
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_A_B 0x0034
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_HIGH_ENERGY_C_D 0x0035
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_HIGH_ENERGY 0x0036
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_A_B 0x0038
#define REGISTER_PULSE_SYNC_GRID_PULSE_DELAY_LOW_ENERGY_C_D 0x0039
#define REGISTER_PULSE_SYNC_RF_TRIGGER_AND_THYRATRON_PULSE_DELAY_LOW_ENERGY 0x003A
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_A_B 0x003C
#define REGISTER_PULSE_SYNC_GRID_PULSE_WIDTH_LOW_ENERGY_C_D 0x003D
#define REGISTER_PULSE_SYNC_AFC_AND_SPARE_PULSE_DELAY_LOW_ENERGY 0x003E

#define REGISTER_CMD_AFC_SELECT_AFC_MODE 0x5202
#define REGISTER_CMD_AFC_SELECT_MANUAL_MODE 0x5203
#define REGISTER_CMD_AFC_MANUAL_TARGET_POSITION 0x5204

#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_25_MORE_SF6_PULSES 0x6200
#define REGISTER_CMD_COOLANT_INTERFACE_ALLOW_SF6_PULSES_WHEN_PRESSURE_BELOW_LIMIT 0x6201
#define REGISTER_CMD_COOLANT_INTERFACE_SET_SF6_PULSES_IN_BOTTLE 0x6202

#define REGISTER_CMD_ECB_RESET_FAULTS 0xE200

#define REGISTER_SYSTEM_SET_TIME 0xE300
#define REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING 0xE301
#define REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING 0xE302
#define REGISTER_SYSTEM_ECB_LOAD_FACTORY_SETTINGS_FROM_EEPROM_MIRROR_AND_REBOOT 0xE303

#define REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT 0xE400
#define REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON 0xE401
#define REGISTER_ETM_ECB_SEND_SLAVE_RELOAD_EEPROM_WITH_DEFAULTS 0xE402
#define REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT 0xE403
#define REGISTER_ETM_ECB_SAVE_FACTORY_SETTINGS_TO_EEPROM_MIRROR 0xE404

#define REGISTER_DEBUG_TOGGLE_RESET_DEBUG 0xE500
#define REGISTER_DEBUG_GUN_DRIVER_RESET_FPGA 0xE501
#define REGISTER_DEBUG_RESET_MCU 0xE502
#define REGISTER_DEBUG_TEST_PULSE_FAULT 0xE503


void ExecuteEthernetCommand(unsigned int personality) {
  ETMEthernetMessageFromGUI next_message;
  unsigned int eeprom_register;
  //unsigned int temp;
  //unsigned int temp_array[16];


  unsigned long temp_long;
  RTC_TIME set_time;

  if (personality >= 3) {
    personality = 0;
  }
  // Personality is now a register offset

  next_message = GetNextMessage();
  if (next_message.index == 0xFFFF) {
    // there was no message
    return;
  }
  
  if ((next_message.index & 0x0F00) == 0x0100) {
    // this is a calibration set message, route to appropriate board
    if ((next_message.index & 0xF000) == 0xE000) {
      // It is a message for the ECB
      eeprom_register = next_message.index & 0x0FFF;
      ETMEEPromWriteWord(eeprom_register, next_message.data_0);
      ETMEEPromWriteWord(eeprom_register + 1, next_message.data_1);
    } else {
      // It is a message for a slave
      SendCalibrationSetPointToSlave(next_message.index, next_message.data_1, next_message.data_0);
    }
  } else if ((next_message.index & 0x0F00) == 0x0900) {
    // this is a calibration requestion message, route to appropriate board
    // When the response is received, the data will be transfered to the GUI
    if ((next_message.index & 0xF000) == 0xE000) {
      // It is a message for the ECB
      eeprom_register = next_message.index & 0x0FFF;
      eeprom_register -= 0x0800;
      SendCalibrationDataToGUI(next_message.index - 0x0800, ETMEEPromReadWord(eeprom_register + 1), ETMEEPromReadWord(eeprom_register));
    } else {
      ReadCalibrationSetPointFromSlave(next_message.index);
    }
  } else {
    // This message needs to be processsed by the ethernet control board
    switch (next_message.index) {
    case REGISTER_HEATER_CURRENT_AT_STANDBY:
      local_heater_current_full_set_point = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_HIGH_ENERGY:
      local_magnet_current_set_point_high_energy = next_message.data_2;
      eeprom_register = next_message.index + personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_ELECTROMAGNET_CURRENT_LOW_ENERGY:
      local_magnet_current_set_point_low_energy = next_message.data_2;
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
      
    case REGISTER_AFC_AFT_CONTROL_VOLTAGE_HIGH_ENERGY:
      local_afc_aft_control_voltage_high_energy = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_AFC_AFT_CONTROL_VOLTAGE_LOW_ENERGY:
      local_afc_aft_control_voltage_low_energy = next_message.data_2;
      eeprom_register = next_message.index;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_HIGH_ENERGY_SET_POINT:
      local_hv_lambda_high_en_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;

    case REGISTER_LOW_ENERGY_SET_POINT:
      local_hv_lambda_low_en_set_point = next_message.data_2;
      eeprom_register = next_message.index + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
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
      break;

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

    case REGISTER_ECB_SYSTEM_SERIAL_NUMBER:
      ETMEEPromWriteWord(next_message.index, next_message.data_2);
      global_data_A36507.system_serial_number = ETMEEPromReadWord(next_message.index);
      //global_data_A36507.system_serial_number = next_message.data_2;
      break;

    case REGISTER_REMOTE_IP_ADDRESS:
      ETMEEPromWriteWord(next_message.index, next_message.data_2);
      ETMEEPromWriteWord(next_message.index + 1, next_message.data_1);
      break;
      
    case REGISTER_IP_ADDRESS:
      ETMEEPromWriteWord(next_message.index, next_message.data_2);
      ETMEEPromWriteWord(next_message.index + 1, next_message.data_1);
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


    case REGISTER_DEBUG_GUN_DRIVER_RESET_FPGA:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_GUN_DRIVER_BOARD << 2)),
			  0x8202,
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
			  MAX_SF6_REFILL_PULSES_IN_BOTTLE);


    case REGISTER_ETM_ECB_RESET_ARC_AND_PULSE_COUNT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD << 2)),
			  0x2200, // DPARKER ADD THIS TO CAN CORE WITH APPROPRIATE NAME
			  0,
			  0,
			  0);
      break;

    case REGISTER_ETM_ECB_RESET_SECONDS_POWERED_HV_ON_XRAY_ON:
      ZeroSystemPoweredTime();      
      break;

    case REGISTER_ETM_ECB_LOAD_DEFAULT_SYSTEM_SETTINGS_AND_REBOOT:
      LoadDefaultSystemCalibrationToEEProm();
      __delay32(1000000);
      __asm__ ("Reset");
      break;

    case REGISTER_ETM_ECB_SAVE_FACTORY_SETTINGS_TO_EEPROM_MIRROR:
      WriteConfigToMirror();
      break;

    case REGISTER_SYSTEM_ECB_LOAD_FACTORY_SETTINGS_FROM_EEPROM_MIRROR_AND_REBOOT:
      ReadConfigFromMirror();
      __delay32(1000000);
      __asm__ ("Reset");
      break;

    case REGISTER_ETM_ECB_SEND_SLAVE_RELOAD_EEPROM_WITH_DEFAULTS:
      if ((global_data_A36507.control_state < STATE_DRIVE_UP) || (global_data_A36507.control_state > STATE_XRAY_ON)) {
	SendSlaveLoadDefaultEEpromData(next_message.data_2);
      }
      break;

    case REGISTER_CMD_ECB_RESET_FAULTS:
      global_data_A36507.reset_requested = 1;
      _FAULT_REGISTER = 0;
      global_data_A36507.drive_up_fault_counter = 0;
      global_data_A36507.high_voltage_on_fault_counter = 0;
      break;

    case REGISTER_DEBUG_RESET_MCU:
      // DPARKER modified for testing reset while running
      if (next_message.data_2 == ETM_CAN_ADDR_ETHERNET_BOARD) {
	__asm__ ("Reset");
      } else {
	SendSlaveReset(next_message.data_2);
      }
      /*
      if ((global_data_A36507.control_state < STATE_DRIVE_UP) || (global_data_A36507.control_state > STATE_XRAY_ON)) {
	SendSlaveReset(next_message.data_2);
      }
      */
      break;

    /*
    case REGISTER_SPECIAL_SEND_ALL_CAL_DATA_TO_GUI:
      // DPARKER Figure out how to impliment this - Is it even possible?
      SendSlaveUploadAllCalData(next_message.data_2);
      break;
    */

      /*
    case REGISTER_DEBUG_TOGGLE_RESET:
      if (_SYNC_CONTROL_RESET_ENABLE) {
	_SYNC_CONTROL_RESET_ENABLE = 0;
      } else {
	_SYNC_CONTROL_RESET_ENABLE = 1;
      }
      break;
      */

    case REGISTER_SYSTEM_SET_TIME:
      temp_long = next_message.data_2;
      temp_long <<= 16;
      temp_long += next_message.data_1;
      RTCSecondsToDate(temp_long, &set_time);
      SetDateAndTime(&U6_DS3231, &set_time);
      break;

    case REGISTER_DEBUG_TEST_PULSE_FAULT:
      ETMCanMasterSendMsg((ETM_CAN_MSG_CMD_TX | (ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD << 2)),
			  0x22FF,
			  0,
			  0,
			  0);
      break;
      /*
    case REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_START:
      CalculatePulseSyncParams(next_message.data_2, psync_grid_stop_high_intensity_3);
      break;


    case REGISTER_SPECIAL_2_5_SET_DOSE_DYNAMIC_STOP:
      CalculatePulseSyncParams(psync_grid_start_high_intensity_3, next_message.data_2);
      break;


    case REGISTER_SPECIAL_2_5_SET_PFN_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      psync_pfn_delay_high = temp;
      psync_pfn_delay_low = temp;
      temp <<= 8;
      temp += (temp_array[2] & 0x00FF);
      temp_array[2] = temp;
      temp_array[10] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, &temp_array[0]);
      break;


    case REGISTER_SPECIAL_2_5_SET_AFC_SAMPLE_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      psync_afc_delay_high = temp;
      psync_afc_delay_low = temp;
      temp <<= 8;
      temp += (temp_array[6] & 0x00FF);
      temp_array[6] = temp;
      temp_array[14] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, &temp_array[0]);
      break;


    case REGISTER_SPECIAL_2_5_SET_MAGNETRON_CURRENT_SAMPLE_DELAY:
      ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      temp =  next_message.data_2;
      if (temp > 255) {
	temp = 255;
      }
      psync_mag_delay_high = temp;
      psync_mag_delay_low = temp;
      
      temp += (temp_array[6] & 0xFF00);
      temp_array[6] = temp;
      temp_array[14] = temp;
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, &temp_array[0]);
      ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, &temp_array[0]);
      break;


    case REGISTER_SPECIAL_2_5_SET_HV_LAMBDA_VOLTAGE:
      local_hv_lambda_low_en_set_point  = next_message.data_2;     
      local_hv_lambda_high_en_set_point = next_message.data_2;

      eeprom_register = REGISTER_HIGH_ENERGY_SET_POINT + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
 
      eeprom_register = REGISTER_LOW_ENERGY_SET_POINT + 2 * personality;
      ETMEEPromWriteWord(eeprom_register, next_message.data_2);
      break;
      */

    case REGISTER_SYSTEM_ENABLE_HIGH_SPEED_LOGGING:
      // Clear the Logging registers
      ETMCanMasterClearHighSpeedLogging();
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 1;
      break;
      
    case REGISTER_SYSTEM_DISABLE_HIGH_SPEED_LOGGING:
      _SYNC_CONTROL_HIGH_SPEED_LOGGING = 0;
      break;

      /*
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
      */

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
    
    psync_grid_start_high_intensity_3 = start_max;
    psync_grid_start_high_intensity_2 = start_med;
    psync_grid_start_high_intensity_1 = start_small;
    psync_grid_start_high_intensity_0 = start_min;

    psync_grid_start_low_intensity_3 = psync_grid_start_high_intensity_3;
    psync_grid_start_low_intensity_2 = psync_grid_start_high_intensity_2;
    psync_grid_start_low_intensity_1 = psync_grid_start_high_intensity_1;
    psync_grid_start_low_intensity_0 = psync_grid_start_high_intensity_0;
 
    psync_grid_stop_high_intensity_3 = stop_max;      
    psync_grid_stop_high_intensity_2 = stop_med;      
    psync_grid_stop_high_intensity_1 = stop_small;      
    psync_grid_stop_high_intensity_0 = stop_min;      

    psync_grid_stop_low_intensity_3 = psync_grid_stop_high_intensity_3;
    psync_grid_stop_low_intensity_2 = psync_grid_stop_high_intensity_2;
    psync_grid_stop_low_intensity_1 = psync_grid_stop_high_intensity_1;
    psync_grid_stop_low_intensity_0 = psync_grid_stop_high_intensity_0;
  }
  
  ETMEEPromWritePage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1 + 0), 16, (unsigned int*)&mirror_pulse_sync.local_data[0]);
  ETMEEPromWritePage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2 + 0), 16, (unsigned int*)&mirror_pulse_sync.local_data[0]);
  ETMEEPromWritePage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3 + 0), 16, (unsigned int*)&mirror_pulse_sync.local_data[0]);
  ETMEEPromWritePage((EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4 + 0), 16, (unsigned int*)&mirror_pulse_sync.local_data[0]);
}



void WriteConfigToMirror(void) {
  unsigned int temp_data[16];

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HTR_MAG_AFC, 16, temp_data);
  
  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HV_LAMBDA, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_GUN_DRV, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_1, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_2, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_3, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_4, 16, temp_data);
}

void ReadConfigFromMirror(void) {
  unsigned int temp_data[16];

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HTR_MAG_AFC, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HTR_MAG_AFC, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_HV_LAMBDA, 16, temp_data); 
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_HV_LAMBDA, 16, temp_data);
 
  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_GUN_DRV, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_GUN_DRV, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_1, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_1, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_2, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_2, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_3, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_3, 16, temp_data);

  ETMEEPromReadPage(EEPROM_PAGE_SYSTEM_CONFIG_MIRROR_PULSE_SYNC_PER_4, 16, temp_data);
  ETMEEPromWritePage(EEPROM_PAGE_SYSTEM_CONFIG_PULSE_SYNC_PER_4, 16, temp_data);
}




void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
  _U1RXIF = 0;
  while (U1STAbits.URXDA) {
    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
  }
}

unsigned int LookForDoseMessageFromReferenceDetector(void) {
  unsigned int crc_received = 0;
  unsigned int crc_calc = 0;
  unsigned int message_received = 0;
  // Look for messages in the UART Buffer;
  // If multiple messages are found the old data is overwritten by the newer data
  unsigned char message[9];
  
  while (BufferByte64BytesInBuffer(&uart1_input_buffer) >= 9) {
    // Look for message
    test_uart_data_recieved++;
    message[0] = BufferByte64ReadByte(&uart1_input_buffer);
    if (message[0] != 0x01) {
      continue;
    }
    message[1] = BufferByte64ReadByte(&uart1_input_buffer);
    if (message[1] != 0x01) {
      continue;
    }
    message[2] = BufferByte64ReadByte(&uart1_input_buffer);
    if (message[2] != 0x04) {
      continue;
    }
    message[3] = BufferByte64ReadByte(&uart1_input_buffer);
    if (message[3] != 0x10) {
      continue;
    }
    message[4] = BufferByte64ReadByte(&uart1_input_buffer);
    if (message[4] != 0x18) {
      continue;
    }
    message[5] = BufferByte64ReadByte(&uart1_input_buffer);
    message[6] = BufferByte64ReadByte(&uart1_input_buffer);
    message[7] = BufferByte64ReadByte(&uart1_input_buffer);
    message[8] = BufferByte64ReadByte(&uart1_input_buffer);
    
    crc_received = message[8];
    crc_received <<= 8;
    crc_received += message[7];

    crc_calc = ETMCRC16(&message[0], 7);
    test_ref_det_recieved++;
    if (crc_received == crc_calc) {
      test_ref_det_good_message++;
      // The CRC Matched
      // Update the dose data for the previous pulse
      global_data_A36507.most_recent_ref_detector_reading = message[6];
      global_data_A36507.most_recent_ref_detector_reading <<= 8;
      global_data_A36507.most_recent_ref_detector_reading += message[5];
      message_received = 1;
    }
  }
  return message_received;
}



void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  
  Nop();
  Nop();
  __asm__ ("Reset");
}



void CRCTest(void) {
  unsigned char uart_message[9];
  unsigned int uart_message_int[5];

  unsigned int result_1;
  unsigned int result_2;

  uart_message_int[0] = 0x0101;
  uart_message_int[1] = 0x1004;
  uart_message_int[2] = 0x8818;
  uart_message_int[3] = 0x0200;
  uart_message_int[4] = 0xFF16;

  result_1 = ETMCRC16(&uart_message_int[0], 6);
  result_2 = ETMCRCModbus(&uart_message_int[0], 6);

 
  uart_message[0] = 0x01;
  uart_message[1] = 0x01;
  uart_message[2] = 0x04;
  uart_message[3] = 0x10;
  uart_message[4] = 0x18;
  uart_message[5] = 0x88;
  uart_message[6] = 0x00;
  uart_message[7] = 0x02;
  uart_message[8] = 0x16;
  
  result_1 = ETMCRC16(&uart_message[0], 6);
  result_2 = ETMCRCModbus(&uart_message[0], 6);
  Nop();
  Nop();
  Nop();

  uart_message[0] = 0x01;
  uart_message[1] = 0x01;
  uart_message[2] = 0x04;
  uart_message[3] = 0x10;
  uart_message[4] = 0x18;
  uart_message[5] = 0x05;
  uart_message[6] = 0xF7;
  uart_message[7] = 0x26;
  uart_message[8] = 0xC0;

  result_2 = ETMCRC16(&uart_message[0], 7);

  Nop();
  Nop();
  Nop();
  Nop();
}


