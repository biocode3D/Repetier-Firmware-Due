#ifndef PINS_H
#define PINS_H


/*
The board assignment defines the capabilities of the motherboard and the used pins.
Each board definition follows the following scheme:

CPU_ARCH
  ARCH_AVR for AVR based boards
  ARCH_ARM for all arm based boards

STEPPER_CURRENT_CONTROL
  CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
  CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
  CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master

*/

#define ARCH_AVR 1
#define ARCH_ARM 2

#define CURRENT_CONTROL_MANUAL  1  // mechanical poti, default if not defined
#define CURRENT_CONTROL_DIGIPOT 2  // Use a digipot like RAMBO does
#define CURRENT_CONTROL_LTC2600 3  // Use LTC2600 like Foltyn 3D Master



#if MOTHERBOARD == 401
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define X_STEP_PIN     54  // A0
#define X_DIR_PIN      55  // A1
#define X_MIN_PIN      3
#define X_MAX_PIN      2
#define X_ENABLE_PIN   38

#define Y_STEP_PIN     60  // A6 
#define Y_DIR_PIN      61  // A7
#define Y_MIN_PIN      14
#define Y_MAX_PIN      15
#define Y_ENABLE_PIN   56  // A2

#define Z_STEP_PIN     46
#define Z_DIR_PIN      48
#define Z_MIN_PIN      18
#define Z_MAX_PIN      19
#define Z_ENABLE_PIN   62  // A8

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN   10
#define TEMP_0_PIN     11   // Due analog pin #
#define HEATER_1_PIN   8
#define TEMP_1_PIN     12  // Due analog pin #
#define HEATER_2_PIN   9
#define TEMP_2_PIN     13  // Due analog pin #

#define E0_STEP_PIN    26
#define E0_DIR_PIN     28
#define E0_ENABLE_PIN  24

#define E1_STEP_PIN    36
#define E1_DIR_PIN     34
#define E1_ENABLE_PIN  40

#define SDPOWER 	   -1
#define SDSS		   10 // 53
#define LED_PIN 	   13
#define FAN_PIN 	   -1
#define PS_ON_PIN      12
#define KILL_PIN	   -1
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.


// Available chip select pins for HW SPI are 4 10 52
#if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) 
#else
#define DUE_SOFTWARE_SPI
#define MOSI_PIN		51
#define MISO_PIN		50
#define SCK_PIN 		52
#endif

#define SDA_PIN 				20  	// 20 or 70
#define SCL_PIN 				21  	// 21 or 71
#define TWI_INTERFACE   		TWI1	// TWI1 if pins 20  TWI0 for pins 70
#define TWI_ID  				ID_TWI1


#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,
#define E1_PINS E1_STEP_PIN,E1_DIR_PIN,E1_ENABLE_PIN,

#endif


#ifndef SDSSORIG
#define SDSSORIG -1
#endif

#ifndef STEPPER_CURRENT_CONTROL // Set default stepper current control if not set yet.
#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_MANUAL
#endif

#ifndef FAN_BOARD_PIN
#define FAN_BOARD_PIN -1
#endif

#if NUM_EXTRUDER==1
#define E1_PINS
#endif

#if NUM_EXTRUDER<3
#define E2_PINS
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, LED_PIN, PS_ON_PIN, \
						HEATER_0_PIN, HEATER_1_PIN, FAN_PIN, E0_PINS E1_PINS E2_PINS TEMP_0_PIN, TEMP_1_PIN,SDSS }
#endif

