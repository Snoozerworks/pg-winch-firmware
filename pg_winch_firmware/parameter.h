#ifndef PARAMETERS_H
#define PARAMETERS_H

// Firmware version to display at startup
#define FW_VERSION __DATE__

#include "Arduino.h"
#include "types.h"

/**
 * This file contains various parameters used to setup, map input
 * and output pins, set sample time, timeouts, tachometer pulses etc.
 *
 * Note! Special characters to be printed on the lcd must be manually 
 * replaced with HEX codes as follows:
 *   å  - 128 - \x80
 *   ä  - 225 - \xE1
 *   ö  - 239 - \xEF
 *   Å  - 129 - \x81
 *   Ä  - 130 - \x82
 *   Ö  - 131 - \x83
 */

// Delays and times 
const byte T_SAMPLE = 125; // Sample period 0-255ms.
const word CONF_TIMEOUT = 6000; // Timeout in ms for exiting config mode.
const int READ_TIMEOUT = 70; // Timeout in ms for I2C and serial data.
const unsigned int GEAR_ENGAGE_DELAY = 2000; // Delay in milliseconds to let automatic gearbox engage.

// Pulses per turn for pump and drum tachometers
const byte PPT_DRUM = 8; // Pulses per turn for drum (equals the number of magnets)
const byte PPT_PUMP = 10; // Pulses per turn for pump (twice the number of magnets)
const byte PPT_ENGINE = 2; // Pulses per turn for pump (twice the number of magnets)

// Number of milliseconds between pulses of lowest detectable speed.
const unsigned int MAX_DELAY_DRUM = 750;  // Pules intervall for 10rpm
const unsigned int MAX_DELAY_ENGI = 1500; // Pules intervall for 20rpm
const unsigned int MAX_DELAY_PUMP = 600;  // Pules intervall for 10rpm

// Gear ratios x100, i.e. 811 is 8,11 x gearbox input = gearbox output.
const int GEAR_1_RATIO = 811;	// 1st gear
const int GEAR_2_RATIO = 491;	// 2nd gear
const int GEAR_3_RATIO = 339;	// 3rd gear

// I2C addresses
const byte I2C_LCD_ADDR = 0x63; // Display
const byte I2C_TMP_ADDR = 0x48; // Temperature sensor

// Tachometer sensor error trigger levels.
const byte TACH_PUMP_ERR_COUNT = 3; 	// Number of samples with pump zero speed before triggering a tachometer error.
const byte TACH_DRUM_ERR_COUNT = 2; 	// Number of samples with drum zero speed before triggering a tachometer error.
const byte ENGINE_ON_SPD 			 = 55;	// Engine on speed threashold value. 

// Tachometer filter parameter see types.h
const byte FILTER_SHIFT = 1; // Shift parameter. 1-3 should likely suffice.

// Manual throttle step (used in idle mode)
const byte MAN_THROTTLE_STEP = 10;

// Factor to reduce throttle position at drum overspeed.
// Is given in 10/255 % throttle per rpm overspeed. 
const byte THROTTLE_OVERSPEED_GAIN = 4;

//
// Pins are defined in namespace Pins
//
namespace Pins {
DigitalPin TACH_PUMP(2, INPUT_PULLUP); // Pump tachometer (interrupt 0)
DigitalPin TACH_DRUM(3, INPUT_PULLUP); // Drum tachometer (interrupt 1)
DigitalPin SW_NE(4, INPUT_PULLUP); // Shift lever neutral switch
DigitalPin SW_UP(5, INPUT_PULLUP); // Up button
DigitalPin SW_DN(6, INPUT_PULLUP); // Down button
DigitalPin SW_SE(7, INPUT_PULLUP); // Set button
DigitalPin ALIVE_LED(8, OUTPUT); // High when working, low when waiting
DigitalPin SERVO(11, OUTPUT); // Servo PWM pin
AnaloguePin PRESSURE(A0, INPUT); // Pump pressure

/* Not currently used */
DigitalPin OUT1(9, OUTPUT); // MOSFET output 1
DigitalPin OUT2(10, OUTPUT); // MOSFET output 2
//AnaloguePin SENSE_420(A1, INPUT); // Spare 4-20 mA input
}

//
// Strings are placed in namespace LCDStrings
//
namespace LCDStrings {
const char MSG_STARTUP[] = "\x0C\x04\x13" // Clear lcd, hide cursor and backlight on
				"UPPSTART"// Startup message including firmware version.
				"\rver " FW_VERSION// Include firmware compile date.
"\r!V\xE1xel ej i l\xE1ge N";
const char MSG_CONFIG[] = "\x0C" // Clear lcd, cursor home
				"INST\x82LLNINGAR   (";// Row 1
const char MSG_IDLE[] = "\x0C" // Clear lcd, cursor home
				"V\x82NTEL\x82GE"// Row 1
				"\r\rOljetemp:      \xDF\x43"// Row 3
				"\rGas     :      %";// Row 4
const char MSG_TOWING[] = "\x0C" // Clear lcd, cursor home
				"DRAGL\x82GE"// Row 1
				"\r\rPumpvarv:      rpm"// Row 3
				"\rGas     :      %";// Row 4
const char ERR_NO_ERR[] = "               ";
const char ERR_TEMP_HIGH[] = "!H\xEFg oljetemp";
const char ERR_TEMP_LOW[] = "!L\x80g oljetemp";
const char ERR_DRUM_MAX[] = "!Maxvarv trumma";
const char ERR_TWI[] = "!Temp.givarefel";
const char ERR_TACH0[] = "!Pumpgivarefel";
const char ERR_TACH1[] = "!Trumgivarefel";
const char VAL_FORMAT[] = "\x02\x15%s" // Error message on row 2
				"\x02\x33%4.0d"// Value on row 3
				"\x02\x48%3.0d";// Value on row 4
const char PRM_FORMAT[] = "\x02\x12%2d)" // Parameter index at row 1, col 18
				"%-20s"// Parameter description at row 2
				"\x02\x3D%6d";// Parameter value at row 4
}

//
// Parameters are placed in namespace P
//
namespace P {

// Index constants
const byte I_DRUM_SPD = 0;
const byte I_PUMP_RPM = 1;
const byte I_OIL_HI = 2;
const byte I_OIL_LO = 3; // Last operation parameter
const byte I_SERV_LO = 4;
const byte I_SERV_HI = 5;
const byte I_SERV_RST = 6;
const byte I_PID_P = 7;
const byte I_PID_I = 8;
const byte I_PID_D = 9;
const byte I_PID_K = 10;
const byte I_PID_IMAX = 11; // Last installation parameter

const byte OPER_PARAM_END = I_OIL_LO;
const byte INST_PARAM_END = I_PID_IMAX;

// See Parameter type definition for how to define a parameter.
Parameter params[] = { //
		{ 0, "Max linhastighet", 100, 73, 114, 1, 55, 86 }, //
		{ 1, "Pumpvarv", 13, 6, 35, 1, 60, 350 }, //
		{ 2, "Max oljetemp.", 140, 100, 180, 2, 50, 90 }, //
		{ 3, "Min oljetemp.", 20, 20, 80, 2, 10, 40 }, //
		{ 4, "Servo min-puls", 800, 600, 2400, 20, 600, 2400 }, //
		{ 5, "Servo max-puls", 2140, 600, 2400, 20, 600, 2400 }, //
		{ 6, "Servo reset tid", 250, 100, 600, 25, 100, 600 }, //
		{ 7, "PID p", 13, 1, 32, 1, 1, 32 }, //
		{ 8, "PID i", 5, 0, 32, 1, 0, 32 }, //
		{ 9, "PID d", 7, 0, 32, 1, 0, 32 }, //
		{ 10, "PID k", -17, -32, 32, 1, -32, 32 }, //
		{ 11, "PID i gr\xE1ns", 95, 0, 255, 1, 0, 255 } //
	};
}

#endif /* PARAMETERS_H */
