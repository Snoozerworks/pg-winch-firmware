#ifndef TYPES_H
#define TYPES_H

#include "Arduino.h"
#include <EEPROM.h>

/**
 * This file contains data type declarations for the winch software.
 */

/**
 * Arduino digital pin
 */
class DigitalPin {
public:
	uint8_t no;

	/**
	 * Initate pin i.e. set pin mode.
	 *
	 * @param n Pin number.
	 * @param m Pin mode.
	 */
	DigitalPin(uint8_t n, uint8_t m) {
		no = n;
		pinMode(n, m);
	}

	/**
	 * Read an digital pin
	 * @return int Digital HIGH or LOW.
	 */
	inline int read() {
		return digitalRead(no);
	}

	/**
	 * Write HIGH.
	 */
	inline void high() {
		digitalWrite(no, HIGH);
	}

	/**
	 * Write LOW.
	 */
	inline void low() {
		digitalWrite(no, LOW);
	}

};

/**
 * Arduino analogue pin
 */
class AnaloguePin {
public:
	uint8_t no;

	/**
	 * Initiate pin i.e. set pin mode.
	 *
	 * @param n Pin number.
	 * @param m Pin mode.
	 */
	AnaloguePin(const uint8_t n, uint8_t m) {
		this->no = n;
		pinMode(n, m);
	}

	/**
	 * Read an analogue pin
	 * @return int Analogue value.
	 */
	inline int read() {
		return analogRead(no);
	}
};

/**
 * Constants
 */
namespace C {
// Running modes
const byte MD_NOMODE = 0; // Undefined mode.
const byte MD_STARTUP = 1; // Startup checks
const byte MD_CONFIG_IS = 2; // Set parameters (config)
const byte MD_CONFIG_OS = 3; // Set parameters (operation)
const byte MD_IDLE = 4; // Winch in stand by (lever in neutral)
const byte MD_TOWING = 5; // In towing operation (lever not in neutral)

// Commands
const byte CM_NOCMD = 0; // No command
const byte CM_SE = 1; // Select switch active
const byte CM_UP = 2; // Up switch active
const byte CM_DN = 3; // Down switch active
const byte CM_SP = 4; // Set parameter (virtual) switch active
const byte CM_GT = 5; // Get sample (virtual) switch active
const byte _CM_LAST = CM_GT; // Last element. Used internally only.

// State of switches are saved in a structure.
const byte SW_NE = 1; // Neutral switch
const byte SW_SE = 2; // Select switch
const byte SW_UP = 4; // Up switch
const byte SW_DN = 8; // Down switch
const byte SW_SP = 16; // Set parameter (virtual) switch
const byte SW_GT = 32; // Get sample (virtual) switch
const byte SW_IS = 64; // Installation settings (virtual) switch

// State of errors are saved in a structure.
const byte ERR_TEMP_HIGH = 1; // Above high temperature limit
const byte ERR_TEMP_LOW = 2; // Below low temperature limit
const byte ERR_DRUM_MAX = 4; // Drum speed exceeded
const byte ERR_TWI = 8; // TWI bus error
const byte ERR_PUMP_SENSOR = 16; // Pump sensor fault
const byte ERR_DRUM_SENSOR = 32; // Drum sensor fault
}

/**
 * Parameters are saved in a common structure type.
 * 
 * Each parameter is defined by
 * - val      : The parameter value
 * - low      : The parameter lower bound
 * - high     : The parameter higher bound
 * - step     : The step by which to increment or decrement val.
 * - low_map  : Map the lower bound of a parameter to a custom scale.
 * - high_map : Map the higher bound of a parameter to a custom scale.
 * 
 * Map values (low_map/high_map) are used to convert parameter value to a unit
 * of own choice when displaying the value on the lcd.
 */
struct Parameter {
	const byte index; // Index in parameter array.
	const char descr[21]; // Parameter description
	int val; // Parameter value.
	int low; // Upper bound.
	int high; // Lower bound.
	int step; // Step. Amount to change val within limits of low and high.
	int low_map; // Display value upper bound.
	int high_map; // Display value lower bound.

	/**
	 * Load value from EEPROM.
	 */
	void eeprom_load() {
		byte hi, lo;
		if (EEPROM.read(index * 3) == 0) {
			hi = EEPROM.read(index * 3 + 1);
			lo = EEPROM.read(index * 3 + 2);
			set(hi << 8 | lo);
		}
	}

	/**
	 * Save values to EEPROM.
	 */
	void eeprom_save() {
		EEPROM.write(index * 3, 0);
		EEPROM.write(index * 3 + 1, (unsigned int) val >> 8);
		EEPROM.write(index * 3 + 2, val);
	}

	/**
	 * Increase value by delta within limits of ubound
	 */
	void increase() {
		set(val + step);
	}

	/**
	 * Increase value by delta within limits of ubound.
	 */
	void decrease() {
		set(val - step);
	}

	/**
	 * Set and save to EEPROM parameter value if not outside bounds.
	 *
	 * @param int new_val
	 */
	void set(int new_val) {
		val = max(min(new_val, high), low);
		eeprom_save();
	}

	/**
	 * Map value v from [low,high] to [low_map, high_map].
	 *
	 * @param int v Value to map.
	 * @return int Mapped value.
	 */
	int get_map(int v) {
		return low_map + ((long) v - low) * (high_map - low_map) / (high - low);
	}

	/**
	 * Write parameter to Serial. Total 14+21=35 bytes.
	 * High bytes are sent before low bytes (big endian byte order).
	 *
	 * @param mode Mode of operation.
	 * @param index Index of parameter in a list of parameters.
	 */
	void transmit(byte mode, byte index) {
		byte tx_buffer[14];

		tx_buffer[0] = mode;
		tx_buffer[1] = index;
		tx_buffer[2] = val >> 8;
		tx_buffer[3] = val;
		tx_buffer[4] = low >> 8;
		tx_buffer[5] = low;
		tx_buffer[6] = high >> 8;
		tx_buffer[7] = high;
		tx_buffer[8] = low_map >> 8;
		tx_buffer[9] = low_map;
		tx_buffer[10] = high_map >> 8;
		tx_buffer[11] = high_map;
		tx_buffer[12] = step >> 8;
		tx_buffer[13] = step;

		Serial.write(tx_buffer, sizeof(tx_buffer));
		Serial.write((byte*) descr, sizeof(descr));
		Serial.flush();
	}

};

#endif
