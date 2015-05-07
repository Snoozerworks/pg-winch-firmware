/*
 * machine_state.h
 *
 *  Created on: 16 aug 2014
 *      Author: markus
 */

#ifndef MACHINESTATE_H
#define MACHINESTATE_H

// Project files to include
#include "types.h"
#include "parameter.h"

/**
 * Namespace for state variables and functions.
 */
class MachineState {
public:
	volatile byte _drum_ticks; 	// Tachometer counter used by interrupt 1
	volatile byte _pump_ticks; 	// Tachometer counter used by interrupt 0

	unsigned long mark_time;	// Time mark
	unsigned long time;			// Timestamp
	byte mode; 					// Mode of operation
	byte errors; 				// Error states (active/inactive)
	byte drum_err_cnt; 			// Drum speed error counter
	byte pump_err_cnt; 			// Pump speed error counter

	byte drum_spd;				// Sampled drum speed in 2 x pulse/sample
	byte pump_spd;				// Sampled pump speed in 2 x pulse/sample
	byte drum_spd_f;			// Filtered drum speed.
	byte pump_spd_f;			// Filtered pump speed.

	int temp; 					// Oil temperature
	int pres; 					// Pump pressure

	// Inputs
	byte sw; 					// Switch states bit field (on/off)

	/**
	 * Returns true if at least one bit in bitfield is active in in bitmask.
	 *
	 * @param byte Bits to look for in bitmask.
	 * @param byte Bits to compare bitfield with.
	 * @return boolean True if any bit is set.
	 */
	static boolean chk_bits(byte& bitfield, byte bitmask) {
		return (bitfield & bitmask);
	}

	/**
	 * Sets or clears bits in bitfield to bits in bitmask. If set is false, bits
	 * are unset otherwise they are set.
	 *
	 * @param set Flag true to set bits. Unset otherwise.
	 * @param bitfield Byte to set or unset bits in.
	 * @param bitmask Byte with bits to set or unset.
	 */
	static void set_bits(boolean set, byte& bitfield, byte bitmask) {
		if (set) {
			bitfield |= bitmask;
		} else {
			bitfield &= ~bitmask;
		}
	}

	/**
	 * Sample tachometer counts.
	 *
	 * Values of drum_spd, pump_spd and its filtered counterparts drum_spd_f
	 * and pump_spd_f are updated.
	 *
	 * The low-pass filter uses a local constant FILTER_SHIFT sets the filter
	 * characteristics.
	 *
	 * Also see http://www.edn.com/design/systems-design/4320010/A-simple-software-lowpass-filter-suits-embedded-system-applications *
	 */
	void read_tachometers() {
		static byte drum_ticks_old = 0;			// Previous sample of drum tachometer count
		static byte pump_ticks_old = 0;			// Previous sample of pump tachometer count
		byte drum_ticks;						// Drum tachometer count
		byte pump_ticks;						// Pump tachometer count
		static unsigned int drum_filter = 0; 	// Filter
		static unsigned int pump_filter = 0; 	// Filter

		// Read volatile variables just once
		drum_ticks = _drum_ticks;
		pump_ticks = _pump_ticks;

		// Update speed. Note multiplication by 2!
		drum_spd = (drum_ticks - drum_ticks_old) << 1;
		pump_spd = (pump_ticks - pump_ticks_old) << 1;

		// Update tachometer count history
		drum_ticks_old = drum_ticks;
		pump_ticks_old = pump_ticks;

		// Filter drum speed
		drum_filter = drum_filter - (drum_filter >> FILTER_SHIFT) + drum_spd;
		drum_spd_f = (byte) (drum_filter >> FILTER_SHIFT);

		// Filter pump speed
		pump_filter = pump_filter - (pump_filter >> FILTER_SHIFT) + pump_spd;
		pump_spd_f = (byte) (pump_filter >> FILTER_SHIFT);

	}

	/**
	 * Reads digital inputs, check serial for commands and updates machine state
	 * accordingly. Does not read tachometers.
	 */
	void read_switches() {
		byte rx_cmd; // Serial command byte

		// Read from serial to get any pending commands.
		rx_cmd = (Serial.available()) ? (byte) Serial.read() : C::CM_NOCMD;
		if (rx_cmd > C::_CM_LAST) {
			rx_cmd = C::CM_NOCMD;
		}

		// Read switches and update machine switch states.
		set_bits(Pins::SW_NE.read() == LOW, sw, C::SW_NE);
		set_bits(Pins::SW_SE.read() == LOW || rx_cmd == C::CM_SE, sw, C::SW_SE);
		set_bits(Pins::SW_UP.read() == LOW || rx_cmd == C::CM_UP, sw, C::SW_UP);
		set_bits(Pins::SW_DN.read() == LOW || rx_cmd == C::CM_DN, sw, C::SW_DN);
		set_bits(rx_cmd == C::CM_SP, sw, C::SW_SP);
		set_bits(rx_cmd == C::CM_GT, sw, C::SW_GT);

		// Read pressure
		pres = Pins::PRESSURE.read();

		// Check I2C bus error
		set_bits(0 != I2c.read(I2C_TMP_ADDR, (uint8_t) 2), errors, C::ERR_TWI);

		// Read temperature
		temp = ((int) I2c.receive() << 8 | (int) I2c.receive()) >> 7;

		// Check temperature fault conditions
		set_bits(temp > P::params[P::I_OIL_HI].val, errors, C::ERR_TEMP_HIGH);
		set_bits(temp < P::params[P::I_OIL_LO].val, errors, C::ERR_TEMP_LOW);
	}

	/**
	 * Write sensor data to Serial.
	 *
	 * High byte are sent before low bytes (big endian byte order). In total 11
	 * bytes are sent;
	 *
	 * byte  0 : mode
	 * byte  1 : timestamp byte 1 (high byte)
	 * byte  2 : timestamp byte 2
	 * byte  3 : timestamp byte 3
	 * byte  4 : timestamp byte 4 (low byte)
	 * byte  5 : pump speed
	 * byte  6 : drum speed
	 * byte  7 : temperature (high byte)
	 * byte  8 : temperature (low byte)
	 * byte  9 : pressure (high byte)
	 * byte 10 : pressure (low byte)
	 *
	 */
	void serial_send() {
		byte tx_buffer[11];
		tx_buffer[0] = mode;
		tx_buffer[1] = time >> 24;
		tx_buffer[2] = time >> 16;
		tx_buffer[3] = time >> 8;
		tx_buffer[4] = time;
		tx_buffer[5] = pump_spd;
		tx_buffer[6] = drum_spd;
		tx_buffer[7] = temp >> 8;
		tx_buffer[8] = temp;
		tx_buffer[9] = pres >> 8;
		tx_buffer[10] = pres;

		Serial.write(tx_buffer, sizeof(tx_buffer));
		Serial.flush();
	}

};
// End namespace MachineState

#endif /* MACHINESTATE_H */
