/*
 * machine_state.h
 *
 *  Created on: 16 aug 2014
 *      Author: markus
 */

#ifndef MACHINESTATE_H
#define MACHINESTATE_H

// Libraries
#include <I2C.h>

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
	volatile byte _engi_ticks;	// Tachometer counter used by pin A2

	volatile unsigned int _drum_time; 	// Tachometer interrupt 1 timings
	volatile unsigned int _pump_time; 	// Tachometer interrupt 0 timings
	volatile unsigned int _engi_time; 	// Tachometer interrupt A2

	char gear;	// Current gear 0 (neutral), 1, 2 or 3.

	// Time for tachometer ticks at previous sample.
	unsigned int drum_time_old; 	// Tachometer interrupt 1 timings
	unsigned int pump_time_old; 	// Tachometer interrupt 0 timings
	unsigned int engi_time_old; 	// Tachometer interrupt A2

	unsigned int mark_time;		// Time for next sample.
	unsigned int time;			// Time for current sample.
	byte mode; 					// Mode of operation
	byte errors; 				// Error states (active/inactive)
	byte drum_err_cnt; 			// Drum speed error counter
	byte pump_err_cnt; 			// Pump speed error counter

	char drum_spd;		// Pulses per time MAX_DELAY_DRUM. Negative speed unwinds line.
	byte engi_spd;		// Pulses per time MAX_DELAY_ENGI.
	byte pump_spd;		// Pulses per time MAX_DELAY_PUMP.
//	char drum_spd_f;			// Filtered drum speed.
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
	 * Also see http://www.edn.com/design/systems-design/4320010/A-simple-software-lowpass-filter-suits-embedded-system-applications
	 *
	 */
	void read_tachometers() {
		byte dc, ec, pc;					// Drum, pump and engine tachometer count
		unsigned int dt, et, pt;	// Timings for drum, engine and pump
		static unsigned int pump_filter = 0;	 // Filter
		unsigned int duration;

		char proposed_gear;
		//char new_drum_spd;
		char drum_spd_old = drum_spd;

		// Get tachometer counts and timings.
		noInterrupts();
		dc = _drum_ticks;
		ec = _engi_ticks;
		pc = _pump_ticks;

		_drum_ticks = 0;
		_engi_ticks = 0;
		_pump_ticks = 0;

		dt = _drum_time;
		et = _engi_time;
		pt = _pump_time;
		interrupts();

		// Get elapsed time since last drum tick. If speed is 0, assume MAX_DELAY_ .
		duration = (drum_spd == 0) ? MAX_DELAY_DRUM : dt - drum_time_old;
		drum_time_old = dt;

		// Calculate drum speed (without sign).
		if (duration == 0) {
			// Avoid divide by zero. Set max speed.
			drum_spd = 127;
		} else if (dc == 0) {
			// No tick during last sample.
			drum_spd = min(127, MAX_DELAY_DRUM / duration);
		} else {
			// At least one tick since last sample.
			drum_spd = min(127,
					((unsigned long )(dc * MAX_DELAY_DRUM)) / duration);
		}

		// Get elapsed time since last engine tick. If speed is 0, assume MAX_DELAY_ .
		duration = (engi_spd == 0) ? MAX_DELAY_ENGI : et - engi_time_old;
		engi_time_old = et;

		// Calculate engine speed (without sign).
		if (duration == 0) {
			// Avoid divide by zero. Set max speed.
			engi_spd = 255;
		} else if (ec == 0) {
			// No tick during last sample.
			engi_spd = min(255, MAX_DELAY_ENGI / duration);
		} else {
			// At least one tick since last sample.
			engi_spd = min(255,
					((unsigned long )(ec * MAX_DELAY_ENGI)) / duration);
		}

		// Get elapsed time since last pump tick. If speed is 0, assume MAX_DELAY_ .
		duration = (pump_spd == 0) ? MAX_DELAY_PUMP : pt - pump_time_old;
		pump_time_old = pt;

		// Calculate pump speed (without sign).
		if (duration == 0) {
			// Avoid divide by zero. Set max speed.
			pump_spd = 255;
		} else if (pc == 0) {
			// No tick during last sample.
			pump_spd = min(255, MAX_DELAY_PUMP / duration);
		} else {
			// At least one tick since last sample.
			pump_spd = min(255,
					((unsigned long )(pc * MAX_DELAY_PUMP)) / duration);
		}

		//
		// Calculate current gear and direction of the drum. The relation
		// between drum, engine and pump velocities is;
		//		 engine_velocity = U * (drum_velocity + pump_velocity).
		//
		// The algorithm finds the gear ratio U that best matches the relation
		// testing both positive and negative drum speeds. Both directions are
		// considered since drum direction is not directly measured. However,
		// due to not measuring the drum direction, the result may be invalid at
		// some points. This may happen when two gear ratios match the relation;
		// one for the positive drum speed and another for the negative drum
		// speed.
		// To avoid invalid gear shifts calculations, result is validated
		// against the drum speed such that drum speed must increase for an
		// up-shift and decrease for a down-shift. If not, gear calculated from
		// previous sample will remain valid.
		// If gear is in neutral, gear ratio will be 0 and drum speed negative,
		// drum is assumed to unwind.
		//

		// Calculate gear ratio and drum direction.
		if (chk_bits(sw, C::SW_NE)) {
			// Gear in neutral. Drum can only unwind.
			gear = 0;
			drum_spd = -drum_spd;

		} else {
			// Gear not in neutral.
			proposed_gear = get_cur_gear_ratio(engi_spd, pump_spd, drum_spd);

			// New gear. Validate gear change.
			char cur_dir = (drum_spd_old < 0) ? -1 : 1;
			char new_dir = 1;
			if (proposed_gear < 0) {
				new_dir = -1;
				proposed_gear = -proposed_gear;
			}

			if (cur_dir == new_dir) {
				// No drum direction change. Just update gear and speed.
				drum_spd *= cur_dir;
				gear = proposed_gear;

			} else if (abs(drum_spd_old) + abs(drum_spd) <= 3) {
				// Drum direction change? Only if speed change is small.
				// Update speed only.
				drum_spd *= new_dir;
				gear = proposed_gear;

			} else {
				// Keep drum direction and gear but updated speed.
				drum_spd *= cur_dir;
			}

		}

		// Filter pump speed
		pump_filter = pump_filter - (pump_filter >> FILTER_SHIFT) + pump_spd;
		pump_spd_f = (byte) (pump_filter >> FILTER_SHIFT);

	}

	/**
	 * Esitmate current gear and drum direction.
	 *
	 * Method returns the gear with ratio U that minimizes the relation
	 * 		| engine_velocity - U * (pump_velocity +/- drum_velocity) |
	 *
	 * Returned value can be -1, 1, 2 or 3. Negative value indicates a negative
	 * drum direction (unwinding).
	 *
	 */
	char get_cur_gear_ratio(byte e_spd, byte p_spd, byte d_spd) const { 
    // Gear ratios are 1/100*GEAR_X_RATIO for X=1,2,3
    // So, find ratio tha minimize;
    //			|a - GEAR_X_RATIO * (b + c)| or
    //      |a - GEAR_X_RATIO * (b - c)|
    // where
    //		 a = engi_spd * TO_RPM_ENGI * 100;
    //		 b = pump_spd * TO_RPM_DRUM;
    //		 c = drum_spd * TO_RPM_PUMP;

		char gear;	// Current gear 1,2 or 3. Sign equals drum direction.
		long diff1;	// Current best guess of gear ratio.
		long diff2;	// Next guess of gear ratio.

		long a = e_spd * TO_RPM_ENGI * 100;
		long b = p_spd * TO_RPM_DRUM;
		long c = d_spd * TO_RPM_PUMP;

    gear = -1;

		// First gear, negative drum speed.
		// Negative drum speed may occur only in 1st gear.
		diff1 = abs( a - GEAR_1_RATIO * (b-c) );

		// First gear, positive drum speed
		diff2 = abs( a - GEAR_1_RATIO * (b+c) );
		if (diff2 < diff1) {
			gear = 1;
			diff1 = diff2;
		}

		// Second gear, positive drum speed
		diff2 = abs( a - GEAR_2_RATIO * (b+c) );
		if (diff2 < diff1) {
			gear = 2;
			diff1 = diff2;
		}

		// Third gear, positive drum speed
		diff2 = abs( a - GEAR_3_RATIO * (b+c) );
		if (diff2 < diff1) {
			gear = 3;
			diff1 = diff2;
		}

		return gear;
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
	 * High byte are sent before low bytes (big endian byte order). In total 13
	 * bytes are sent;
	 *
	 * byte  0 : mode
	 * byte  1 : timestamp byte 1 (high byte)
	 * byte  2 : timestamp byte 2
	 * byte  3 : timestamp byte 3
	 * byte  4 : timestamp byte 4 (low byte)
	 * byte  5 : errors
	 * byte  6 : pump speed
	 * byte  7 : drum speed
	 * byte  8 : engine speed
	 * byte  9 : temperature (high byte)
	 * byte 10 : temperature (low byte)
	 * byte 11 : pressure (high byte)
	 * byte 12 : pressure (low byte)
	 *
	 */
	void serial_send() {
		byte tx_buffer[13];
		tx_buffer[0] = mode;
		tx_buffer[1] = 0; // time >> 24;	// Skip msb? Use for servopos instead?
		tx_buffer[2] = 0; // time >> 16;
		tx_buffer[3] = time >> 8;
		tx_buffer[4] = time;
		tx_buffer[5] = errors;
		tx_buffer[6] = pump_spd;
		tx_buffer[7] = drum_spd;
		tx_buffer[8] = engi_spd;
		tx_buffer[9] = temp >> 8;
		tx_buffer[10] = temp;
		tx_buffer[11] = pres >> 8;
		tx_buffer[12] = pres;

		Serial.write(tx_buffer, sizeof(tx_buffer));
		Serial.flush();
	}

};
// End namespace MachineState

#endif /* MACHINESTATE_H */
