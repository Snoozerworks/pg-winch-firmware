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

	char drum_spd;				// Drum speed in 1/10 x rpm, i.e. 1=10 rpm. Negative speed unwinds.
	byte engi_spd;				// Engine speed in 1/20 x rpm, i.e. 1=20 rpm.
	byte pump_spd;				// Pump speed in 1/10 x rpm, i.e. 1=10 rpm.
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
		byte dc;						// Drum tachometer count
		byte pc;						// Pump tachometer count
		byte ec;						// Engine tachometer count
		static unsigned int pump_filter = 0; 	// Filter
		unsigned int duration;
		unsigned int dt, et, pt;	// Timings for drum, engine and pump

		char new_gear;
		char new_drum_spd;

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

		// Calculcate drum speed. Ranges from -1270 to 1270rpm with 10 rpm resolution.
		if (dc > 0) {
			// At least one tick since last sample
			duration = dt - drum_time_old;
			drum_time_old = dt;
			new_drum_spd = min(127, (long )(dc * MAX_DELAY_DRUM) / duration); // rpm * 0.1

		} else {
			// No tick during last sample.
			duration = time - drum_time_old;
			new_drum_spd = min(abs(drum_spd), MAX_DELAY_DRUM / duration);
			if (duration > MAX_DELAY_DRUM) {
				// Pulse duration longer than min detecable speed. Set old time so speed become zero.
				drum_time_old = mark_time - MAX_DELAY_DRUM - 1;

			}

		}

		// Calculcate engine speed. Ranges from 0 to 5100rpm with 20 rpm resolution.
		if (ec > 0) {
			// At least one tick since last sample
			duration = et - engi_time_old;
			engi_time_old = et;
			engi_spd = min(255, (long )(ec * MAX_DELAY_ENGI) / duration); // rpm * 0.05

		} else {
			// No tick during last sample.
			duration = time - engi_time_old;
			engi_spd = min(engi_spd, MAX_DELAY_ENGI / duration);
			if (duration > MAX_DELAY_ENGI) {
				// Pulse duration longer than min detecable speed. Set old time so speed become zero.
				engi_time_old = mark_time - MAX_DELAY_ENGI - 1;

			}

		}

		// Calculcate pump speed. Ranges from 0 to 2550rpm with 10rpm resolution.
		if (pc > 0) {
			// At least one tick since last sample
			duration = pt - pump_time_old;
			pump_time_old = pt;
			pump_spd = min(255, (long )(pc * MAX_DELAY_PUMP) / duration); // rpm * 0.1

		} else {
			// No tick during last sample.
			duration = time - pump_time_old;
			pump_spd = min(pump_spd, MAX_DELAY_PUMP / duration); // rpm * 0.1

			if (duration > MAX_DELAY_PUMP) {
				// Pulse duration longer than min detecable speed. Set old time so speed become zero.
				pump_time_old = mark_time - MAX_DELAY_PUMP - 1;

			}

		}

		//
		// Calculate current gear and direction of the drum. The relation
		// between drum, engine and pump velocities is;
		// 		engine_velocity = U * (drum_velocity + pump_velocity).
		//
		// The algorithm finds what gear ratio U that best matches the relation
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
			drum_spd = -new_drum_spd;

		} else {
			// Gear not in neutral.
			new_gear = get_cur_gear_ratio(engi_spd, pump_spd, new_drum_spd);
			if (new_gear < 0) {
				new_drum_spd = -new_drum_spd;
				new_gear = -new_gear;
			}
			// Drum speed shall increase/decrease if gear decrease/increase. If
			// not, keep gear of previous sample.
			if ((new_gear - gear) * (new_drum_spd - drum_spd) > 0) {
				// Gear change valid
				gear = new_gear;
				drum_spd = new_drum_spd;
			} else {
				drum_spd =
						(drum_spd < 0) ? -abs(new_drum_spd) : abs(new_drum_spd);
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
	 * Returned value can be +/- 1,2 or 3. Negative value indicates negative
	 * drum direction.
	 *
	 * Drum speed in rpm is pump_spd*10
	 * Engine speed in rpm is engi_spd*20
	 * Pump speed in rpm is pump_spd*10
	 *
	 * Gear ratios are 1/100*GEAR_X_RATIO for X=1,2,3
	 * So, minimize;
	 * 	   | engi_spd*20 - GEAR_X_RATIO/100 * (drum_spd*10 + pump_spd*10) |
	 * 	...which can be simplified to
	 * 	   | 200*engi_spd - GEAR_X_RATIO * (drum_spd + pump_spd) |
	 */
	char get_cur_gear_ratio(byte e_spd, byte p_spd, char d_spd) const {
		long e_term, diff1, diff2;
		char gear;	// Current gear 1,2 or 3. Sign equals drum direction.

		e_term = 200L * e_spd;

		// First gear, positive drum speed
		diff1 = e_term - GEAR_1_RATIO * (p_spd + d_spd);
		gear = 1;
		// First gear, negative drum speed
		diff2 = e_term - GEAR_1_RATIO * (p_spd - d_spd);
		if (abs(diff2) < abs(diff1)) {
			gear = -1;
			diff1 = diff2;
		}
		// Second gear, positive drum speed
		diff2 = e_term - GEAR_2_RATIO * (p_spd + d_spd);
		if (abs(diff2) < abs(diff1)) {
			gear = 2;
			diff1 = diff2;
		}
		// Second gear, negative drum speed
		diff2 = e_term - GEAR_2_RATIO * (p_spd - d_spd);
		if (abs(diff2) < abs(diff1)) {
			gear = -2;
			diff1 = diff2;
		}
		// Third gear, positive drum speed
		diff2 = e_term - GEAR_3_RATIO * (p_spd + d_spd);
		if (abs(diff2) < abs(diff1)) {
			gear = 3;
			diff1 = diff2;
		}
		// Third gear, negative drum speed
		diff2 = e_term - GEAR_3_RATIO * (p_spd - d_spd);
		if (abs(diff2) < abs(diff1)) {
			gear = -3;
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
		byte tx_buffer[12];
		tx_buffer[0] = mode;
		tx_buffer[1] = time >> 24;	// Skip msb? Use for servopos instead?
		tx_buffer[2] = time >> 16;
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
