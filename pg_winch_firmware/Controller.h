#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "parameter.h"

/**
 * Controller "bang-bang PID" 
 *
 * Output calculated as Kp*error + Ki*integral + Kd*derivative.
 * Integral can be limited by imax. Output is limited to 0..255.
 *  
 * Note!
 * Kp, Ki and Kd are specified by global parameters I_PID_P, I_PID_I and
 * I_PID_D. These must not be outside range -128..127 !
 * 
 * Controller process value PV range is 0..255.
 * If controller error (setpoint-PV) exceed +/-127 controller output is
 * maximized (minimized) to 0 or 255.
 */
class Controller {
private:
	int error;
	int integral;
	int preError;
	int derivative;

public:
	byte setpoint; // Controller setpoint value (pump speed)

	/**
	 * Reset controller to zero (p,i and d parts) except setpoint value.
	 *
	 * @returns void
	 */
	void reset() {
		// Private initiation
		error = 0;
		preError = 0;
		integral = 0;
		derivative = 0;
	}

	/**
	 * The controller to regulate pump speed.
	 *
	 * By also supplying engine speed, compensation for throttle response at
	 * different engine speeds can be made.
	 *
	 * See parameters.h for parameters to adjust the controller.
	 *
	 * @param byte pump_spd Process value 0-255.
	 * @param byte engine_spd Process value 0-127.
	 * @return byte signal 0-255.
	 */
	byte process(byte pump_spd, byte engi_spd) {
		using namespace P;

		int eng_spd_comp; // Gain compensation for engine speed
		long output;

		// calculate error
		error = setpoint - pump_spd; // [-127,127] (limited below)

		// Maximise/minimise output if error is too big/small.
		if (error > 127) {
			integral = derivative = 0;
			return 255;
		}
		if (error < -127) {
			integral = derivative = 0;
			return 0;
		}

		// track error over time
		integral = constrain(integral + error, 0, params[I_PID_IMAX].val); // [0, 255]

		// determine the amount of change from the last time checked
		derivative = constrain(error - preError, -64, 64); // [-64770, 64770] => [-64, 64]

		// remember the error for the next time around.
		preError = error;

		// To linearise throttle response, which is believed to be a function of
		// engine speed, the compensation factor eng_spd_comp is used. It's applied
		// as a output gain.
		eng_spd_comp = 8 - ((engi_spd * params[I_PID_K].val) / 64);
		eng_spd_comp = constrain(eng_spd_comp, 1, 8);

		output = output * eng_spd_comp;

		return (byte) output;
	}

};

#endif /* CONTROLLER_H */
