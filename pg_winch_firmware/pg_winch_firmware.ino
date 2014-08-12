// Libraries to include
#include <I2C.h>
#include <I2C_LCD.h>
#include <Servo.h>
#include <EEPROM.h>

// Project files to include
#include "types.h"
#include "parameter.h"
#include "controller.h"
#include "throttle_servo.h"

/**
 * REMINDER
 * 1. On linux Debian; to get the Arduino IDE use the bluetooth serial port
 * (/dev/rfcomm0) add user to dialout and tty group. Type;
 * > su
 * > usermod -a -G tty yourUserName
 * > usermod -a -G dialout yourUserName
 * Log off and log on again for the changes to take effect! 
 *
 * 2. Set Arduino IDE to use board "Arduino UNO"
 *
 * 3. The RN42 bluetooth module shall be configure with profile 4 "SPP and
 * DUN-DCE" or possibly with profile 1 "DUN-DCE" to make the arduino reset work.
 * This is done by sending the module first "$$$" to get in command mode. The
 * module shall respond with CMD. Then send "S~,4<CR>". To verify the setting
 * send "O<CR>" (O as in Oliver) and module will repond with a list of settings.
 * Don't include the "" when sending the commands.
 * 
 */

/* This is for debugging only */
/*
 int freeRam() {
 extern int __heap_start, *__brkval;
 int v;
 return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
 }
 */

// Machine states, servo object and controller object.
struct MachineState {
	byte md; // Mode of operation
	byte sw; // Switch states  (on/off)
	byte err; // Error states (active/inactive)
	byte pump_err_cnt; // Pump speed error counter
	byte drum_err_cnt; // Drum speed error counter
	unsigned long time; // Timestamp
	unsigned long mark_time; // Time mark

	SensorsState sensors; // Sensor states (sampled values)
	ThrottleServo servo; // Servo object
	Controller pid; // PID controller object
} M;

// Some other globals
char sprintfbuffer[96]; // Print buffers.
I2C_LCD lcd(I2C_LCD_ADDR); // Initiate object for lcd.

/**
 * Arduino entry point for program.
 */
void setup() {
	// Switch off outputs (inverted logic here...)
	Pins::OUT1.high();
	Pins::OUT2.high();

	// Initialize states
	M.md = C::MD_STARTUP; // Flag for startup
	M.mark_time = 0;

	// Start serial for RN-42 bluetooth module.
	Serial.begin(115200);
	Serial.setTimeout(READ_TIMEOUT);

	// Start I2C 
	I2c.begin();
	I2c.timeOut(READ_TIMEOUT);

	// Load parameters stored in flash memory
	for (byte i = 0; i <= P::INST_PARAM_END; i++) {
		P::params[i].eeprom_load();
	}

	// Allow devices to settle a bit
	delay(250);

	// Program custom lcd characters.
	lcd.loadCustomChars();

	// Show lcd startup message
	lcd.print(LCDStrings::MSG_STARTUP);

	// Prepare I2C temp sensor
	I2c.write(I2C_TMP_ADDR, (uint8_t) 0);

	// Setup pump and drum tachometers
	attachInterrupt(0, pump_tic, CHANGE);
	attachInterrupt(1, drum_tic, RISING);

	// Now enter main loop...
}

/**
 * Arduino main loop runs every T_SAMPLE period until power is shut-off.
 */
void loop() {

	// Loop until timeout
	do {
		M.time = millis(); // Only one call to millis()
	} while (M.mark_time > M.time);

	// Read inputs (except tachometer)
	read_switches();

	// Select mode. 
	mode_select();

	// Read tachometer inputs.
	M.sensors.tachometer_read();

	switch (M.md) {
	case C::MD_IDLE:
		idle_mode();
		break;

	case C::MD_TOWING:
		tow_mode();
		break;

	case C::MD_CONFIG_IS:
	case C::MD_CONFIG_OS:
		config_mode();
		break;

	case C::MD_STARTUP:
	case C::MD_NOMODE:
	default:
		break;
	}

	// Finish phase
	finish();

}

/**
 * Reads digital inputs, check serial for commands and updates machine state
 * accordingly. Does not read tachometers.
 */
void read_switches() {
	byte rx_cmd; // Serial command byte

	// Read from serial to get any pending commands.
	rx_cmd = (Serial.available()) ? (byte) Serial.read() : C::CM_NOCMD;
	if (rx_cmd > C::CM__LAST) {
		rx_cmd = C::CM_NOCMD;
	}

	//if (rx_cmd != C::CM_NOCMD) {
	//	lcd.print("\x02\x4E");
	//	lcd.print(rx_cmd+48); // Add 48 to get correct ascii.
	//}

	// Read switches and update machine switch states.
	set_bits(Pins::SW_NE.read() == LOW, M.sw, C::SW_NE);
	set_bits(Pins::SW_SE.read() == LOW || rx_cmd == C::CM_SE, M.sw, C::SW_SE);
	set_bits(Pins::SW_UP.read() == LOW || rx_cmd == C::CM_UP, M.sw, C::SW_UP);
	set_bits(Pins::SW_DN.read() == LOW || rx_cmd == C::CM_DN, M.sw, C::SW_DN);
	set_bits(rx_cmd == C::CM_SP, M.sw, C::SW_SP);
	set_bits(rx_cmd == C::CM_GT, M.sw, C::SW_GT);

	// Read pressure
	M.sensors.pres = Pins::PRESSURE.read();

	// Check I2C bus error
	set_bits(0 != I2c.read(I2C_TMP_ADDR, (uint8_t) 2), M.err, C::ERR_TWI);

	// Read temperature
	M.sensors.temp = ((int) I2c.receive() << 8 | (int) I2c.receive()) >> 7;

	// Check temperature fault conditions
	set_bits(M.sensors.temp > P::params[P::I_OIL_HI].val, M.err,
			C::ERR_TEMP_HIGH);
	set_bits(M.sensors.temp < P::params[P::I_OIL_LO].val, M.err,
			C::ERR_TEMP_LOW);
}

/**
 * This is the first step in each loop when running. Tachometers and neutral
 * switches are read first in the loop.
 */
void mode_select() {
	byte last_mode; // Last mode of operation	

	// Set alive pin high while working
	Pins::ALIVE_LED.high();

	// Update last mode.
	last_mode = M.md;

	// Check conditioins for changing mode.
	switch (M.md) {
	case C::MD_IDLE:
		if (chk_bits(M.sw, C::SW_NE)) {
			// Gear in neutral.
			if (chk_bits(M.sw, C::SW_SE | C::SW_SP)) {
				// Select switch or Installation settings (virtual) switch active.
				// Change to installation configuration mode.
				M.md = (chk_bits(M.sw, C::SW_SE)) ?
						C::MD_CONFIG_OS : C::MD_CONFIG_IS;

				// Reset and detach servo.
				M.servo.reset();
				M.servo.detach();
			}

		} else if (!chk_bits(M.err, C::ERR_TEMP_HIGH)) {
			// Gear engaged and oil temperature not too high.

			// Change to tow mode.
			M.md = C::MD_TOWING;

			// Reset tachometer errors each time tow mode is entered.
			set_bits(false, M.err, C::ERR_DRUM_SENSOR | C::ERR_PUMP_SENSOR);

			// Reset PID-controller to avoid servo glitches.
			M.pid.reset();

			// Wait for automatic transmission to engage.
			delay(GEAR_ENGAGE_DELAY);

		}
		break;

	case C::MD_TOWING:
		if (chk_bits(M.sw, C::SW_NE)) {
			// Gear in neutral. Reset servo and change to idle mode.
			M.md = C::MD_IDLE;
			M.servo.reset();
		}
		break;

	case C::MD_CONFIG_IS:
	case C::MD_CONFIG_OS:
		if (M.time - M.mark_time >= CONF_TIMEOUT) {
			// Settings mode timed out. Change to to idle mode.
			M.md = C::MD_IDLE;

			// Reconnect servo
			M.servo.attach(Pins::SERVO.no);
		}
		break;

	case C::MD_NOMODE:
	case C::MD_STARTUP:
	default:
		if (chk_bits(M.sw, C::SW_SE)) {
			// Go into installation configuration mode if select switch is pressed.
			M.md = C::MD_CONFIG_IS;
		} else if (chk_bits(M.sw, C::SW_NE)) {
			// Gear in neutral. Reset servo and change to idle mode.
			M.md = C::MD_IDLE;

			// Attach and reset servo position
			M.servo.attach(Pins::SERVO.no);
			M.servo.reset();
		}
		break;

	}

	// Update lcd background if mode changes
	if (M.md != last_mode) {
		switch (M.md) {
		case C::MD_IDLE:
			lcd.print(LCDStrings::MSG_IDLE);
			break;

		case C::MD_TOWING:
			lcd.print(LCDStrings::MSG_TOWING);
			break;

		case C::MD_CONFIG_OS:
		case C::MD_CONFIG_IS:
			lcd.print(LCDStrings::MSG_CONFIG);
			break;

		case C::MD_STARTUP:
		case C::MD_NOMODE:
		default:
			break;
		}

		// Delay to prevent messed up lcd display.
		delay(5);

	}

}

/**
 * This is the second step when in idle mode. Read set, up, down and is
 * (installation settings) switches.
 */
void idle_mode() {
	using namespace P;

	if (chk_bits(M.sw, C::SW_UP)) {
		// Increase throttle
		M.servo.pos += min(255 - M.servo.pos, MAN_THROTTLE_STEP);
		M.servo.set();
	} else if (chk_bits(M.sw, C::SW_DN)) {
		// Decrease throttle
		M.servo.pos -= min(M.servo.pos, MAN_THROTTLE_STEP);
		M.servo.set();
	}

	// Prepare text (sprintfbuffer) to update displayed lcd values.
	snprintf(sprintfbuffer, sizeof(sprintfbuffer), //
			LCDStrings::VAL_FORMAT, // Format string
			get_err_str(), // Error string
			params[I_OIL_HI].get_map(M.sensors.temp), // Oil temperature
			M.servo.pos * 100 / 255); // Servo position

	// Update lcd.
	lcd.print(sprintfbuffer);

	// Schedule next sample
	M.mark_time = M.time + T_SAMPLE;
}

/**
 * This is the second step when in tow mode. Calculates and controls the servo
 * position based on tachometer values. Also check for drum overspeed, drum 
 * tachometer fault and pump tachometer fault.
 */
void tow_mode() {
	using namespace P;

	// Update tachometer zero-speed counters. Used for error detection.
	M.drum_err_cnt = (M.sensors.drum_speed_raw == 0) ? M.drum_err_cnt + 1 : 0;
	M.pump_err_cnt = (M.sensors.pump_speed_raw == 0) ? M.pump_err_cnt + 1 : 0;

	// Reset drum overspeed flag...
	set_bits(false, M.err, C::ERR_DRUM_MAX);

	// Check over speeds and tachometer error conditions.
	if (M.sensors.drum_speed > params[I_DRUM_SPD].val) {
		// Drum overspeed!
		set_bits(true, M.err, C::ERR_DRUM_MAX);

		// Reduce throttle to limit drum speed. Don't update PID-controller.
		M.servo.pos -= min(M.servo.pos, 10);

	} else if ((M.drum_err_cnt > TACH_DRUM_ERR_COUNT)
			&& (M.servo.pos > TACH_DRUM_ERR_SERVO_TRESHOLD)) {
		// Drum tachometer zero-speed fault condition. Can be reset only by
		// re-enter tow mode.
		set_bits(true, M.err, C::ERR_DRUM_SENSOR);

		// Release throttle.
		M.servo.pos = 0;

	} else if (M.pump_err_cnt > TACH_PUMP_ERR_COUNT) {
		// Pump tachometer fault condition. Can be reset only by re-enter tow
		// mode.
		set_bits(true, M.err, C::ERR_PUMP_SENSOR);

		// Release throttle.
		M.servo.pos = 0;

	} else if (chk_bits(M.err, C::ERR_TEMP_HIGH)) {
		// Oil temperature too high. Reduce pump speed to decrease further heat
		// generation.
		M.pid.setpoint = (byte) params[I_PUMP_RPM].low;
		M.servo.pos = M.pid.process(M.sensors.pump_speed, M.sensors.drum_speed);

	} else {
		// No errors. Update PID-controller. Set pump setpoint (which may have
		// been previously changed due to a fault condition).
		M.pid.setpoint = (byte) params[I_PUMP_RPM].val;
		M.servo.pos = M.pid.process(M.sensors.pump_speed, M.sensors.drum_speed);

	}

	// Update servo position.
	M.servo.set();

	// Prepare text (sprintfbuffer) to update displayed lcd values.
	snprintf(sprintfbuffer, sizeof(sprintfbuffer), //
			LCDStrings::VAL_FORMAT, // Format string
			get_err_str(), // Error string
			params[I_PUMP_RPM].get_map(M.sensors.pump_speed), // Pump speed
			M.servo.pos * 100 / 255); // Oil temperature

	// Update lcd.
	lcd.print(sprintfbuffer);

	// Schedule next sample
	M.mark_time = M.time + T_SAMPLE;
}

/**
 * Configuration mode.
 */
void config_mode() {
	// Use namespace P for parameters.
	using namespace P;

	const byte buf_len = 3;
	byte rx_buf[buf_len]; // Serial read buffer. Bytes; 0:parameter index,
						  // 1+2:parameter value bytes (big endian).
	char rx_len; // Number of byte read by Serial.readBuffer().
	static byte i = 0; // Parameter array index.

	if (chk_bits(M.sw, C::SW_SE)) {
		// Select next parameter.
		if (M.md == C::MD_CONFIG_OS) {
			// In operation settings mode
			if (i >= OPER_PARAM_END) {
				i = 0;
			} else {
				i++;
			}

		} else {
			// In installation settings mode
			if (i >= INST_PARAM_END || i <= OPER_PARAM_END) {
				i = OPER_PARAM_END + 1;
			} else {
				i++;
			}

		}

	} else if (chk_bits(M.sw, C::SW_UP)) {
		// Increase parameter value.
		params[i].increase();

	} else if (chk_bits(M.sw, C::SW_DN)) {
		// Decrease parameter value.
		params[i].decrease();

	} else if (chk_bits(M.sw, C::SW_SP)) {
		// Using bluetooth connection; select or select and set parameter.
		rx_len = Serial.readBytes((char*) rx_buf, buf_len);
		if (rx_len > 0 && rx_buf[0] <= INST_PARAM_END) {
			// Parameter index received.
			i = rx_buf[0];
			if (rx_len == buf_len) {
				// Set parameter value received. Update parameter.
				params[i].set((int) rx_buf[1] << 8 | (int) rx_buf[2]);
			}

		} else if (i < INST_PARAM_END) {
			// No index specified. Select next.
			i++;

		} else {
			// No index specified. Select next (wrap to first).
			i = 0;

		}

	}

	// Update lcd, transmit parameter to serial and postpone timeout if anything
	// changes.
	if (chk_bits(M.sw, C::SW_SE | C::SW_UP | C::SW_DN | C::SW_SP)) {
		// Update values on the lcd.
		snprintf(sprintfbuffer, sizeof(sprintfbuffer), LCDStrings::PRM_FORMAT,
				i + 1, params[i].descr, params[i].get_map(params[i].val));
		lcd.print(sprintfbuffer);

		// Transmit parameter through serial.
		params[i].transmit(M.md, i);

		// Wait until buttons are all released.
		while (Pins::SW_SE.read() == LOW || Pins::SW_UP.read() == LOW
				|| Pins::SW_DN.read() == LOW) {
		}

		// Mark time when changing parameter. Will pause 100ms, then timeout
		// after CONF_TIMEOUT ms.  
		M.mark_time = millis() + 100;
	}

}

/**
 * Last step independent of mode.
 */
inline void finish() {
	using namespace P;

	// Transmit sample to Serial if requested.
	if (chk_bits(M.sw, C::SW_GT)) {
		M.sensors.transmit(M.md, M.time);
	}

	// Set alive pin low while waiting.
	Pins::ALIVE_LED.low();
}

/**
 * Checks error bits and returns corresponding error string.
 *
 * @return char* Any of the LCDStrings::ERR_* strings.
 */
const char* get_err_str() {

	if (chk_bits(M.err, C::ERR_PUMP_SENSOR)) {
		return LCDStrings::ERR_TACH0;

	} else if (chk_bits(M.err, C::ERR_DRUM_SENSOR)) {
		return LCDStrings::ERR_TACH1;

	} else if (chk_bits(M.err, C::ERR_DRUM_MAX)) {
		return LCDStrings::ERR_DRUM_MAX;

	} else if (chk_bits(M.err, C::ERR_TWI)) {
		return LCDStrings::ERR_TWI;

	} else if (chk_bits(M.err, C::ERR_TEMP_HIGH)) {
		return LCDStrings::ERR_TEMP_HIGH;

	} else if (chk_bits(M.err, C::ERR_TEMP_LOW)) {
		return LCDStrings::ERR_TEMP_LOW;

	}

	return LCDStrings::ERR_NO_ERR;
}

/**
 * Configuration mode. 
 */
/**
 * Sets or clears bits in bitfield to bits in bitmask. If set is false, bits
 * are unset otherwise they are set.
 * 
 * @param set Flag true to set bits. Unset otherwise.
 * @param bitfield Byte to set or unset bits in.
 * @param bitmask Byte with bits to set or unset.
 */
void set_bits(boolean set, byte& bitfield, byte bitmask) {
	if (set) {
		bitfield |= bitmask;
	} else {
		bitfield &= ~bitmask;
	}
}

/**
 * Returns true if at least one bit in bitfield is active in in bitmask. 
 * 
 * @param byte Bits to look for in bitmask.
 * @param byte Bits to compare bitfield with.
 * @return boolean True if any bit is set.
 */
boolean chk_bits(byte& bitfield, byte bitmask) {
	return (bitfield & bitmask);
}

/**
 * Interupt function for interupt 0 (digital pin 2)
 */
void pump_tic() {
	M.sensors._pump_cnt++;
}

/**
 * Interupt function for interupt 1 (digital pin 3)
 */
void drum_tic() {
	M.sensors._drum_cnt++;
}
