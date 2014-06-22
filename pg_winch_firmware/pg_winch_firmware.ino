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
 * 3. The RN42 bluetooth module shall be configure with profile 4 "SPP and DUN-DCE"
 * or possibly with profile 1 "DUN-DCE" to make the arduino reset work.
 * This is done by sending the module first "$$$" to get in command mode. The module
 * shall respond with CMD. Then send "S~,4<CR>". To verify the setting send "O<CR>"
 * (O as in Oliver) and module will repond with a list of settings. 
 * Don't include the "" when sending the commands.
 * 
 */

/* This is for debugging only */
//int freeRam() {
//  extern int __heap_start, *__brkval;
//  int v;
//  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//}
// Machine states, servo object and controller object.
struct MachineState {
	SensorsState sensors; // Sensor states (sampled values)
	byte err; // Error states (active/inactive)
	byte md; // Mode of operation
	byte sw; // Switch states  (on/off)

	ThrottleServo servo; // Servo object
	Controller pid; // PID controller object

	byte pump_err_cnt;
	byte drum_err_cnt;

	unsigned long time;
	unsigned long mark_time;
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

	// Initilize states
	M.sw = 0;
	M.err = 0;
	M.pump_err_cnt = 0;
	M.drum_err_cnt = 0;
	M.md = C::MD_STARTUP; // Flag for startup

	// Start serial for RN-42 bluetooth module.
	Serial.begin(115200);
	Serial.setTimeout(BUS_TIMEOUT);

	// Start I2C 
	I2c.begin();
	I2c.timeOut(BUS_TIMEOUT);

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

	// Flag startup mode.
	M.md = C::MD_STARTUP;

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
 * Reads digital inputs, check serial for commands and updates machine state accordingly.   
 * Does not read tachometers.
 */
inline void read_switches() {
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
	set_bits(Pins::NEUTRAL_SW.read() == LOW, M.sw, C::SW_NE);
	set_bits(Pins::CONFIG_ST_SW.read() == LOW || rx_cmd == C::CM_SET, M.sw,
			C::SW_ST);
	set_bits(Pins::CONFIG_UP_SW.read() == LOW || rx_cmd == C::CM_UP, M.sw,
			C::SW_UP);
	set_bits(Pins::CONFIG_DN_SW.read() == LOW || rx_cmd == C::CM_DOWN, M.sw,
			C::SW_DN);
	set_bits(rx_cmd == C::CM_CONF, M.sw, C::SW_IS);
	set_bits(rx_cmd == C::CM_SETP, M.sw, C::SW_SP);
	set_bits(rx_cmd == C::CM_GET, M.sw, C::SW_GT);

	// Read pressure
	M.sensors.pres = Pins::PRESSURE.read();

	// Check I2C bus error
	set_bits(0 != I2c.read(I2C_TMP_ADDR, (uint8_t) 2), M.err, C::ERR_TWI);

	// Read temperature
	M.sensors.temp = (I2c.receive() << 8 | I2c.receive()) >> 7;

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
inline void mode_select() {
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
			if (chk_bits(M.sw, C::SW_ST | C::SW_IS)) {
				// Set switch or Installation settings (virtual) switch active.
				// Change to installation configuration mode.
				M.md = (chk_bits(M.sw, C::SW_ST)) ?
						C::MD_CONFIG_OS : C::MD_CONFIG_IS;

				// Reset and detach servo.
				M.servo.reset();
				M.servo.detach();
			}

		} else {
			// Gear engaged.
			if (chk_bits(M.err, C::ERR_TEMP_HIGH)) {
				// Oil temperature limit is exceeded. Stay in idle mode.
				M.servo.reset();

			} else {
				// Change to tow mode.
				M.md = C::MD_TOWING;

				// Reset tachometer errors each time tow mode is entered.
				set_bits(false, M.err, C::ERR_DRUM_SENSOR | C::ERR_PUMP_SENSOR);

				// Reset PID-controller to avoid servo glitches.
				M.pid.reset();

				// Wait for automatic transmission to engage.
				delay(GEAR_ENGAGE_DELAY);
			}

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
		if (chk_bits(M.sw, C::SW_NE)) {
			// Gear in neutral. Reset servo and change to idle mode.
			M.md = C::MD_IDLE;

			// Attach and reset servo position
			M.servo.attach(Pins::SERVO.no);
			M.servo.reset();
		} else if (chk_bits(M.sw, C::SW_ST)) {
			// Go into installation configuration mode if set switch is pressed.
			M.md = C::MD_CONFIG_IS;
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
inline void idle_mode() {
	if (chk_bits(M.sw, C::SW_UP)) {
		// Increase throttle
		M.servo.pos += min(255 - M.servo.pos, MAN_THROTTLE_STEP);
		M.servo.set();
	} else if (chk_bits(M.sw, C::SW_DN)) {
		// Decrease throttle
		M.servo.pos -= min(M.servo.pos, MAN_THROTTLE_STEP);
		M.servo.set();
	}

	// Schedule next sample
	M.mark_time = M.time + T_SAMPLE;
}

/**
 * This is the second step when in tow mode. Calculates and controls the servo
 * position based on tachometer values. Also check for drum overspeed, drum 
 * tachometer fault and pump tachometer fault.
 */
inline void tow_mode() {
	using namespace P;

	// Update tachometer zero-speed counters. Used for error detection.
	M.pump_err_cnt = (M.sensors.pump_speed_raw == 0) ? M.pump_err_cnt + 1 : 0;
	M.drum_err_cnt = (M.sensors.drum_speed_raw == 0) ? M.drum_err_cnt + 1 : 0;

	// Reset drum overspeed flag...
	set_bits(false, M.err, C::ERR_DRUM_MAX);

	// Check over speeds and tachometer error conditions.
	if (M.sensors.drum_speed > params[I_DRUM_SPD].val) {
		// Drum overspeed!
		set_bits(true, M.err, C::ERR_DRUM_MAX);

		// Reduce throttle to limit drum speed. Don't update PID-controller.
		M.servo.pos -= min(M.servo.pos, 10);

	} else if ((M.drum_err_cnt > TACH1_ERR_COUNT)
			&& (M.servo.pos > TACH1_ERR_SERVO_TRESHOLD)) {
		// Drum tachometer zero-speed fault condition. Can be reset only by re-enter tow mode.
		set_bits(true, M.err, C::ERR_DRUM_SENSOR);

		// Release throttle.
		M.servo.pos = 0;

	} else if (M.pump_err_cnt > TACH0_ERR_COUNT) {
		// Pump tachometer fault condition. Can be reset only by re-enter tow mode.
		set_bits(true, M.err, C::ERR_PUMP_SENSOR);

		// Release throttle.
		M.servo.pos = 0;

	} else if (chk_bits(M.err, C::ERR_TEMP_HIGH)) {
		// Oil temperature too high. Reduce pump speed to decrease further heat generation.
		M.pid.setpoint = (byte) params[I_PUMP_RPM].low;
		M.servo.pos = M.pid.process(M.sensors.pump_speed, M.sensors.drum_speed);

	} else {
		// No errors. Update PID-controller. Set pump setpoint (which may have been previously
		// changed due to a fault condition).
		M.pid.setpoint = (byte) params[I_PUMP_RPM].val;
		M.servo.pos = M.pid.process(M.sensors.pump_speed, M.sensors.drum_speed);

	}

	// Update servo position.
	M.servo.set();

	// Schedule next sample
	M.mark_time = M.time + T_SAMPLE;
}

/**
 * This is the third and last step. Read pressure, temperature and serial data and update display.
 */
inline void finish() {
	using namespace P;
	//static byte last_mode; // Last mode of operation
	//byte rx_cmd; // Serial command byte
	const char prn_format[] = "\x02\x33%4.0d" "\x02\x48%3.0d"; // Print format string

	// Transmit sample to Serial if requested.
	if (chk_bits(M.sw, C::SW_GT)) {
		M.sensors.transmit(M.md, M.time);
	}

	// Prepare text (sprintfbuffer) to update displayed lcd values.
	switch (M.md) {
	case C::MD_CONFIG_IS:
	case C::MD_CONFIG_OS:
		// TODO
		break;

	case C::MD_IDLE:
		snprintf(sprintfbuffer, sizeof(sprintfbuffer), prn_format,
				params[I_OIL_HI].get_map(M.sensors.temp),
				M.servo.pos * 100 / 255);
		break;

	case C::MD_TOWING:
		snprintf(sprintfbuffer, sizeof(sprintfbuffer), prn_format,
				params[I_PUMP_RPM].get_map(M.sensors.pump_speed),
				M.servo.pos * 100 / 255);
		break;

	case C::MD_STARTUP:
	case C::MD_NOMODE:
	default:
		sprintfbuffer[0] = 0; // Print null character
		break;
	}

	lcd.print(sprintfbuffer);

	// Update lcd. Errors are displayed in a priority.
	lcd.print("\x02\x15"); // Move to row 2
	if (chk_bits(M.err, C::ERR_PUMP_SENSOR)) {
		lcd.print(LCDStrings::ERR_TACH0);

	} else if (chk_bits(M.err, C::ERR_DRUM_SENSOR)) {
		lcd.print(LCDStrings::ERR_TACH1);

	} else if (chk_bits(M.err, C::ERR_DRUM_MAX)) {
		lcd.print(LCDStrings::ERR_DRUM_MAX);

	} else if (chk_bits(M.err, C::ERR_TWI)) {
		lcd.print(LCDStrings::ERR_TWI);

	} else if (chk_bits(M.err, C::ERR_TEMP_HIGH)) {
		lcd.print(LCDStrings::ERR_TEMP_HIGH);

	} else if (chk_bits(M.err, C::ERR_TEMP_LOW)) {
		lcd.print(LCDStrings::ERR_TEMP_LOW);

	} else {
		lcd.print(LCDStrings::ERR_NO_ERR);

	}

	// Set alive pin low while waiting.
	Pins::ALIVE_LED.low();
}

/**
 * Configuration mode. 
 */
void config_mode() {
	using namespace P;
	// This is the parameter namespace.

	byte rx_buf[3]; // Serial read buffer. Bytes; 0:parameter index, 1+2:parameter value bytes (big endian),
	char rx_len; // Number of byte read by Serial.readBuffer(). 
	static byte i = 0; // Parameter array index.

	if (chk_bits(M.sw, C::SW_ST)) {
		// Select next parameter.
		if (M.md == C::MD_CONFIG_OS && i >= OPER_PARAM_END) {
			i = 0;
		} else if (M.md == C::MD_CONFIG_IS && i >= INST_PARAM_END) {
			i = OPER_PARAM_END + 1;
		} else {
			i++;
		}

	} else if (chk_bits(M.sw, C::SW_UP)) {
		// Increase parameter value.
		params[i].increase();

	} else if (chk_bits(M.sw, C::SW_DN)) {
		// Decrease parameter value.
		params[i].decrease();

	} else if (chk_bits(M.sw, C::SW_SP)) {
		// Wait for serial data
		delay(50);

		// Recieve serial data. Reset command byte (first byte in buffer) to 
		// C::CM_NOCMD if zero bytes received or if bytes are invalid.
		rx_len = Serial.readBytes((char*) rx_buf, 3);
		if (rx_len == 3 && rx_buf[0] <= INST_PARAM_END) {
			// Set parameter value from serial data.
			params[rx_buf[0]].set((int) rx_buf[1] << 8 | (int) rx_buf[2]);
		}

	}

	// Update lcd, transmit parameter to serial and postpone timeout if anything 
	// changes.
	if (chk_bits(M.sw, C::SW_ST | C::SW_UP | C::SW_DN | C::SW_SP)) {
		// Update values on the lcd.
		snprintf(sprintfbuffer, sizeof(sprintfbuffer), "\x02\x12" "%2d)" // Parameter index at lcd pos 19
						"%-20s"// Description on row 2
						"\x02\x3D" "%6d",// Value at lcd pos 61 
		i + 1, params[i].descr, params[i].get_map(params[i].val));
		lcd.print(sprintfbuffer);

		// Transmit parameter through serial.
		params[i].transmit(M.md, i);

		// Wait until buttons are all released.
		while (Pins::CONFIG_ST_SW.read() == LOW
				|| Pins::CONFIG_UP_SW.read() == LOW
				|| Pins::CONFIG_DN_SW.read() == LOW) {
		}

		// Delay to avoid contact bounce
		delay(10);

		// Mark time when changing parameter.
		M.mark_time = millis();

		// Postpone timeout
		//exit_time = millis() + CONF_TIMEOUT;
	}

}

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
