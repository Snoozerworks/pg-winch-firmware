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
    //    if (no < 8) {
    //      return bitRead(PORTD, no);
    //    } else {
    //      return bitRead(PORTB, no);
    //    }
    return digitalRead(no);
  };

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
   * Initate pin i.e. set pin mode.
   * 
   * @param n Pin number.
   * @param m Pin mode.
   */
  AnaloguePin(const uint8_t n, uint8_t m) {
    this->no = n;
    pinMode(n, m);
  };

  /**
   * Read an analogue pin
   * @return int Analogue value.
   */
  inline int read() {
    return analogRead(no);
  };
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
  const byte CM_CONF = 1; // Request mode change to Installation Settings
  const byte CM_SET = 2; // Set switch active
  const byte CM_UP = 3; // Up switch active
  const byte CM_DOWN = 4; // Down switch active
  const byte CM_SETP = 5; // Set parameter (viritual) switch active
  const byte CM_GET = 6; // Get sample (viritual) switch active
  const byte CM__LAST = CM_GET; // Last element. Used internally only.

  // State of switches are saved in a structure.  
  const byte SW_NE = 1; // Neutral switch
  const byte SW_ST = 2; // Set switch
  const byte SW_UP = 4; // Up switch
  const byte SW_DN = 8; // Down switch
  const byte SW_SP = 16; // Set parameter (viritual) switch
  const byte SW_GT = 32; // Get sample (viritual) switch
  const byte SW_IS = 64; // Installation settings (viritual) switch

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
 * - low_map  : Map the lower bound of a paramater to a custom scale.
 * - high_map : Map the higher bound of a paramater to a custom scale.
 * 
 * Map values (low_map/high_map) are used to convert paramater value to a unit 
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
    //return map(v, low, high, low_map, high_map);
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

    Serial.write(tx_buffer, sizeof (tx_buffer));
    Serial.write((byte*) descr, sizeof (descr));
    Serial.flush();
  }


};

/**
 * Structure for keeping sensor state data.
 *
 * byte  0 : mode
 * byte  1 : timestamp() byte 1 (high byte)
 * byte  2 : timestamp() byte 2
 * byte  3 : timestamp() byte 3
 * byte  4 : timestamp() byte 4 (low byte)
 * byte  5 : tachometer 0 (pump)
 * byte  6 : tachometer 1 (diff)
 * byte  7 : temp sensor (high byte)
 * byte  8 : temp sensor (low byte)
 * byte  9 : pressure sensor (high byte)
 * byte 10 : pressure sensor (low byte)
 */
struct SensorsState {
  // Properties
  volatile byte _drum_cnt; // Used by interrupt handler 1
  volatile byte _pump_cnt; // Used by interrupt handler 0

  byte drum_speed_raw; // Drum speed (counts per sample x 2)
  byte pump_speed_raw; // Pump speed (counts per sample x 2)
  byte drum_speed; // LP filtered drum_speed_raw
  byte pump_speed; // LP filtered pump_speed_raw
  int temp; // Oil temperature
  int pres; // Pump pressure

  unsigned int _tach_drum_filt; // Drum low pass filter
  unsigned int _tach_pump_filt; // Pump low pass filter

  /**
   * Write sensor data to Serial. High byte are sent before low bytes
   * (big endian byte order). 
   * In total 11 bytes are sent.
   * 
   * @param mode
   * @param time
   */
  void transmit(byte mode, unsigned long time) {
    byte tx_buffer[11];
    tx_buffer[0] = mode;
    tx_buffer[1] = time >> 24;
    tx_buffer[2] = time >> 16;
    tx_buffer[3] = time >> 8;
    tx_buffer[4] = time;
    tx_buffer[5] = pump_speed_raw;
    tx_buffer[6] = drum_speed_raw;
    tx_buffer[7] = temp >> 8;
    tx_buffer[8] = temp;
    tx_buffer[9] = pres >> 8;
    tx_buffer[10] = pres;

    Serial.write(tx_buffer, sizeof (tx_buffer));
    Serial.flush();
  };

  /**
   * Updates pump and drum speeds.
   * 
   * Low pass filtered values are calculated too. A local parameter FILTER_SHIFT 
   * sets the low pass filter characteristics.
   * 
   * NOTE! The tachometer count is multiplied by 2 when doing speed 
   * calculations. This is to improve the resolution for the low pass filtered
   * values. 
   * 
   * For example, if a tachometer has counted [126, 127] the raw speed value is
   * even values like e.g. [252, 254]. The low pass filterd speed value may 
   * however also contain odd numbers [253, 253].
   * 
   * Also see http://www.edn.com/design/systems-design/4320010/A-simple-software-lowpass-filter-suits-embedded-system-applications
   * 
   * @return void
   */
  void tachometer_read() {
    const byte FILTER_SHIFT = 1; // Shift parameter. 1-3 should likely suffice.
    static byte drum_cnt_old; // Previous drum tachometer count sample
    static byte pump_cnt_old; // Previous pump tachometer count sample
    byte drum_cnt; // Drum tachometer count 
    byte pump_cnt; // Pump tachometer count

    // Read drum_cnt and pump_cnt only once!
    drum_cnt = _drum_cnt;
    pump_cnt = _pump_cnt;
    drum_speed_raw = (drum_cnt - drum_cnt_old)*2; // Note multiplication by 2!
    pump_speed_raw = (pump_cnt - pump_cnt_old)*2; // Note multiplication by 2!
    drum_cnt_old = drum_cnt;
    pump_cnt_old = pump_cnt;

    _tach_drum_filt = _tach_drum_filt - (_tach_drum_filt >> FILTER_SHIFT) + drum_speed_raw;
    drum_speed = _tach_drum_filt >> FILTER_SHIFT;

    _tach_pump_filt = _tach_pump_filt - (_tach_pump_filt >> FILTER_SHIFT) + pump_speed_raw;
    pump_speed = _tach_pump_filt >> FILTER_SHIFT;

  }

};

#endif
