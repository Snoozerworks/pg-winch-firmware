#ifndef I2C_LCD_H
#define	I2C_LCD_H

#include "Arduino.h"
#include <I2C.h>

/**
 * 
 * LCD03 characters      å   ä   ö   Å   Ä   Ö
 * LCD03 HEX code     \x80\xE1\xEF\x81\x82\x83";
 */

class I2C_LCD {
private:
  uint8_t addr;

public:

  I2C_LCD(uint8_t address) {
    addr = address;
  }

  void print(const char* data) {
    I2c.write(addr, (uint8_t) 0, (uint8_t*) data, strlen(data));
  }

  void loadCustomChars() {
    uint8_t ch[10];

    // å - HEX 0x80
    ch[0] = 27;
    ch[1] = 128;
    ch[2] = B11100100;
    ch[3] = B11100000;
    ch[4] = B11101110;
    ch[5] = B11100001;
    ch[6] = B11101111;
    ch[7] = B11110001;
    ch[8] = B11101111;
    ch[9] = B11100000;
    I2c.write(addr, (uint8_t) 0, ch, 10);

    // Å - HEX 0x81
    ch[0] = 27;
    ch[1] = 129;
    ch[2] = B11100100;
    ch[3] = B11100000;
    ch[4] = B11101110;
    ch[5] = B11110001;
    ch[6] = B11111111;
    ch[7] = B11110001;
    ch[8] = B11110001;
    ch[9] = B11100000;
    I2c.write(addr, (uint8_t) 0, ch, 10);

    // Ä - HEX 0x82
    ch[0] = 27;
    ch[1] = 130;
    ch[2] = B11110001;
    ch[3] = B11101110;
    ch[4] = B11110001;
    ch[5] = B11110001;
    ch[6] = B11111111;
    ch[7] = B11110001;
    ch[8] = B11110001;
    ch[9] = B11100000;
    I2c.write(addr, (uint8_t) 0, ch, 10);

    // Ö - HEX 0x83
    ch[0] = 27;
    ch[1] = 131;
    ch[2] = B11110001;
    ch[3] = B11101110;
    ch[4] = B11110001;
    ch[5] = B11110001;
    ch[6] = B11110001;
    ch[7] = B11110001;
    ch[8] = B11101110;
    ch[9] = B11100000;
    I2c.write(addr, (uint8_t) 0, ch, 10);

    //    // first matrix-column
    //    ch[0] = 27;
    //    ch[1] = 132;
    //    ch[2] = B11110000;
    //    ch[3] = B11110000;
    //    ch[4] = B11110000;
    //    ch[5] = B11110000;
    //    ch[6] = B11110000;
    //    ch[7] = B11110000;
    //    ch[8] = B11110000;
    //    ch[9] = B11110000;
    //    write(ch, 10);
    //
    //    // first, second matrix-column
    //    ch[0] = 27;
    //    ch[1] = 133;
    //    ch[2] = B11111000;
    //    ch[3] = B11111000;
    //    ch[4] = B11111000;
    //    ch[5] = B11111000;
    //    ch[6] = B11111000;
    //    ch[7] = B11111000;
    //    ch[8] = B11111000;
    //    ch[9] = B11111000;
    //    write(ch, 10);
    //
    //    // first, second, third matrix-column
    //    ch[0] = 27;
    //    ch[1] = 134;
    //    ch[2] = B11111100;
    //    ch[3] = B11111100;
    //    ch[4] = B11111100;
    //    ch[5] = B11111100;
    //    ch[6] = B11111100;
    //    ch[7] = B11111100;
    //    ch[8] = B11111100;
    //    ch[9] = B11111100;
    //    write(ch, 10);
    //
    //    // first, second, third, forth matrix-column
    //    ch[0] = 27;
    //    ch[1] = 135;
    //    ch[2] = B11111110;
    //    ch[3] = B11111110;
    //    ch[4] = B11111110;
    //    ch[5] = B11111110;
    //    ch[6] = B11111110;
    //    ch[7] = B11111110;
    //    ch[8] = B11111110;
    //    ch[9] = B11111110;
    //    write(ch, 10);
  }

};



#endif	/* I2C_LCD_H */

