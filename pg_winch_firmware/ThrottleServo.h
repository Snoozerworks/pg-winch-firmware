#ifndef THROTTLESERVO_H
#define THROTTLESERVO_H

#include "Arduino.h"
#include "parameter.h"

//class ThrottleServo: public Servo {
class ThrottleServo: public Servo {
public:
	/**
	 * Servo position 0-255. A value of 0 means the short servo pulse is used and
	 * a value of 255 means the long servo pulse is used. Pulse lengths are set by
	 * parameters params[I_SERV_LO].val and params[I_SERV_HI].val.
	 */
	byte pos;

	/**
	 * Set servo to 0 position.
	 */
	void reset() {
		using namespace P;
		pos = 0;
		set();
		delay(params[I_SERV_RST].val);
	}

	/**
	 * Set position of servo 0-255
	 */
	void set() {
		using namespace P;
		int us;
		us =
				params[I_SERV_LO].val
						+ (int) pos
								* ((params[I_SERV_HI].val
										- params[I_SERV_LO].val) / 255);
		writeMicroseconds(us);
	}

};

#endif /* THROTTLESERVO_H */
