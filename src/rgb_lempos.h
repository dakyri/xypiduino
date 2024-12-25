#pragma once

#include <Arduino.h>

/*!
 * an array of rgb leds driven by a trio of shift registers, one for each color
 */
class RGBLempos
{
public:
	RGBLempos(const byte dataPin, const byte clockPin, const byte latchPin):
			dataPin(dataPin), clockPin(clockPin), latchPin(latchPin) {
		pinMode(latchPin, OUTPUT);
		pinMode(clockPin, OUTPUT);
		pinMode(dataPin, OUTPUT);
	}

	/*!
	 * sets the given lamp to the particular color.
	 * colors are 3bit quantities 1 bit each for R, G, and B
	 */
	void setLamp(const byte lamp_id, const byte color) {
		byte bm = 1;
		const byte lm = (1 << lamp_id);
		for (byte i=0; i<3; ++i) {
			if (color & bm) {
				nuState[i] |= lm;
			} else {
				nuState[i] &= ~lm;
			}
			bm <<= 1;
		}
	}

	/*!
	 * sets state to nuState and sends that to the shift regs
	 */
	void illuminate() {
		digitalWrite(latchPin, LOW);
		for (byte i=0; i<3; i++) {
			shiftOut(dataPin, clockPin, MSBFIRST, nuState[i]); // matbe that's LSBFIRST and/or reverse order?
			state[i] = nuState[i];
		}
		digitalWrite(latchPin, HIGH);
	}

	enum class color_idx: byte {
		R=0, G=1, B=2
	};

	enum color {
		black = 0,
		red = 0b100,
		green = 0b010,
		blue = 0b001,
		white = 0b111
	};

protected:
	const byte dataPin;
	const byte clockPin;
	const byte latchPin;

	byte state[3] = {0, 0, 0};
	byte nuState[3] = {0, 0, 0};
};