#include <Arduino.h>
#include <MIDI.h>

#include <I2c.h>

#include "rgb_lempos.h"

#undef SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);


using color=RGBLempos::color;

RGBLempos lamps(19, 21, 20);

void setup() {
#ifdef SERIAL_DEBUG
	Serial.begin (9600);
#endif
	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::green);
	}
	lamps.illuminate();

	midiA.turnThruOn();
	midiA.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
	if (midiA.read()) {
#ifdef SERIAL_DEBUG
		Serial.print(midiA.getType(), HEX);Serial.print(' ');
		Serial.print(midiA.getData1(), HEX);Serial.print(' ');
		Serial.print(midiA.getData2(), HEX);Serial.print(' ');
		Serial.println(midiA.getChannel(), HEX);
#endif
	} else {
//		Serial.println("nope");
	}
}