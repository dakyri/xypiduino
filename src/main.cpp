#include <Arduino.h>
#include <MIDI.h>

#include <I2c.h>
#include <PCF8574mw.h>
#include <PCF8591mw.h>
#include <common.h>

#include "rgb_lempos.h"

#undef SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1

#ifdef SERIAL_DEBUG
#include <Wire.h>
#endif

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);

using color=RGBLempos::color;

constexpr uint8_t kButtonStateAdr = 0x38;
constexpr uint8_t kAdcEnableAdr = 0x39;
constexpr uint8_t kAdcAdr = 0x48;
constexpr uint8_t debounceDelay_ms = 50;
constexpr uint8_t buttonVelocity = 80;
constexpr uint8_t pedalChangeMinTime_ms = 50;


RGBLempos lamps(19, 21, 20);
PCF8574<I2c> buttonStates(kButtonStateAdr);
PCF8574<I2c> adcEnable(kAdcEnableAdr);
PCF8591<I2c> adc(kAdcAdr);

/*!
 * wraps an common anode rgb led on the given bit of an 8574
 */
class Lamp8574 {
public:
	Lamp8574(PCF8574<I2c> &d, uint8_t b): device(d), atbit(b), baseColor(Lamp8574::color::green), unblink_ms(0) {}
	enum color {
		black = 0b111,
		red = 0b011,
		green = 0b110,
		blue = 0b101,
		white = 0b000
	};

	void setColor(uint8_t c, uint32_t blink_t=0) {
		if (unblink_ms == 0) {
			device.writeOutputs(c << atbit);
		}
		if (blink_t > 0) {
			unblink_ms = millis() + blink_t;
		}
	}

	void checkBlink() {
		if (millis() > unblink_ms) {
			unblink_ms = 0;
			setColor(baseColor);
		}
	}

protected:
	PCF8574<I2c> & device;
	const uint8_t atbit;
	const uint8_t baseColor;
	unsigned long unblink_ms;
};

Lamp8574 statusLamp(adcEnable, 4);

/*!
 * mappings for button states
 */
struct button {
	enum class mode_t: uint8_t {
		note = 0,
		prog = 1,
		ctrl = 2,
		ctrlLatch = 3,
		ctrlLatchOn = 4
	};
	button(const mode_t mode, const uint8_t val, const uint8_t channel,
		const color lampColor, const color activeColor)
	: lastBounceTime_ms(0), mode(mode), lastState(false), pressed(false),
	  value(val), midiChannel(channel), lampColor(lampColor), activeColor(activeColor) {}

	long lastBounceTime_ms;
	mode_t mode;
	bool lastState;
	bool pressed;
	uint8_t value;
	uint8_t midiChannel;
	color lampColor;
	color activeColor;
};

struct button buttons[] = {
	{ button::mode_t::ctrl, 32, 0, color::green, color::red},
	{ button::mode_t::note, 33, 0, color::green, color::red},
	{ button::mode_t::ctrlLatch, 34, 0, color::green, color::blue},
	{ button::mode_t::prog, 35, 0, color::green, color::red},
	{ button::mode_t::ctrl, 36, 0, color::green, color::red},
	{ button::mode_t::ctrl, 37, 0, color::green, color::red},
	{ button::mode_t::ctrl, 38, 0, color::green, color::red},
	{ button::mode_t::ctrl, 39, 0, color::green, color::red},
};
constexpr uint8_t nButtons = sizeof(buttons)/sizeof(button);

/*!
 * mappings for pedals
 */
struct pedal {
	pedal(byte _ctrl, byte _midiChannel)
	: lastChangeTime_ms(0), ctrl(_ctrl), midiChannel(_midiChannel), lastVal(255), lastVal2(255) {}

	long lastChangeTime_ms;
	byte ctrl;
	uint8_t midiChannel;

	byte lastVal;
	byte lastVal2;
};

struct pedal pedals[] = {
	{ 85, 0 },
	{ 86, 0 },
	{ 87, 0 }
};
constexpr uint8_t nPedals = sizeof(pedals)/sizeof(pedal);

/*!
 *
 */
void checkPanelButton(const uint8_t which, const bool pressed) {

	long theTime = millis();
	
	auto & b{buttons[which]};

	if (pressed != b.lastState) {
		b.lastState = pressed;
		b.lastBounceTime_ms = theTime; //set the current time
	}
	if (theTime > debounceDelay_ms + b.lastBounceTime_ms) {
		if (pressed && !b.pressed) {
			Serial.print(which); Serial.println(" pressed");
			b.pressed = true;
			if (b.mode == button::mode_t::ctrl) {
				midiA.sendControlChange(b.value, 127, b.midiChannel);
			} else if (b.mode == button::mode_t::prog){
				midiA.sendProgramChange(b.value, b.midiChannel);
			} else if (b.mode == button::mode_t::note){
				midiA.sendNoteOn(b.value, buttonVelocity, b.midiChannel);
			} else if (b.mode == button::mode_t::ctrlLatch){
				b.mode = button::mode_t::ctrlLatchOn;
				midiA.sendControlChange(b.value, 127, b.midiChannel);
			} else if (b.mode == button::mode_t::ctrlLatchOn){
				b.mode = button::mode_t::ctrlLatch;
				midiA.sendControlChange(b.value, 0, b.midiChannel);
			}
			lamps.setLamp(which,
				b.mode == button::mode_t::ctrlLatch? b.lampColor: b.activeColor);

		} else if (!pressed && b.pressed ) {
			b.pressed = false;
			if (b.mode == button::mode_t::ctrl) {
				midiA.sendControlChange(b.value, 0, b.midiChannel);
			} else if (b.mode == button::mode_t::note){
				midiA.sendNoteOff(b.value, buttonVelocity, b.midiChannel); 
			}
			switch (b.mode) {
			case button::mode_t::ctrl:
			case button::mode_t::note:			
			case button::mode_t::prog:
				lamps.setLamp(which, b.lampColor);
				break;
			default:
				break;
			}

		}
	}
}

void checkPedal(const uint8_t which, const uint8_t reading) {
	uint8_t ctrlVal = reading / 2;
	auto &ped{pedals[which]};
	int8_t chg = ctrlVal - ped.lastVal;

	if (chg != 0) {
		long theTime = millis();
		int8_t chg2 = ctrlVal - ped.lastVal2;
		long potChangeT_ms = theTime - ped.lastChangeTime_ms;
		if (potChangeT_ms > pedalChangeMinTime_ms) {
			if (ped.lastVal2 == 255
							|| (ctrlVal <= 64 && (chg < 0 || chg2 > 3))
							|| (ctrlVal > 64 && (chg > 0 || chg2 < -3))) {
				ped.lastVal2 = ped.lastVal;
				ped.lastVal = ctrlVal;
				ped.lastChangeTime_ms = theTime;
				midiA.sendControlChange(ped.ctrl, ctrlVal, ped.midiChannel);
				/*
				if (potChangeT_ms > 200) {
					lamp.blink(3, 50);
				}
				*/
			}
		}
	}
}

#ifdef SERIAL_DEBUG
#endif

void setup() {

#ifdef SERIAL_DEBUG
	Serial.begin (9600);
	delay(2000);
	Serial.println("Scanning...");
	uint8_t nDevices = 0;
	uint8_t ports[127];

	I2c::scan(ports, nDevices);
	Serial.println(nDevices == 0? "No I2C devices found\n" : "Found devices:\n");
	for (uint8_t i=0; i<nDevices; i++) {
		print_hex_byte(ports[i]);
	}

#endif
	adcEnable.setInputs(0x07);
	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::red);
	}
	lamps.illuminate();
	mw::hang(200);
	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::blue);
	}
	lamps.illuminate();
	mw::hang(200);
	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::green);
	}
	lamps.illuminate();
	mw::hang(200);
	if (buttonStates.begin()) {
		lamps.setLamp(0, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}
	if (adcEnable.begin()) {
		lamps.setLamp(1, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}
	if (adc.begin()) {
		lamps.setLamp(2, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}

	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::green);
	}
	lamps.illuminate();

	midiA.turnThruOn();
	midiA.begin(MIDI_CHANNEL_OMNI);
}
int m = 0;
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

	const uint8_t currentButtons = buttonStates.getInputs();
	uint8_t mask = 1;
	for (uint8_t i=0; i<nButtons; ++i) {
		const bool pressed = (currentButtons & mask) != 0;
		checkPanelButton(i, pressed);
		mask <<= 1;
	}
	lamps.illuminate();
	const uint8_t currentAdcEnables = (~adcEnable.getInputs()) & 0x7;
	mask = 1;
	for (uint8_t i=0; i<3; ++i) {
//		uint8_t value0=adc.adRead(i); print_hex_byte(value0, false);

		if (currentAdcEnables & mask) {
			auto val = adc.adRead(i);
			checkPedal(i, val);
		}
		mask <<= 1;
	}
//	print_hex_byte(currentAdcEnables);
}