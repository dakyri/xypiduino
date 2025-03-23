#include <Arduino.h>
#include <MIDI.h>
#include <I2c.h>

#include <PCF8574mw.h>
#include <PCF8591mw.h>
#include <common.h>

#include "rgb_lempos.h"

#define SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);

using color=RGBLempos::color;

constexpr uint8_t kButtonStateAdr = 0x38;
constexpr uint8_t kAdcEnableAdr = 0x39;
constexpr uint8_t kAdcAdr = 0x48;

constexpr uint8_t debounceDelay_ms = 10;
constexpr uint16_t longPressTime_ms = 500;

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
		blue = 0b011,
		red = 0b110,
		green = 0b101,
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
	enum mode_t: uint8_t {
		nieko = 0,
		prog = 1,
		ctrl = 2,
		chanpress = 3,
		keypress = 4,
		note = 5,
		start = 6,
		stop = 7,
		type = 0x07,
		latch = 0x10,
	};

	enum state_t: uint8_t {
		open = 0,
		pressed = 0x01,
		longPressed = 0x02,
		latched = 0x80,
		latching = 0x40 // latch state = latched|latching when first engaged, so the next button up will keep the latch
	};

	button(const uint8_t chan, const uint8_t md, const uint8_t v1, const uint8_t v2, const uint8_t lmd, const uint8_t lv1, const uint8_t lv2)
	: lastBounceTime_ms(0), mode(md), val1(v1), val2(v2), long_mode(lmd), long_val1(lv1), long_val2(lv2),
	  channel(chan), state(state_t::open), lastPressState(false) {}

	long lastBounceTime_ms;
	long pressedTime_ms;
	const uint8_t mode;
	const uint8_t val1;
	const uint8_t val2;
	const uint8_t long_mode;
	const uint8_t long_val1;
	const uint8_t long_val2;
	const uint8_t channel;
	uint8_t active_type;
	uint8_t active_val1;
	uint8_t state;
	bool lastPressState;
};

using btmode = button::mode_t;
using btst8 = button::state_t;

#define LATCH(X) ((button::mode_t)(X|btmode::latch))

struct button buttons[] = {
	{ 0, btmode::ctrl, 32, 127,			btmode::latch, 0, 0},
	{ 0, btmode::note, 33, 80,			btmode::latch, 0, 0},
	{ 0, LATCH(btmode::ctrl), 34, 80,	btmode::ctrl, 34, 120},
	{ 0, btmode::prog, 35, 33,			btmode::latch, 0, 0},
	{ 0, btmode::ctrl, 36, 120,			btmode::latch, 0, 0},
	{ 0, btmode::note, 37, 80,			LATCH(btmode::keypress), 37, 100},
	{ 0, btmode::start, 38, 120,		btmode::latch, 0, 0},
	{ 0, btmode::nieko, 39, 120,		btmode::latch, 0, 0},
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

constexpr uint8_t lmpBtOpenCol = color::green;
constexpr uint8_t lmpBtDownCol = color::red;
constexpr uint8_t lmpBtLatchCol = color::blue;
/*!
 *
 */

 void sendButtonOnMidi(uint8_t typ, uint8_t chan, uint8_t v1, uint8_t v2) {
	bool sent = false;
	switch (typ) {
		case btmode::prog:
			midiA.sendProgramChange(v1, chan); sent = true;
			break;
		case btmode::ctrl:
			midiA.sendControlChange(v1, v2, chan); sent = true;
			break;
		case btmode::keypress:
			midiA.sendAfterTouch(v1, v2, chan); sent = true;
			break;
		case btmode::chanpress:
			midiA.sendAfterTouch(v1, chan); sent = true;
			break;
		case btmode::note:
			midiA.sendNoteOn(v1, v2, chan); sent = true;
			break;
		case btmode::start:
			midiA.sendRealTime(midi::MidiType::Start); sent = true;
			break;
		case btmode::stop:
			midiA.sendRealTime(midi::MidiType::Stop); sent = true;
			break;
	}
	if (sent) {
		statusLamp.setColor(Lamp8574::red, 100);
	}
};

void sendButtonOffMidi(uint8_t typ, uint8_t chan, uint8_t v1, uint8_t v2) {
	bool sent = false;
	switch (typ) {
		case btmode::ctrl:
			midiA.sendControlChange(v1, 0, chan); sent = true;
			break;
		case btmode::note:
			midiA.sendNoteOff(v1, v2, chan); sent = true;
			break;
		case btmode::start:
			midiA.sendRealTime(midi::MidiType::Stop); sent = true;
			break;
		case btmode::stop:
			midiA.sendRealTime(midi::MidiType::Start); sent = true;
			break;
	}
	if (sent) {
		statusLamp.setColor(Lamp8574::red, 100);
	}
}

void checkPanelButton(const uint8_t which, const bool pressed) {

	long theTime = millis();
	
	auto & b{buttons[which]};

	if (pressed != b.lastPressState) {
		b.lastPressState = pressed;
		b.lastBounceTime_ms = theTime; //set the current time
	}
	if (theTime > debounceDelay_ms + b.lastBounceTime_ms) {
		if (pressed) {
			if (!(b.state & btst8::pressed)) {
				b.state |= btst8::pressed;
				b.pressedTime_ms = theTime;
				const auto btype = b.mode & btmode::type;
				b.active_type = btype;

				if (b.mode & btmode::latch){
					b.state |= (btst8::latched|btst8::latching);
				}
				sendButtonOnMidi(btype, b.channel, b.val1, b.val2);
				lamps.setLamp(which, (b.state & btst8::latched)? lmpBtLatchCol: lmpBtDownCol);
			} else if (!(b.state & btst8::longPressed) && b.pressedTime_ms + longPressTime_ms < theTime) {
				b.state |= btst8::longPressed;
				const auto btype = b.long_mode & btmode::type;
				if (b.long_mode & btmode::latch){
					b.state |= (btst8::latched|btst8::latching);
				}
				if (btype > b.active_type) {
					b.active_type = btype;
					b.active_val1 = b.long_val1;
				}
				sendButtonOnMidi(btype, b.channel, b.long_val1, b.long_val2);
				lamps.setLamp(which, (b.state & btst8::latched)? lmpBtLatchCol: lmpBtDownCol);
			}
		} else {
			if (b.state & btst8::pressed) {
				if (b.state & btst8::latching) {
					b.state &= ~btst8::latching;
				} else if (b.state & btst8::latched){
					b.state &= ~btst8::latched;
				}
				b.state &= ~btst8::pressed;	
				const auto btype = b.active_type & btmode::type;

				sendButtonOffMidi(btype, b.channel, b.val1, b.val2);
				lamps.setLamp(which, b.state & btst8::latched? lmpBtLatchCol: lmpBtOpenCol);
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
				statusLamp.setColor(Lamp8574::red, 100);
				/*
				if (potChangeT_ms > 200) {
					lamp.blink(3, 50);
				}
				*/
			}
		}
	}
}

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

	/* light show, just to check shit works */
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
	statusLamp.setColor(Lamp8574::blue);
	mw::hang(200);

	/* check for main i2c devices. first 8574 for top panel buttons */
	if (buttonStates.begin()) {
		lamps.setLamp(0, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}

	/* check 8574 for status lamp and pedal enables */
	if (adcEnable.begin()) {
		lamps.setLamp(1, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}

	/* check the 8591 adc for pedals */
	if (adc.begin()) {
		lamps.setLamp(2, color::blue);
		lamps.illuminate();
		mw::hang(200);
	}

	/* set main lamps to base color */
	for (byte i=0; i<8; ++i) {
		lamps.setLamp(i, color::green);
	}
	lamps.illuminate();
	statusLamp.setColor(Lamp8574::green);

	/* setup MIDI */
	midiA.turnThruOn();
	midiA.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
	statusLamp.checkBlink();
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