#include <Arduino.h>
#include <MIDI.h>
#include <I2c.h>
#include <SPI.h>

#include <PCF8574mw.h>
#include <PCF8591mw.h>
#include <ADXL345mw.h>
#include <common.h>

#include "rgb_lempos.h"
#include "xyspi.h"
#include "midi_ring_buffer.h"

#define SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);

using color=RGBLempos::color;

constexpr uint8_t kButtonStateAdr = 0x38;
constexpr uint8_t kAdcEnableAdr = 0x39;
constexpr uint8_t kAdcAdr = 0x48;

constexpr uint16_t xlm8rReadDelay_ms = 100;
constexpr uint8_t debounceDelay_ms = 10;
constexpr uint16_t longPressTime_ms = 500;

constexpr uint8_t pedalChangeMinTime_ms = 50;


RGBLempos lamps(19, 21, 20);
PCF8574<I2c> buttonStates(kButtonStateAdr);
PCF8574<I2c> adcEnable(kAdcEnableAdr);
PCF8591<I2c> adc(kAdcAdr);
adxl345::Interface<I2c> accelerometer(adxl345::kAddress1);


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
	enum state_t: uint8_t {
		open = 0,
		pressed = 0x01,
		longPressed = 0x02,
		latched = 0x80,
		latching = 0x40 // latch state = latched|latching when first engaged, so the next button up will keep the latch
	};

	button(config::button cfg)
		: lastBounceTime_ms(0), mode(cfg.mode), val1(cfg.val1), val2(cfg.val2),
		long_mode(cfg.long_mode), long_val1(cfg.long_val1), long_val2(cfg.long_val2),
		channel(cfg.chan), state(state_t::open), lastPressState(false) {}

	void set(config::button &cfg) {
		lastBounceTime_ms = 0;
		mode = cfg.mode;
		val1 = cfg.val1;
		val2 = cfg.val2;
		long_mode = cfg.long_mode;
		long_val1 = cfg.long_val1;
		long_val2 = cfg.long_val2;
		channel = cfg.chan;
		state = state_t::open;
		lastPressState = false;		
	}

	long lastBounceTime_ms;
	long pressedTime_ms;
	uint8_t mode;
	uint8_t val1;
	uint8_t val2;
	uint8_t long_mode;
	uint8_t long_val1;
	uint8_t long_val2;
	uint8_t channel;
	uint8_t active_type;
	uint8_t active_val1;
	uint8_t state;
	bool lastPressState;
};

using btmode = config::mode_t;
using btst8 = button::state_t;

#define LATCH(X) ((config::mode_t)(X|btmode::latch))

struct button buttons[] = {
	{{ 0, btmode::ctrl, 32, 127,		btmode::latch, 0, 0}},
	{{ 0, btmode::note, 33, 80,			btmode::latch, 0, 0}},
	{{ 0, LATCH(btmode::ctrl), 34, 80,	btmode::ctrl, 34, 120}},
	{{ 0, btmode::prog, 35, 33,			btmode::latch, 0, 0}},
	{{ 0, btmode::ctrl, 36, 120,		btmode::latch, 0, 0}},
	{{ 0, btmode::note, 37, 80,			LATCH(btmode::keypress), 37, 100}},
	{{ 0, btmode::start, 38, 120,		btmode::latch, 0, 0}},
	{{ 0, btmode::nieko, 39, 120,		btmode::latch, 0, 0}},
};
constexpr uint8_t nButtons = sizeof(buttons)/sizeof(button);

/*!
 * mappings for pedals
 */
struct pedal {
	pedal(config::pedal cfg)
	: lastChangeTime_ms(0), ctrl(cfg.which), midiChannel(cfg.chan), lastVal(255), lastVal2(255) {}

	void set(config::pedal &cfg) {
		lastChangeTime_ms = 0;
		lastVal = 255;
		lastVal2 = 255;
		ctrl = cfg.which;
		midiChannel = cfg.chan;
	}

	long lastChangeTime_ms;
	uint8_t ctrl;
	uint8_t midiChannel;

	uint8_t lastVal;
	uint8_t lastVal2;
};

struct pedal pedals[] = {
	{{ 0, btmode::ctrl, 85, 0, 127 }},
	{{ 0, btmode::ctrl, 86, 0, 127 }},
	{{ 0, btmode::ctrl, 87, 0, 127 }}
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
				if (potChangeT_ms > 200) {
					statusLamp.setColor(Lamp8574::red, 50);
				}
			}
		}
	}
}

/*****************************************************
 * XY CONTROL
 *****************************************************/
constexpr float kRadGrad = (180.0/M_PI);
constexpr float kRad = (1.0/M_PI);

struct axis_ctl {
	axis_ctl(uint8_t _mode, uint8_t _chan, uint8_t _ctrl, int _min, int _max, uint8_t _ctrlMin, uint8_t _ctrlMax) {
		set(_mode, _chan, _ctrl, _min, _max, _ctrlMin, _ctrlMax);
	}

	void set(uint8_t _mode, uint8_t _chan, uint8_t _ctrl, int _min, int _max, uint8_t _ctrlMin, uint8_t _ctrlMax) {
		mode = _mode;
		chan = _chan;
		ctrl = _ctrl;
		minV = _min;
		maxV = _max;
		ctrlMin = _ctrlMin;
		ctrlMax = _ctrlMax;
		lastCV = 255;
		factor = (ctrlMax != ctrlMin)? float(ctrlMax - ctrlMin)/float(maxV - minV) : 0;
	}

	void checkSend(int angle) {
		if (angle < minV) angle = minV;
		else if (angle > maxV) angle = maxV;
		uint8_t cv = ctrlMin + (angle - minV) * factor;
	 if (cv != lastCV) {
#ifdef SERIAL_DEBUG
			Serial.print("axus control "); Serial.print(ctrl); Serial.print(" angle "); Serial.print(angle);Serial.print(" minV "); Serial.print(minV);Serial.print(" maxV "); Serial.print(maxV);Serial.print(" factor "); Serial.print(factor);Serial.print(" cv "); Serial.println(cv);
#endif 
			midiA.sendControlChange(ctrl, cv, chan);
			statusLamp.setColor(Lamp8574::blue, 50);
			lastCV = cv;
		}
	}
	
	uint8_t mode;
	uint8_t chan;
	uint8_t ctrl;
	int minV; // assuming we range -180..180, and probably a smaller ambit than even 180
	int maxV; // > min
	uint8_t ctrlMin; // ctrl value at min 0..127
	uint8_t ctrlMax; // ctrl value at max . may be > or < than CtrlMin
	float factor;
	uint8_t lastCV;
 
};

struct xl_joy {
	xl_joy(config::xlrm8r cfg)
		: lastReading_ms(0), activeButton(cfg.enable_button),
		pitch(cfg.mode_x, cfg.chan, cfg.which_x, cfg.min_ambit_x, cfg.max_ambit_x, cfg.min_x_val, cfg.max_x_val),
		roll(cfg.mode_y, cfg.chan, cfg.which_y, cfg.min_ambit_y, cfg.max_ambit_y, cfg.min_y_val, cfg.max_y_val) {}

	void set(config::xlrm8r &cfg) {
		activeButton = cfg.enable_button;
		pitch.set(cfg.mode_x, cfg.chan, cfg.which_x, cfg.min_ambit_x, cfg.max_ambit_x, cfg.min_x_val, cfg.max_x_val),
		roll.set(cfg.mode_y, cfg.chan, cfg.which_y, cfg.min_ambit_y, cfg.max_ambit_y, cfg.min_y_val, cfg.max_y_val);
	}
	
	long lastReading_ms;
	uint8_t activeButton;

	struct axis_ctl pitch;
	struct axis_ctl roll;
};

struct xl_joy xlmr8rs[] = {
	{{0, 2, /* pitch */	btmode::ctrl, 83, -45, 45, 0, 127, /* roll */ btmode::ctrl, 84, -45, 45, 0, 127}}
};
constexpr uint8_t nXlmr8rs = sizeof(xlmr8rs)/sizeof(xl_joy);

void checkAccelerometer(adxl345::Interface<I2c> & xlm8r, struct xl_joy &xl) {
	if (xlm8r.isValid && (xl.activeButton == 255 || buttons[xl.activeButton].pressed)) {
		long theTime = millis();
		if (theTime - xl.lastReading_ms > xlm8rReadDelay_ms) {
			int16_t x, y, z;
			xlm8r.readRaw(x, y, z);
			adxl345::Vector v(x,y,z);
/* filtered = filtered.lowPassFilter(v, 0.5); // Low Pass Filter to smooth out data. 0.1 - 0.9 */
// convert to degrees: these are probably going to be simpler to work with if we need to configure on the fly through front panel
			int pitch = -atan2(v.x, sqrt(v.y*v.y + v.z*v.z))*kRadGrad;
			int roll	= atan2(v.y, v.z)*kRadGrad;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG_LEVEL > 2
			Serial.print("xyz ->");Serial.print(x); Serial.print(' '); Serial.print(y); Serial.print(' '); Serial.print(z);
					Serial.print(" :: "); Serial.print(pitch); Serial.print(' '); Serial.println(roll);
#endif
			xl.pitch.checkSend(pitch);
			xl.roll.checkSend(roll);
			xl.lastReading_ms = theTime;
		}
	}
}


/*********************************************
 * SPI HANDLER
 *********************************************/
MidiRingBuffer<32> spiMidiIn; // midi coming from the pi via SPI
MidiRingBuffer<32> spiMidiOut; // midi going to the pi via SPI

/*!
 * the spi interrupt routine is a pair of ad hoc state machines, with these states
 */
enum spi_io_state_t: uint8_t {
	command_byte = 0,	//!< processing a command byte
	midi_data = 1,		//!< processing midi bytes
	midi_data_1 = 2,		//!< processing midi bytes
	midi_data_2 = 3,		//!< processing midi bytes
	tempo_data = 4,
	tempo_data_1 = 5,
	tempo_data_2 = 6,
	tempo_data_3 = 7,
	config_btn_id = 8,
	config_btn_len = 9,
	config_btn_dta = 10,
	config_ped_id = 11,
	config_ped_len = 12,
	config_ped_dta = 13,
	config_xlr_id = 14,
	config_xlr_len = 15,
	config_xlr_dta = 16,
	config_skip = 17,
	filler = 18
};

spi_io_state_t spi_in_state = command_byte;
spi_io_state_t spi_out_state = command_byte;
uint8_t n_midi_cmd_incoming = 0, n_midi_cmd_outgoing = 0;
uint8_t cmd_in = 0, val1_in = 0, val2_in = 0;
bool dropped_midi_in = false;
uint8_t cmd_out = 0, val1_out = 0, val2_out = 0;
uint8_t config_id = 0;
uint8_t config_len = 0;
uint8_t config_idx = 0;
uint8_t config_data[max(sizeof(config::xlrm8r), sizeof(config::button))];
bool rescan_requested = false;

ISR (SPI_STC_vect)
{
	bool was_pinged = false;
	uint8_t scrap = 0;
	switch (spi_in_state) {
		case command_byte: {
			uint8_t cmd = SPDR;
			if (cmd & xyspi::midi) {
				spi_in_state = midi_data;
				n_midi_cmd_incoming = cmd & 0x7f;
			} else {
				switch (cmd) {
					case xyspi::null:
						break;
					case xyspi::ping:
						was_pinged = true;
						break;
					case xyspi::pong:
						break;
					case xyspi::rescan:
						rescan_requested = true;
						break;
					case xyspi::tempo:
						spi_in_state = tempo_data;
						break;
					case xyspi::cfg_button:
						spi_in_state = config_btn_id;
						break;
					case xyspi::cfg_pedal:
						spi_in_state = config_ped_id;
						break;
					case xyspi::cfg_xlrm8:
						spi_in_state = config_xlr_id;
						break;
				}
			}
			break;
		}
		case midi_data:
			cmd_in = SPDR;
			spi_in_state = midi_data_1;
			break;
		case midi_data_1:
			val1_in = SPDR;
			spi_in_state = midi_data_2;
			break;
		case midi_data_2:
			val2_in =SPDR;
			if (!spiMidiIn.addToBuf(cmd_in, val1_in, val2_in)) {
				dropped_midi_in = true;
			}
			spi_in_state = (--n_midi_cmd_incoming > 0)? midi_data: command_byte;
			break;
		case tempo_data:
			spi_in_state = tempo_data_1;
			break;
		case tempo_data_1:
			spi_in_state = tempo_data_2;
			break;
		case tempo_data_2:
			spi_in_state = tempo_data_3;
			break;
		case tempo_data_3:
			spi_in_state = command_byte;
			break;
		case config_btn_id:
			config_id = SPDR;
			spi_in_state = config_btn_len;
			break;
		case config_btn_len:
			config_len = SPDR;
			config_idx = 0;
			spi_in_state = (config_len == sizeof(config::button))? config_btn_dta : config_skip;
			break;
		case config_btn_dta:
			config_data[config_idx++] = SPDR;
			if ((--config_len) <= 0) {
				spi_in_state = command_byte;
				if (config_id < nButtons) {
					buttons[config_id].set(*((config::button*)config_data));
				}
			}
			break;
		case config_ped_id:
			config_id = SPDR;
			spi_in_state = config_ped_len;
			break;
		case config_ped_len:
			config_len = SPDR;
			config_idx = 0;
			spi_in_state = (config_len == sizeof(config::pedal))? config_ped_dta : config_skip;
			break;
		case config_ped_dta:
			config_data[config_idx++] = SPDR;
			if ((--config_len) <= 0) {
				spi_in_state = command_byte;
				if (config_id < nPedals) {
					pedals[config_id].set(*((config::pedal*)config_data));
				}
			}
			break;
		case config_xlr_id:
			config_id = SPDR;
			spi_in_state = config_xlr_len;
			break;
		case config_xlr_len:
			config_len = SPDR;
			config_idx = 0;
			spi_in_state = (config_len == sizeof(config::xlrm8r))? config_xlr_dta : config_skip;
			break;
		case config_xlr_dta:
			config_data[config_idx++] = SPDR;
			if ((--config_len) <= 0) {
				spi_in_state = command_byte;
				if (config_id < nXlmr8rs) {
					xlmr8rs[config_id].set(*((config::xlrm8r*)config_data));
				}
			}
			break;
		case config_skip:
			scrap = SPDR;
			if ((--config_len) <= 0) spi_in_state = command_byte;
			break;
		case filler:
			spi_in_state = filler;
			break;
		default:
			spi_in_state = command_byte;
			break;
	}

	switch (spi_out_state) {
		case command_byte: {
			n_midi_cmd_outgoing = spiMidiOut.count;
			if (n_midi_cmd_outgoing > 0) {
				SPDR = (xyspi::midi | n_midi_cmd_outgoing);
				spi_out_state = midi_data;
			} else {
				SPDR = xyspi::pong;
				spi_out_state = filler;
			}
			break;
		}
		case midi_data:
			spiMidiOut.getFromBuf(cmd_out, val1_out, val2_out);
			SPDR = cmd_out;
			spi_out_state = midi_data_1;
			break;
		case midi_data_1:
			SPDR = val1_out;
			spi_out_state = midi_data_2;
			break;
		case midi_data_2:
			SPDR = val2_out;
			if ((--n_midi_cmd_outgoing) <= 0) {
				spi_out_state = command_byte;
			} else {
				spi_out_state = midi_data;
			}
			break;
		case filler:
			if (spiMidiOut.count > 0) {
				spi_out_state = command_byte;
				SPDR = xyspi::null; // keep talking!
			} else {
				SPDR = xyspi::pong;
				if (was_pinged) {
					spi_out_state = command_byte;
				}
			}
			break;
		default:
			SPDR = xyspi::null;
			break;
	}
}

/************************************************************
 * MAIN INIT
 ************************************************************/
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

	/* setup SPI to with rasbpi as master */
	pinMode(MISO, OUTPUT);
	SPCR |= _BV(SPE);
	SPI.attachInterrupt();
}

/************************************************************
 * MAIN LOOP
 ************************************************************/
void loop() {
	statusLamp.checkBlink();
	if (midiA.read()) {
#ifdef SERIAL_DEBUG
		Serial.print(midiA.getType(), HEX);Serial.print(' ');
		Serial.print(midiA.getData1(), HEX);Serial.print(' ');
		Serial.print(midiA.getData2(), HEX);Serial.print(' ');
		Serial.println(midiA.getChannel(), HEX);
#endif
		uint8_t midi_typ = midiA.getType();
		switch (midi_typ) {
			case midi::NoteOn:
			case midi::NoteOff:
			case midi::ControlChange:
			case midi::ProgramChange:
			case midi::AfterTouchChannel:
			case midi::AfterTouchPoly:
				spiMidiOut.addToBuf(midi_typ|(midiA.getChannel()-1), midiA.getData1(), midiA.getData2());
				statusLamp.setColor(Lamp8574::blue, 50);
				break;

			case midi::PitchBend:
				break;

			case midi::Clock:
				break;

			case midi::Start:
			case midi::Stop:
			case midi::Continue:
				spiMidiOut.addToBuf(midi_typ, 0, 0);
				statusLamp.setColor(Lamp8574::blue, 50);
				break;
		}
	} else {
//		Serial.println("nope");
	}
	while (spiMidiIn.count > 0) {
		uint8_t cmd, chan, val1, val2;
		spiMidiIn.getFromBuf(cmd, val1, val2);
		chan = (cmd & 0xf) + 1;
		cmd &= 0xf0;
		switch (cmd) {
			case midi::NoteOn:
			case midi::NoteOff:
			case midi::ControlChange:
			case midi::ProgramChange:
			case midi::AfterTouchChannel:
			case midi::AfterTouchPoly:
				midiA.send(static_cast<midi::MidiType>(cmd), val1, val2, chan);
				statusLamp.setColor(Lamp8574::red, 50);
				break;

			case midi::PitchBend:
				break;
				
			case midi::Clock:
				break;

			case midi::Start:
			case midi::Stop:
			case midi::Continue:
				midiA.sendRealTime(static_cast<midi::MidiType>(cmd));
				statusLamp.setColor(Lamp8574::red, 50);
				break;
		}
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
	checkAccelerometer(accelerometer, xlmr8rs[0]);

}