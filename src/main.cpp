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
#include "xycfg.h"
#include "midi_ring_buffer.h"

#define SERIAL_DEBUG
#define SERIAL_DEBUG_LEVEL 1
#define ENABLE_SPI
#define DIAG_MESSAGE

midi::MidiInterface<HardwareSerial> midiA((HardwareSerial&)Serial1);

using color=RGBLempos::color;

constexpr uint8_t buttonStateI2CAdr = 0x38;
constexpr uint8_t adcEnableI2CAdr = 0x39;
constexpr uint8_t adcI2CAdr = 0x48;

constexpr uint16_t xlm8rReadDelay_ms = 100;
constexpr uint8_t debounceDelay_ms = 10;
constexpr uint16_t longPressTime_ms = 500;

constexpr uint8_t pedalChangeMinTime_ms = 50;
constexpr uint8_t nCachedClockTicks = 24;

RGBLempos lamps(19, 21, 20);
PCF8574<I2c> buttonStates(buttonStateI2CAdr);
PCF8574<I2c> adcEnable(adcEnableI2CAdr);
PCF8591<I2c> adc(adcI2CAdr);
adxl345::Interface<I2c> accelerometer(adxl345::kAddress1);

MidiRingBuffer<32> deviceMidiOut; // midi going to the duino midi out, coming from the pi via SPI, buttons, pedal, and stuff
#ifdef ENABLE_SPI
MidiRingBuffer<32> spiMidiOut; // midi going to the pi via SPI, includes the pedals, buttons and accelerometers
#endif

float globalTempo = 120;
float beatLength_s = 0.5;
int16_t clockDurCache[nCachedClockTicks]; // keep track of past ticks so we can make a running average
uint8_t clockCacheHead = -1;
uint8_t clockCacheTail = -1;
int16_t tickDurationRunningTotal = 0;
unsigned long lastTickMillis = 0;
unsigned long nextTickMillis = 0;
int16_t runningDurTotal = 0;
bool isMidiMasterClock = false;
volatile bool cancelMidiMasterClock = false;
volatile bool startMidiMasterClock = false;

#ifdef DIAG_MESSAGE
volatile bool hasDiagMessage = false;
volatile uint8_t diagMessageLength = 0;
volatile uint8_t diagMessageIdx = 0;
constexpr uint8_t maxDiagMessageLength = 32;
uint8_t diagMessage[maxDiagMessageLength];

/*!
 * build diagnostic messages to send up spi to the pi
 */
void addToMsg(const char *p) {
	hasDiagMessage = true;
	while (diagMessageLength < maxDiagMessageLength) {
		diagMessage[diagMessageLength++] = *p;
		if (*p++ == '\0') {
			break;
		}
	}
}
#endif

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

using mod = config::mode_t;

bool do_close = false;
bool close_mode = mod::nieko;
bool close_chan = 0;
bool close_val1 = 0;

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

	/*!
	 * might be called from an ISR
	 */
	void set(config::button &cfg, bool and_close) {
		if (and_close) {
			close_mode = mode;
			close_chan = channel;
			close_val1 = val1;
			do_close = true;
		}
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

	/*!
	 * in any kind of pressed or latched state 
	 */
	bool active() {
		return state | (state_t::latched | state_t::latching | state_t::pressed | state_t::longPressed); 	
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

#define LATCH(X) ((config::mode_t)(X|mod::latch))

struct button buttons[] = {
	{{ 0, mod::ctrl, 32, 127,		mod::latch, 0, 0}},
	{{ 0, mod::note, 33, 80,			mod::latch, 0, 0}},
	{{ 0, LATCH(mod::ctrl), 34, 80,	mod::ctrl, 34, 120}},
	{{ 0, mod::prog, 35, 33,			mod::latch, 0, 0}},
	{{ 0, mod::ctrl, 36, 120,		mod::latch, 0, 0}},
	{{ 0, mod::note, 37, 80,			LATCH(mod::keypress), 37, 100}},
	{{ 0, mod::start, 38, 120,		mod::latch, 0, 0}},
	{{ 0, mod::nieko, 39, 120,		mod::latch, 0, 0}},
};
constexpr uint8_t nButtons = sizeof(buttons)/sizeof(button);
using btst8 = button::state_t;

/*!
 * mappings for pedals
 * the minval and maxval are [0, 127] for pedals and bends, just to simplify the config. bends will be scaled up
 * and spread across two bytes
 */
struct pedal {
	pedal(config::pedal cfg) {
		set(cfg);
	}

	void set(config::pedal &cfg) {
		lastChangeTime_ms = 0;
		lastCV = 255;
		lastCV2 = 255;
		ctrl = cfg.which;
		mode = cfg.mode;
		chan = cfg.chan;
		minVal = cfg.min_val;
		maxVal = cfg.max_val;
		factor = (maxVal - minVal + 1);
		factor /= (mode == mod::bend? 2.0 : 256.0);
		alt_mode = cfg.alt_mode;
		alt_chan = cfg.alt_chan;
		alt_minVal = cfg.alt_min_val;
		alt_maxVal = cfg.alt_max_val;
		auto p = alt_ctrl;
		for (const auto i: cfg.alt_which) {
			*p++ = i;
		}
		alt_factor = (alt_maxVal - alt_minVal + 1);
		alt_factor /= (alt_mode == mod::bend? 2.0 : 256.0);

	}

	long lastChangeTime_ms;
	float factor = 0.5;
	float alt_factor = 0.5;

	uint8_t shift_btn;

	uint8_t mode;
	uint8_t ctrl;
	uint8_t chan;
	uint8_t minVal;
	uint8_t maxVal;

	uint8_t alt_mode;
	uint8_t alt_chan;
	uint8_t alt_minVal;
	uint8_t alt_maxVal;
	uint8_t alt_ctrl[config::nButtons];

	uint8_t lastCV;
	uint8_t lastCV2;
};

struct pedal pedals[] = {
	{{ 0, mod::ctrl, 0, 127, 85,	0, 1, mod::ctrl, 0, 127,
									{12, 0, 0, 0, 0, 0, 0, 0} }},
	{{ 0, mod::ctrl, 0, 127, 86, 	mod::all_btn,	mod::btn_chn, mod::ctrl, 0, 127,
									{86, 86, 86, 86, 86, 86, 86, 86} }},
	{{ 0, mod::keypress, 0, 127, 0, mod::all_btn,	mod::btn_chn, mod::bend, 0, 127,
									{mod::nc, 0, mod::nc, mod::nc, mod::nc, 0, mod::nc, mod::nc} }}
};
constexpr uint8_t nPedals = sizeof(pedals)/sizeof(pedal);

constexpr uint8_t lmpBtOpenCol = color::green;
constexpr uint8_t lmpBtDownCol = color::red;
constexpr uint8_t lmpBtLatchCol = color::blue;
/*!
 *
 */

 void sendButtonOnMidi(uint8_t typ, uint8_t chan, uint8_t v1, uint8_t v2) {
	uint8_t cmd = mod::nieko;
	switch (typ) {
		case mod::prog:
			cmd = static_cast<uint8_t>(midi::ProgramChange);
			break;
		case mod::ctrl:
			cmd = static_cast<uint8_t>(midi::ControlChange);
			break;
		case mod::keypress:
			cmd = static_cast<uint8_t>(midi::AfterTouchPoly);
			break;
		case mod::chanpress:
			cmd = static_cast<uint8_t>(midi::AfterTouchChannel);
			break;
		case mod::note:
			cmd = static_cast<uint8_t>(midi::NoteOn);
			break;
		case mod::bend:
			cmd = static_cast<uint8_t>(midi::PitchBend) ;
			break;
		case mod::start:
			cmd = static_cast<uint8_t>(midi::Start);
			break;
		case mod::stop:
			cmd = static_cast<uint8_t>(midi::Stop);
			break;
	}
	if (cmd) {
#ifdef ENABLE_SPI
		spiMidiOut.addToBuf(cmd | chan, v1, v2); // send it also to the pi via the spi bus
#endif
		midiA.send(static_cast<midi::MidiType>(cmd), v1, v2, chan + 1); // send that to an actual midi device
		statusLamp.setColor(Lamp8574::red, 100);
	}
};

void sendButtonOffMidi(uint8_t typ, uint8_t chan, uint8_t v1) {
	uint8_t cmd = mod::nieko;
	uint8_t v2 = 0;
	switch (typ) {
		case mod::ctrl:
			cmd = static_cast<uint8_t>(midi::ControlChange) | chan;
			break;
		case mod::bend:
			cmd = static_cast<uint8_t>(midi::PitchBend) | chan;
			v1 = 0x20;
			break;
		case mod::note:
			cmd = static_cast<uint8_t>(midi::NoteOff) | chan;
			break;
		case mod::start:
			cmd = midi::Stop;
			break;
		case mod::stop:
			cmd = midi::Start;
		break;
	}
	if (cmd) {
#ifdef ENABLE_SPI
		spiMidiOut.addToBuf(cmd | chan, v1, v2); // send it also to the pi via the spi bus
#endif
		midiA.send(static_cast<midi::MidiType>(cmd), v1, v2, chan + 1); // send that to an actual midi device
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
				const auto btype = b.mode & mod::type;
				b.active_type = btype;

				if (b.mode & mod::latch){
					b.state |= (btst8::latched|btst8::latching);
				}
				sendButtonOnMidi(btype, b.channel, b.val1, b.val2);
				lamps.setLamp(which, (b.state & btst8::latched)? lmpBtLatchCol: lmpBtDownCol);
			} else if (!(b.state & btst8::longPressed) && b.pressedTime_ms + longPressTime_ms < theTime) {
				b.state |= btst8::longPressed;
				const auto btype = b.long_mode & mod::type;
				if (b.long_mode & mod::latch){
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
				const auto btype = b.active_type & mod::type;

				sendButtonOffMidi(btype, b.channel, b.val1);
				lamps.setLamp(which, b.state & btst8::latched? lmpBtLatchCol: lmpBtOpenCol);
			}
		}
	}
}

/*!
 *
 */
uint8_t checkShiftButton(uint8_t shift_btn, uint8_t alt_chan, uint8_t &chan, uint8_t alt_ctrl[], uint8_t &ctrlVal1){
	if (shift_btn == mod::no_shft) {
		return mod::no_shft;
	}
	if (shift_btn != mod::all_btn) {
		if (shift_btn < nButtons && buttons[shift_btn].active()) {
			ctrlVal1 = alt_ctrl[0];
			chan = alt_chan;
			return shift_btn;
		}
	}
	for (uint8_t i=0; i<nButtons; i++) {
		if (alt_ctrl[i] != mod::nc && buttons[i].active()) {
			ctrlVal1 = alt_ctrl[i];
			chan = (alt_chan == mod::btn_chn? alt_chan: buttons[i].channel);
			return i;
		}
	}
	return mod::no_shft;
}

/*!
 * we might need some low pass filtering here.
 * 'cv' is the raw voltage reading in the range [0, 255]
 */
void checkPedal(const uint8_t which, const uint8_t cv) {
	auto &ped{pedals[which]};
	int8_t chg = cv - ped.lastCV;

	if (chg != 0) {
		auto theTime = millis();
		auto potChangeT_ms = theTime - ped.lastChangeTime_ms;
		if (potChangeT_ms > pedalChangeMinTime_ms) {
			ped.lastCV2 = ped.lastCV;
			ped.lastChangeTime_ms = theTime;

			uint8_t cmd = mod::nieko;
			uint8_t mode = ped.mode;
			uint8_t ctrlVal2;
			uint8_t ctrlVal1 = ped.ctrl;
			uint8_t chan = ped.chan;
			float factor = ped.factor;
			
			if (checkShiftButton(ped.shift_btn, ped.alt_chan, chan, ped.alt_ctrl, ctrlVal1) != mod::no_shft) {
				mode = ped.alt_mode;
				factor = ped.alt_factor;
			}
			
			if (mode != mod::bend) {
				ctrlVal2 = static_cast<uint8_t>(cv * factor); // factor <= 0.5
			} else {
				uint16_t full_bend = cv * factor; // bend has a large factor
				ctrlVal2 = static_cast<uint8_t>(full_bend & 0x7f);
				ctrlVal1 = static_cast<uint8_t>((full_bend >> 7) & 0x7f);
			}
			switch (mode) {
				case mod::ctrl:
					cmd = midi::ControlChange;
					break;
				case mod::chanpress:
					cmd = midi::AfterTouchChannel;
					break;
				case mod::keypress:
					cmd = midi::AfterTouchPoly;
					break;
				case midi::PitchBend:
					// range is 0–16383, with center value 8192
					cmd = midi::PitchBend;
					break;
			}
			if (cmd) {
#ifdef ENABLE_SPI
				spiMidiOut.addToBuf((cmd|chan), ctrlVal1, ctrlVal2);
#endif
				midiA.send(static_cast<midi::MidiType>(cmd), ctrlVal1, ctrlVal2, chan); // send that to an actual midi device
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
	axis_ctl(uint8_t _mode, uint8_t _chan, uint8_t _ctrl[], int _min, int _max, uint8_t _ctrlMin, uint8_t _ctrlMax) {
		set(_mode, _chan, _ctrl, _min, _max, _ctrlMin, _ctrlMax);
	}

	void set(uint8_t _mode, uint8_t _chan, uint8_t _ctrl[], int _min, int _max, uint8_t _ctrlMin, uint8_t _ctrlMax) {
		mode = _mode;
		chan = _chan;
		for (uint8_t i=0; i<nButtons; i++) {
			ctrl[i] = _ctrl[i];
		}
		minV = _min;
		maxV = _max;
		ctrlMin = _ctrlMin;
		ctrlMax = _ctrlMax;
		lastCV = 255;
		factor = (ctrlMax != ctrlMin)? float(ctrlMax - ctrlMin)/float(maxV - minV) : 0;
		if (mode == mod::bend) factor *= 128;
	}

	void checkSend(uint8_t en_but, float angle) {
		if (angle < minV) angle = minV;
		else if (angle > maxV) angle = maxV;
		uint16_t cv = ctrlMin + (angle - minV) * factor;
		uint8_t val1 = ctrl[0];
		uint8_t midi_chan = chan;
		if (checkShiftButton(en_but, midi_chan, midi_chan, ctrl, val1) == mod::no_shft) {
			return;
		}

		if (cv != lastCV) {
#ifdef SERIAL_DEBUG
			Serial.print("axus control "); Serial.print(val1); Serial.print(" angle "); Serial.print(angle);Serial.print(" minV "); Serial.print(minV);Serial.print(" maxV "); Serial.print(maxV);Serial.print(" factor "); Serial.print(factor);Serial.print(" cv "); Serial.println(cv);
#endif 
			uint8_t cmd = mod::nieko;
			uint8_t val2 = cv & 0x7f;
			switch (mode) {
				case mod::ctrl:
					cmd = midi::ControlChange;
					break;
				case mod::chanpress:
					cmd = midi::AfterTouchChannel;
					break;
				case mod::keypress:
					cmd = midi::AfterTouchPoly;
					break;
				case midi::PitchBend:
					val1 = ((cv >> 7) & 0x7f);
					cmd = midi::PitchBend;
					break;
			}
			if (cmd) {
#ifdef ENABLE_SPI
				spiMidiOut.addToBuf((cmd|midi_chan), val1, val2);
#endif
				midiA.send(static_cast<midi::MidiType>(cmd), val1, val2, midi_chan); // send that to an actual midi device
				statusLamp.setColor(Lamp8574::blue, 50);
			}
			lastCV = cv;
		}
	}
	
	float factor;
	uint16_t lastCV;
	uint8_t mode;
	uint8_t chan;
	uint8_t ctrl[nButtons];
	int8_t minV;	// assuming we range -127..127, and probably a smaller ambit than even 180. that's all the config supports
					// and sanity dictates. easy enough to revise this. for current usage [-45,45] is fine
	int8_t maxV;	// and even [-90, 90] is pretty acrobatic
	uint8_t ctrlMin; // ctrl value at min 0..127
	uint8_t ctrlMax; // ctrl value at max . may be > or < than CtrlMin
 
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
	{{adxl345::kAddress1,	0, 2,
			/* pitch */	mod::ctrl, -45, 45, 0, 127, {51, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc},
			/* roll */	mod::ctrl, -45, 45, 0, 127, {50, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc, mod::nc}}}
};
constexpr uint8_t nXlmr8rs = sizeof(xlmr8rs)/sizeof(xl_joy);


bool checkXlButton(const xl_joy &xl)
{
	if (xl.activeButton == mod::no_shft) return true;
	if (xl.activeButton != mod::all_btn && buttons[xl.activeButton].active()) return true;
	for (uint8_t i=0; i<nButtons; ++i) {
		if (buttons[i].active() && ((xl.pitch.ctrl[i] & xl.roll.ctrl[i]) != mod::nc)) { // nc == 0xff -> simplify this check
			return true;
		}
	}
	return false;
}


void checkAccelerometer(adxl345::Interface<I2c> & xlm8r, struct xl_joy &xl) {
	if (xlm8r.isValid && checkXlButton(xl)) {
		long theTime = millis();
		if (theTime - xl.lastReading_ms > xlm8rReadDelay_ms) {
			int16_t x, y, z;
			xlm8r.readRaw(x, y, z);
			adxl345::Vector v(x,y,z);
/* filtered = filtered.lowPassFilter(v, 0.5); // Low Pass Filter to smooth out data. 0.1 - 0.9 */
// convert to degrees: these are probably going to be simpler to work with if we need to configure on the fly through front panel
			float pitch = -atan2(v.x, sqrt(v.y*v.y + v.z*v.z))*kRadGrad;
			float roll	= atan2(v.y, v.z)*kRadGrad;
#if defined(SERIAL_DEBUG) && SERIAL_DEBUG_LEVEL > 2
			Serial.print("xyz ->");Serial.print(x); Serial.print(' '); Serial.print(y); Serial.print(' '); Serial.print(z);
					Serial.print(" :: "); Serial.print(pitch); Serial.print(' '); Serial.println(roll);
#endif
			xl.pitch.checkSend(xl.activeButton, pitch);
			xl.roll.checkSend(xl.activeButton, roll);
			xl.lastReading_ms = theTime;
		}
	}
}


/*********************************************
 * SPI HANDLER
 *********************************************/

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
	diag_message_length = 18,
	diag_message_data = 19,
	filler = 20,
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
uint8_t config_data[max(sizeof(config::pedal), max(sizeof(config::xlrm8r), sizeof(config::button)))];

volatile float incoming_tempo = 0;
volatile float outgoing_tempo = 0;
volatile bool set_tempo = false;
volatile bool rescan_requested = false;
volatile bool tempo_requested = false;

#ifdef ENABLE_SPI
/*!
 * service routine for SPI interupts.
 * This could easily happen while we are adding to the spi out midi ring buffer or taking from the device out midi
 * ring buffer.
 * We are trying to minimize locking or avoid completely, so we are playing fast and loose for the moment
 */
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
					case xyspi::send_tempo:
						tempo_requested = true;
						break;
					case xyspi::tempo:
						spi_in_state = tempo_data;
						set_tempo = false;
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
					case xyspi::set_masterclk:
						startMidiMasterClock = true;
						break;
					case xyspi::set_masterclk_off:
						isMidiMasterClock = false;
						break;
					case xyspi::set_midithru:
						break;
					case xyspi::set_midithru_off:
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
			if (!deviceMidiOut.addToBuf(cmd_in, val1_in, val2_in)) {
				dropped_midi_in = true;
			}
			spi_in_state = (--n_midi_cmd_incoming > 0)? midi_data: command_byte;
			break;
		case tempo_data:
			spi_in_state = tempo_data_1;
			((uint8_t*)&incoming_tempo)[0] = SPDR;
			break;
		case tempo_data_1:
			spi_in_state = tempo_data_2;
			((uint8_t*)&incoming_tempo)[1] = SPDR;
			break;
		case tempo_data_2:
			spi_in_state = tempo_data_3;
			((uint8_t*)&incoming_tempo)[2] = SPDR;
			break;
		case tempo_data_3:
			spi_in_state = command_byte;
			((uint8_t*)&incoming_tempo)[3] = SPDR;
			set_tempo = true;
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
					buttons[config_id].set(*((config::button*)config_data), true);
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
			n_midi_cmd_outgoing = spiMidiOut.itemsAvail();
#ifdef DIAG_MESSAGE
			if (hasDiagMessage) {
				hasDiagMessage = false;
				SPDR = xyspi::diag_message;
				spi_out_state = diag_message_length;
			} else 
#endif
			if (n_midi_cmd_outgoing > 0) {
				SPDR = (xyspi::midi | n_midi_cmd_outgoing);
				spi_out_state = midi_data;
			} else if (cancelMidiMasterClock) {
				SPDR = xyspi::set_masterclk_off;
				cancelMidiMasterClock = false;
			} else if (tempo_requested) {
				SPDR = xyspi::tempo;
				tempo_requested = false;
				outgoing_tempo = globalTempo;
				spi_out_state = tempo_data;
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
		case tempo_data:
			spi_out_state = tempo_data_1;
			SPDR = ((uint8_t*)&outgoing_tempo)[0];
			break;
		case tempo_data_1:
			spi_out_state = tempo_data_2;
			SPDR = ((uint8_t*)&outgoing_tempo)[1];
			break;
		case tempo_data_2:
			spi_out_state = tempo_data_3;
			SPDR = ((uint8_t*)&outgoing_tempo)[2];
			break;
		case tempo_data_3:
			spi_out_state = command_byte;
			SPDR = ((uint8_t*)&outgoing_tempo)[3];
			break;
#ifdef DIAG_MESSAGEs
		case diag_message_length:
			spi_out_state = diagMessageLength > 0? diag_message_data : command_byte;
			diagMessageIdx = 0;
			SPDR = diagMessageLength;
			break;
		case diag_message_data:
			SPDR = diagMessage[diagMessageIdx];
			if (++diagMessageIdx >= diagMessageLength) {
				spi_out_state = command_byte;
				hasDiagMessage = false;
			}
			break;
#endif
		case filler:
			if (spiMidiOut.hasAvail() > 0) {
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
#endif

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

#ifdef ENABLE_SPI
	/* setup SPI to with rasbpi as master */
	pinMode(MISO, OUTPUT);
	SPCR |= _BV(SPE);
	SPI.attachInterrupt();
#endif
}

/************************************************************
 * MAIN LOOP
 ************************************************************/
void loop() {
	// check our stat lamp. Maybe it's blink is off and it will be reset to the default color
	statusLamp.checkBlink();

	if (do_close) {
		sendButtonOffMidi(close_mode, close_chan, close_val1);
		do_close = false;
	}

	if (set_tempo) {
		globalTempo = incoming_tempo;
		beatLength_s = 60 / globalTempo;
		set_tempo = false;
	}

	auto theTime = millis();

	// process incoming midi data, sending anything useful down the spi bus to the pi
	if (midiA.read()) {
#ifdef SERIAL_DEBUG
		Serial.print(midiA.getType(), HEX);Serial.print(' ');
		Serial.print(midiA.getData1(), HEX);Serial.print(' ');
		Serial.print(midiA.getData2(), HEX);Serial.print(' ');
		Serial.println(midiA.getChannel(), HEX);
#endif
#ifdef ENABLE_SPI
		uint8_t midi_typ = midiA.getType();
		switch (midi_typ) {
			case midi::NoteOn:
			case midi::NoteOff:
			case midi::ControlChange:
			case midi::ProgramChange:
			case midi::AfterTouchChannel:
			case midi::AfterTouchPoly:
			case midi::PitchBend:
				spiMidiOut.addToBuf(midi_typ|(midiA.getChannel()-1), midiA.getData1(), midiA.getData2());
				statusLamp.setColor(Lamp8574::blue, 50);
				break;

			case midi::Clock:
				if (isMidiMasterClock) {
					isMidiMasterClock = false;
					cancelMidiMasterClock = true;
				}
				if (lastTickMillis == 0) {
					lastTickMillis = theTime;
				} else {
					auto tickDur = theTime - lastTickMillis;
					uint8_t nCached;
					if (clockCacheHead < 0) {
						clockCacheTail = clockCacheHead = 0;
						clockDurCache[0] = runningDurTotal = tickDur;
						nCached = 1;
					} else {
						auto f = clockCacheTail - clockCacheHead - 1;
						if (f == 0 || f == -nCachedClockTicks) /* full */{
							runningDurTotal -= clockDurCache[clockCacheTail];
							if (++clockCacheTail >= nCachedClockTicks) clockCacheTail = 0;
							nCached = nCachedClockTicks;
						} else {
							if (f < 0) f += nCachedClockTicks; 
							nCached = nCachedClockTicks - f;
						}
						if (++clockCacheHead >= nCachedClockTicks) clockCacheHead = 0;
						clockDurCache[clockCacheHead] = tickDur;
						runningDurTotal += tickDur;
					}
					auto aveBeatLen_ms = runningDurTotal;
					if (nCached != 24) aveBeatLen_ms *= 24.0 / nCached;
					globalTempo = 60000.0 / aveBeatLen_ms; 
				}
				break;

			case midi::Start:
			case midi::Stop:
			case midi::Continue:
				spiMidiOut.addToBuf(midi_typ, 0, 0);
				statusLamp.setColor(Lamp8574::blue, 50);
				break;
		}
#endif
	} else {
//		Serial.println("nope");
	}

	if (startMidiMasterClock) {
		isMidiMasterClock = true;
		nextTickMillis = theTime;
	}

	if (isMidiMasterClock) {
		if (theTime >= nextTickMillis) {
			midiA.sendRealTime(midi::Clock);
			nextTickMillis = nextTickMillis + (beatLength_s * 1000 / 24.0);
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

	// Now send everything that has come from the spi bus to an actual midi device
	// This includes all the button presses, accelerometers and pedals
	while (deviceMidiOut.hasAvail() > 0) {
		uint8_t cmd, chan, val1, val2;
		deviceMidiOut.getFromBuf(cmd, val1, val2);
		chan = (cmd & 0xf) + 1;
		cmd &= 0xf0;
		switch (cmd) {
			case midi::NoteOn:
			case midi::NoteOff:
			case midi::ControlChange:
			case midi::ProgramChange:
			case midi::AfterTouchChannel:
			case midi::AfterTouchPoly:
			case midi::PitchBend:
				midiA.send(static_cast<midi::MidiType>(cmd), val1, val2, chan);
				statusLamp.setColor(Lamp8574::red, 50);
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

	if (rescan_requested) { // look around for a new i2c device
		rescan_requested = false;
		if (accelerometer.begin()) {
			accelerometer.setRange(adxl345::kRange16G);
		}
			
	}
}