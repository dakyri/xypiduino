#pragma once

#include <stdint.h>

/*!
 * protocols for communicating between the arduino and the rasbpi on spi bus
 */

namespace xyspi {

	/*!
	 * command byte is the first byte of the stream, followed maybe by data
	 */
	enum cmd_t: uint8_t {
		midi = 0x80,	//!< regular midi cmd stream. rest of byte is #cmds, followed by 3 * n midi commands
		null = 0,		//!< sent when we have an spi data length catchup between rx and tx
		ping = 1,		//!< HALLLOOO
		pong = 2,		//!< I'M FUCKING HERE OK
		rescan = 3,		//!< rescan the i2c bus for some weird plugnplay shit. ie. xlrm8rs
		tempo = 4,		//!< next 4 bytes are the tempo as a float
		send_tempo = 5,	//!< send an updated tempo.
		cfg_button = 6,	//!< upload button settings and mappings.
						//!< 1 byte for button number, 1 byte blob size, blob of button config data
		cfg_pedal = 7,	//!< upload pedal settings and mappings. 1 byte pedal number, 1 byte blob size, blob data
		cfg_xlrm8 = 8,	//!< upload accelerometer settings and mappings. 1 byte xlm8r number, 1 byte blob size, blob data
		set_masterclk = 9,		//!< single byte. what it says
		set_masterclk_off = 10,	//!< single byte. what it says
		set_midithru = 11,		//!< single byte. what it says
		set_midithru_off = 12,	//!< single byte. what it says
		diag_message = 13,	//!< diagnostic message (duino -> pi). 1 byte length in bytes, followed by multiple bytes
	};

};

namespace config {
	enum mode_t: uint8_t {
		nieko = 0,
		prog = 1,
		ctrl = 2,
		chanpress = 3,
		keypress = 4,
		note = 5,
		bend = 6,
		start = 7,
		stop = 8,
		type = 0x0f,
		latch = 0x10,

		// in button field of pedal config
		no_shft = 0xff, // no alt mode or shift button 
		all_btn = 0xf0, // shift mode on all buttons
		// in channel field of pedal config
		btn_chn = 0x80, // shift mode uses the button midi channel
		// in the alt_which button mapping
		nc = 0xff // shift mode doesn't use this button
	};

constexpr uint8_t nButtons = 8;

#pragma pack(push, 1)
	struct button {
		uint8_t chan;
		uint8_t mode;
		uint8_t val1;
		uint8_t val2;
		uint8_t long_mode;
		uint8_t long_val1;
		uint8_t long_val2;
	};

	struct pedal {
		uint8_t chan;			// midi channel
		uint8_t mode;			// actually. ctrl or maybe pressure or even note
		uint8_t min_val;		// some part of the range 0-127
		uint8_t max_val;
		uint8_t which;			// controller or maybe note number
		uint8_t shift_button;	// which panel button, if any, alters it.
		uint8_t alt_chan;		// if shift button => alternate channel
		uint8_t alt_mode;		// if shift button => actually. ctrl or maybe pressure or even note
		uint8_t alt_min_val;	// if shift button => some part of the range 0-127
		uint8_t alt_max_val;
		uint8_t alt_which[nButtons];	// if shift button => controller or maybe note number
	};

	struct xlrm8r {
		uint8_t device_id;		// i2c id is the only way we have to distinguish theese
		uint8_t chan;			// midi channel
		uint8_t enable_button;	// which panel button, if any, enables it.
		uint8_t mode_x;			// pitch, actually. ctrl or maybe pressure or even note
		int8_t	min_ambit_x;	// ambit in range -127 -> 127 ... probably -45 -> +45  is the most useful
		int8_t	max_ambit_x;
		uint8_t min_x_val;		// some part of the range 0-127
		uint8_t max_x_val;
		uint8_t which_x[nButtons];	// controller or maybe note number
		uint8_t mode_y;			// roll, actually. ctrl or maybe pressure or even note
		int8_t	min_ambit_y;	// ambit in range -127 -> 127 ... probably -45 -> +45  is the most useful
		int8_t	max_ambit_y;
		uint8_t min_y_val; 		// some part of the range 0-127
		uint8_t max_y_val;
		uint8_t which_y[nButtons];	// controller or maybe note number
	};

#pragma pack(pop)

};