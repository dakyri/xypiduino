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

	constexpr uint8_t maxCmdLen = 256;

};
