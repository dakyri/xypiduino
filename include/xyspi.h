#pragma once

#include <stdint.h>

namespace xyspi {
	enum class cmd: uint8_t {
		greets = 'h',
		sendMidi = 'm',
		sendButton = 'b',
		ok = 'k'
	};

#pragma pack(1)

	struct midi_t {
		uint8_t cmd;
		union val {
			uint16_t bend;
			struct note {
				uint8_t pitch;
				uint8_t vel;
			};
			struct ctrl {
				uint8_t tgt;
				uint8_t amt;
			};
			struct prog {
				uint8_t no;
			};
		};
	};

#pragma pack(pop)

};