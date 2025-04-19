#pragma once

#include <stdint.h>

namespace xymidi {
	enum class bus_cmd: uint8_t {
		greets = 'h',
		sendMidiAny = '0', // or every
		sendMidiPort1 = '1', // specific ports. we probably won't have more than 1 or 2 extras
		sendMidiPort2 = '2',
		sendMidiPort3 = '3',
		sendMidiPort4 = '4',
		sendMidiPort5 = '5',
		sendMidiPort6 = '6',
		sendMidiPort7 = '7',
		sendMidiPort8 = '8',
		sendMidiPort9 = '9',
		setMidiPortCmd = '>',
		sendButton = 'b',
		sendDevice = 'i', // output from a generic i2c device that we haven't got a specific command for
		ok = 'k'
	};

	enum class midi_port_cmd : uint8_t { // 4 bit bit field
		enableThru = 1,
		disableThru = 2
	};

	inline bool isSysCmd(uint8_t cmd) { return (cmd & 0xf0) != 0; }
	inline bool isCmdByte(uint8_t _byte) { return (_byte & 0x80) != 0; }
	inline uint8_t getCCCmd(uint8_t _byte) { return (_byte & 0xf0); }
	inline uint8_t getCCChan(uint8_t _byte) { return (_byte & 0x0f); }

	enum class cmd : uint8_t {
		nul			= 0x00,
		noteOff		= 0x80,
		noteOn		= 0x90,
		keyPress	= 0xa0,
		ctrl		= 0xb0,
		prog		= 0xc0,
		chanPress	= 0xd0,
		bend		= 0xe0,

		sysxStart	= 0xf0,
		timeCode	= 0xf1,
		songPos		= 0xf2,
		songSel		= 0xf3,
		cableMsg	= 0xf5,
		tuneReq		= 0xf6,
		sysxEnd		= 0xf7,
		clock		= 0xf8,
		start		= 0xfa,
		cont		= 0xfb,
		stop		= 0xfc,
		sensing		= 0xfe,
		sysReset	= 0xff
	};

	inline uint8_t operator | (cmd op, uint8_t t) { return static_cast<uint8_t>(op) | t; }
	inline uint8_t operator & (cmd op, uint8_t t) { return static_cast<uint8_t>(op) & t; }
	inline uint8_t operator ^ (cmd op, uint8_t t) { return static_cast<uint8_t>(op) ^ t; }
	inline uint8_t ot(cmd op) { return static_cast<uint8_t>(op); }
	inline cmd ot(uint8_t op) { return static_cast<cmd>(op); }

	enum class midi_ctrl : uint8_t {
		modulation		= 0x01,
		breathController = 0x02,
		footController	= 0x04,
		portamentoTime	= 0x05,
		dataEntry		= 0x06,
		mainVolume		= 0x07,
		midiBalance		= 0x08,
		pan				= 0x0a,
		expressionCtrl	= 0x0b,
		general_1		= 0x10,
		general_2		= 0x11,
		general_3		= 0x12,
		general_4		= 0x13,
		sustainPedal	= 0x40,
		portamento		= 0x41,
		sostenuto		= 0x42,
		softPedal		= 0x43,
		hold_2			= 0x45,
		general_5		= 0x50,
		general_6		= 0x51,
		general_7		= 0x52,
		general_8		= 0x53,
		effects_depth	= 0x5b,
		tremolo_depth	= 0x5c,
		chorus_depth	= 0x5d,
		celeste_depth	= 0x5e,
		phaser_depth	= 0x5f,
		data_increment	= 0x60,
		data_decrement	= 0x61,
		reset_all		= 0x79,
		local_control	= 0x7a,
		all_notes_off	= 0x7b,
		omni_mode_off	= 0x7c,
		omni_mode_on	= 0x7d,
		mono_mode_on	= 0x7e,
		poly_mode_on	= 0x7f,

		tempo_change	= 0x51
	};

#pragma pack(push, 1)

	struct msg {
		msg() : cmd(0), val({0}) { }

		void noteon(uint8_t chan, uint8_t note, uint8_t vel) { cmd = cmd::noteOn | (chan & 0xf); val.note.pitch = note; val.note.vel = vel; }
		void noteon(uint8_t args[3]) { cmd = cmd::noteOn | (args[0] & 0xf); val.note.pitch = args[1]; val.note.vel = args[2]; }
		void noteoff(uint8_t chan, uint8_t note, uint8_t vel) { cmd = cmd::noteOff | (chan & 0xf); val.note.pitch = note; val.note.vel = vel; }
		void keypress(uint8_t chan, uint8_t note, uint8_t vel) { cmd = cmd::keyPress | (chan & 0xf); val.note.pitch = note; val.note.vel = vel; }
		void control(uint8_t chan, uint8_t tgt, uint8_t amt) { cmd = cmd::ctrl | (chan & 0xf); val.ctrl.tgt = tgt; val.ctrl.amt = amt; }
		void prog(uint8_t chan, uint8_t prog) { cmd = cmd::prog | (chan & 0xf); val.prog = prog; }
		void chanpress(uint8_t chan, uint8_t press) { cmd = cmd::chanPress | (chan & 0xf); val.press = press; }
		void bend(uint8_t chan, uint16_t bend) { cmd = cmd::bend | (chan & 0xf); val.bend = bend; }
		void timecode(uint8_t typ, uint8_t v) { cmd = ot(cmd::timeCode); val.time_code.type = typ; val.time_code.val = v; }
		void songpos(uint16_t pos) { cmd = ot(cmd::songPos); val.song_pos = pos; }
		void songsel(uint8_t sel) { cmd = ot(cmd::songSel); val.song_sel = sel; }
		void tune() { cmd = ot(cmd::tuneReq); }
		void clock() { cmd = ot(cmd::clock); }
		void start() { cmd = ot(cmd::start); }
		void cont() { cmd = ot(cmd::cont); }
		void stop() { cmd = ot(cmd::stop); }

		uint8_t channel() { return cmd & 0xf; }

		uint8_t cmd;
		union {
			uint16_t bend;
			uint16_t song_pos;
			uint8_t song_sel;
			uint8_t press;
			uint8_t prog;
			struct {
				uint8_t pitch;
				uint8_t vel;
			} note;
			struct {
				uint8_t tgt;
				uint8_t amt;
			} ctrl;
			struct {
				uint8_t type;
				uint8_t val;
			} time_code;
		} val;
	};

	struct midi_port_cmd_t {
		uint8_t port : 4;
		uint8_t cmd : 4;
	};

#pragma pack(pop)

};