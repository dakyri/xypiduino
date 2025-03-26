#pragma once

#include <stdint.h>

template <int N>
class MidiRingBuffer {
public:
	MidiRingBuffer(): count(0), readIdx(-1), writeIdx(0) {}

	bool addToBuf(uint8_t cmd, uint8_t v1, uint8_t v2) {
		auto i = writeIdx;
		if (i == readIdx) {
			return false;
		}
		buffer[i] = cmd;
		buffer[i+1] = v1;
		buffer[i+2] = v2;
		if (readIdx < 0) readIdx = i;
		if ((i+=3) >= 3*N) i = 0;
		writeIdx = i;
		count++;
		return true;
	}

	bool getFromBuf(uint8_t &cmd, uint8_t &v1, uint8_t &v2) {
		if (count == 0) {
			return false;
		}
		auto i = readIdx;
		cmd = buffer[i];
		v1 = buffer[i+1];
		v2 = buffer[i+2];
		if ((i+=3) >= 3*N) i = 0;
		readIdx = (i == writeIdx? -1: i);
		count--;
		return true;
	}

	volatile uint8_t count;

protected:
	volatile uint16_t readIdx;
	volatile uint16_t writeIdx;

	uint8_t buffer[3*N];
};
