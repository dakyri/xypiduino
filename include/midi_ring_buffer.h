#pragma once

#include <stdint.h>

/*!
 * NOTE small buffer, N < 84 so
 * AVR is limited, so thread/ISR safety is very sketchy
 * incrementing a volatile byte is not atomic!
 * we have a 3 byte/1 command of wasted space so the readidx will only equal writeidx if we are empty
 */
template <int N>
class MidiRingBuffer {
public:
	MidiRingBuffer(): readIdx(0), writeIdx(0) {}

	bool addToBuf(uint8_t cmd, uint8_t v1, uint8_t v2) {
		if (bytesFree() <= 3) {
			return false;
		}
		auto i = writeIdx;
		buffer[i] = cmd;
		buffer[i+1] = v1;
		buffer[i+2] = v2;
		if ((i+=3) >= 3*N) i = 0;
		writeIdx = i;
		return true;
	}

	bool getFromBuf(uint8_t &cmd, uint8_t &v1, uint8_t &v2) {
		if (!hasAvail()) {
			return false;
		}
		auto i = readIdx;
		cmd = buffer[i];
		v1 = buffer[i+1];
		v2 = buffer[i+2];
		if ((i+=3) >= 3*N) i = 0;
		readIdx = i;
		return true;
	}

	/*!
	 * this could be out, but it should never be more than the available
	 */
	uint8_t bytesAvail() {
		auto n = writeIdx - readIdx;
		if (n < 0) n += 3*N;
		return n;
	}
	/*!
	 * this can only work if we never fill up entirely, so readIdx == writeIdx => we have a fully empty buffer
	 */
	uint8_t bytesFree() {
		auto n = readIdx - writeIdx;
		if (n <= 0) n += 3*N;
		return n;
	}

	/*!
	 * number of midi commands available
	 */
	uint8_t itemsAvail() {
		return bytesAvail() / 3;
	}

	/*!
	 * do we have something?
	 * absolutely not thread safe
	 * !!! works if we have an exact multiple of 3 for the buffer size
	 */
	bool hasAvail() {
		return readIdx != writeIdx; 
	}

protected:
	volatile uint8_t readIdx;
	volatile uint8_t writeIdx;

	uint8_t buffer[3*N];
};
