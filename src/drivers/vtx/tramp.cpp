/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "tramp.h"
#include <inttypes.h>
#include <string.h>
#include <cerrno>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <math.h>

namespace vtx
{

int Tramp::get_settings()
{
	return get_status();
}


bool Tramp::print_settings()
{
	PX4_INFO("Tramp:");
	PX4_INFO("  frequency: %hu MHz", _settings.frequency);
	PX4_INFO("  requested power: %hu mW", _settings.requested_power_mW);
	PX4_INFO("  power: %hu mW", _settings.power_mW);
	PX4_INFO("  pit mode: %s", _settings.pit_mode ? "on" : "off");

	PX4_INFO("  temperature: %hi C", _settings.temperature);

	PX4_INFO("  min frequency: %hu MHz", _settings.min_frequency);
	PX4_INFO("  max frequency: %hu MHz", _settings.max_frequency);
	PX4_INFO("  max power: %hu mW", _settings.max_power_mW);
	return true;
}

void Tramp::print_diagnostics()
{
	PX4_INFO("Tramp UART: baud %d, RX %" PRIu32 " bytes, %" PRIu32 " valid, %" PRIu32 " echo, %" PRIu32
		 " invalid, last error %d, write-only: %s", _smartbaud, _rx_bytes, _valid_frames, _echo_frames, _invalid_frames,
		 _last_query_error, _write_only ? "yes" : "no");
}

bool Tramp::copy_to(vtx_s *msg)
{
	msg->protocol = vtx_s::PROTOCOL_TRAMP;

	msg->frequency = _settings.frequency;
	msg->power_level = _requested_power_level;
	msg->mode = _settings.pit_mode ? vtx_s::MODE_PIT : vtx_s::MODE_NORMAL;

	return true;
}

int Tramp::get_status()
{
	if (_write_only) { return PX4_OK; }

	const uint8_t buf[] = {COMMAND_GET_SETTINGS};
	const int rv = transmit(buf, sizeof(buf));

	if (rv == WRITE_ONLY) { return PX4_OK; }
	if (rv != 0) { return rv; }

	_settings.frequency = (_rx_buf[2] | (_rx_buf[3] << 8));
	_settings.requested_power_mW = (_rx_buf[4] | (_rx_buf[5] << 8));
	_settings.control_mode = _rx_buf[6];
	_settings.pit_mode = _rx_buf[7];
	_settings.power_mW = (_rx_buf[8] | (_rx_buf[9] << 8));

	return PX4_OK;
}

int Tramp::get_temperature()
{
	if (_write_only) { return PX4_OK; }

	const uint8_t buf[] = {COMMAND_GET_TEMPERATURE};
	const int rv = transmit(buf, sizeof(buf));

	if (rv == WRITE_ONLY) { return PX4_OK; }
	if (rv != 0) { return rv; }

	_settings.temperature = int16_t(_rx_buf[6] | (_rx_buf[7] << 8));

	return PX4_OK;
}

int Tramp::reset()
{
	if (_write_only) { return PX4_OK; }

	const uint8_t buf[] = {COMMAND_RESET};
	const int rv = transmit(buf, sizeof(buf));

	if (rv == WRITE_ONLY) { return PX4_OK; }
	if (rv != 0) { return rv; }

	_settings.min_frequency = int16_t(_rx_buf[2] | (_rx_buf[3] << 8));
	_settings.max_frequency = int16_t(_rx_buf[4] | (_rx_buf[5] << 8));
	_settings.max_power_mW = int16_t(_rx_buf[6] | (_rx_buf[7] << 8));

	return get_status();
}

int Tramp::set_power(int16_t power_level)
{
	int16_t power_mW;
	_requested_power_level = -1;

	if (power_level < 0) {
		power_mW = -power_level;

	} else {
		power_mW = vtxtable().power_value(power_level);

		if (power_mW == 0) { return -EINVAL; }

		_requested_power_level = power_level;
	}

	if (_settings.requested_power_mW == power_mW) { return PX4_OK; }

	const uint8_t buf[] = {COMMAND_SET_POWER, uint8_t(power_mW), uint8_t(power_mW >> 8)};
	int rv = transmit(buf, sizeof(buf));

	if (rv != PX4_OK) { return rv; }

	_settings.requested_power_mW = power_mW;
	_settings.power_mW = power_mW;

	if (_write_only) { return PX4_OK; }

	rv = get_status();

	if (rv != PX4_OK) { return rv; }

	return _settings.requested_power_mW == power_mW ? PX4_OK : PX4_ERROR;
}

int Tramp::set_frequency(int16_t frequency_MHz)
{
	if (frequency_MHz < 0) { return -EINVAL; } // Tramp does not support pit frequency setting
	if (_settings.frequency == frequency_MHz) { return PX4_OK; }

	const uint8_t buf[] = {COMMAND_SET_FREQUENCY, uint8_t(frequency_MHz), uint8_t(frequency_MHz >> 8)};
	int rv = transmit(buf, sizeof(buf));

	if (rv != PX4_OK) { return rv; }

	_settings.frequency = frequency_MHz;

	if (_write_only) { return PX4_OK; }

	rv = get_status();

	if (rv != PX4_OK) { return rv; }

	return _settings.frequency == frequency_MHz ? PX4_OK : PX4_ERROR;
}

int Tramp::set_pit_mode(bool onoff)
{
	if (bool(_settings.pit_mode) == onoff) { return PX4_OK; }

	const uint8_t mode = onoff ? 0u : 1u;
	const uint8_t buf[] = {COMMAND_SET_MODE, mode};
	int rv = transmit(buf, sizeof(buf));

	if (rv != PX4_OK) { return rv; }

	_settings.pit_mode = onoff;

	if (_write_only) { return PX4_OK; }

	rv = get_status();

	if (rv != PX4_OK) { return rv; }

	return bool(_settings.pit_mode) == onoff ? PX4_OK : PX4_ERROR;
}

int Tramp::transmit(const uint8_t *buf, size_t len)
{
	if (len > 28) { return -1; }
	const bool query = buf[0] & 0x20;

	if (_last_request_timestamp != 0) {
		const hrt_abstime elapsed = hrt_elapsed_time(&_last_request_timestamp);

		if (elapsed < MIN_REQUEST_INTERVAL_US) {
			px4_usleep(MIN_REQUEST_INTERVAL_US - elapsed);
		}
	}

	// ArduPilot discards stale input before issuing a query.
	// The 200 ms request interval guarantees that no previous frame remains in TX.
	if (query) {
		_serial->flush();
	}

	memset(_tx_buf + 1, 0, sizeof(_tx_buf) - 1);
	// copy the message data
	memcpy(_tx_buf + 1, buf, len);
	// compute the CRC
	_tx_buf[offsetof(Frame, crc)] = crc8(_tx_buf);

	// send command
	const ssize_t written = _serial->writeBlocking(_tx_buf, sizeof(Frame), 100);

	if (written < 0) {
		return -errno;
	}

	if (written != ssize_t(sizeof(Frame))) { return -EIO; }

	_last_request_timestamp = hrt_absolute_time();

	// Set commands do not have response frames.
	if (!query) {
		return PX4_OK;
	}

	const uint32_t echo_frames_before = _echo_frames;
	const uint32_t valid_frames_before = _valid_frames;
	const int rv = rx_msg();
	_last_query_error = rv;

	if (rv == PX4_OK) {
		_query_failures = 0;
		return PX4_OK;
	}

	if (_echo_frames > echo_frames_before && _valid_frames == valid_frames_before) {
		_write_only = true;
		_query_failures = 0;
		return WRITE_ONLY;
	}

	if (++_query_failures >= SMARTBAUD_FAILURES) {
		const int previous_baud = _smartbaud;

		if (_smartbaud_direction > 0 && _smartbaud == SMARTBAUD_MAX) {
			_smartbaud_direction = -1;

		} else if (_smartbaud_direction < 0 && _smartbaud == SMARTBAUD_MIN) {
			_smartbaud_direction = 1;
		}

		_smartbaud += SMARTBAUD_STEP * _smartbaud_direction;
		_serial->flush();

		if (!_serial->setBaudrate(_smartbaud)) { return -EIO; }

		PX4_DEBUG("Tramp query error %d, baud %d -> %d", rv, previous_baud, _smartbaud);
		_query_failures = 0;
	}

	return rv;
}

int Tramp::rx_parser(uint8_t c)
{
	_rx_bytes++;

	enum {
		SYNC = 0,
		COMMAND = 1,
		DATA = 2,
		CRC = 3,
		END = 4,
	};
	static constexpr uint8_t CRCPOS{offsetof(Frame, crc)};

	switch (_read_state) {
	case SYNC:
		PX4_DEBUG("SYNC %x", c);

		if (c == 0x0F) {
			_rx_buf[0] = c;
			_read_state = COMMAND;
			return 1;

		} else {
			_read_state = SYNC;
			return 1;
		}

	case COMMAND:
		PX4_DEBUG("COMMAND %x", c);

		switch (c) {
		case COMMAND_RESET:
		case COMMAND_GET_TEMPERATURE:
		case COMMAND_GET_SETTINGS:
			_rx_buf[1] = c;
			_read_state = DATA;
			_read_data_length = 2;
			return CRCPOS;
		}

		_read_state = SYNC;
		return 1;

	case DATA:
		PX4_DEBUG("DATA %x", c);
		_rx_buf[_read_data_length] = c;

		if (++_read_data_length >= CRCPOS) {
			_read_state = CRC;
		}

		return sizeof(Frame) - _read_data_length;

	case CRC:
		PX4_DEBUG("CRC %x", c);
		_read_state = END;
		_rx_buf[CRCPOS] = c;
		return 1;

	case END:
		PX4_DEBUG("END %x", c);
		_read_state = SYNC;
		_rx_buf[sizeof(Frame) - 1] = c;

		if (c != 0 || _rx_buf[CRCPOS] != crc8(_rx_buf) || _rx_buf[1] != _tx_buf[1]) {
			_invalid_frames++;
			return 1;
		}

		// Lowercase query frames may be echoed on a bidirectional UART. A real
		// response has a populated value in the command-specific payload.
		if (((_rx_buf[1] == COMMAND_RESET || _rx_buf[1] == COMMAND_GET_SETTINGS)
		     && !_rx_buf[2] && !_rx_buf[3])
		    || (_rx_buf[1] == COMMAND_GET_TEMPERATURE && !_rx_buf[6] && !_rx_buf[7])) {
			_echo_frames++;
			return 1;
		}

		_valid_frames++;
		return 0;
	}

	return -6000;
}

uint8_t Tramp::crc8(const uint8_t *data)
{
	uint8_t crc{0};

	for (uint_fast8_t ii{1}; ii < offsetof(Frame, crc); ii++) {
		crc += data[ii];
	}

	return crc;
}

}
