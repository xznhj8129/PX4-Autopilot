/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

/**
 * @file atxxxx.cpp
 * @author Daniele Pettenuzzo
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * Driver for the ATXXXX chip (e.g. MAX7456) on the omnibus f4 fcu connected via SPI.
 */

#include "atxxxx.h"
#include "symbols.h"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{50_ms};	// 20 Hz
static constexpr int OSD_WARNING_LEVEL{3};

OSDatxxxx::OSDatxxxx(const I2CSPIDriverConfig &config) :
	SPI(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int
OSDatxxxx::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = reset();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = init_osd();

	if (ret != PX4_OK) {
		return ret;
	}

	// clear the screen
	int num_rows = (_param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL);

	for (int i = 0; i < OSD_CHARS_PER_ROW; i++) {
		for (int j = 0; j < num_rows; j++) {
			add_character_to_screen(' ', i, j);
		}
	}

	if (ret == PX4_OK) {
		start();
	}

	return ret;
}

int
OSDatxxxx::start()
{
	ScheduleOnInterval(OSD_UPDATE_RATE, 10000);

	return PX4_OK;
}

int
OSDatxxxx::probe()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= writeRegister(0x00, 0x01); //disable video output
	ret |= readRegister(0x00, &data, 1);

	if (data != 1 || ret != PX4_OK) {
		PX4_ERR("probe failed (%i %i)", ret, data);
	}

	return ret;
}

int
OSDatxxxx::init_osd()
{
	int ret = PX4_OK;
	uint8_t data = OSD_ZERO_BYTE;

	if (_param_osd_atxxxx_cfg.get() == 2) {
		data |= OSD_PAL_TX_MODE;
	}

	ret |= writeRegister(0x00, data);
	ret |= writeRegister(0x04, OSD_ZERO_BYTE);

	enable_screen();

	return ret;
}

int
OSDatxxxx::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5] {}; // read up to 4 bytes

	cmd[0] = DIR_READ(reg);

	int ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (ret != PX4_OK) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;
}

int
OSDatxxxx::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2] {}; // write 1 byte

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	int ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;
}

int
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb = 0;
	int ret = PX4_ERROR;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		ret = writeRegister(0x05, 0x01); //DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		ret = writeRegister(0x05, 0x00); //DMAH
	}

	if (ret != 0) {
		return ret;
	}

	ret = writeRegister(0x06, position_lsb); //DMAL

	if (ret != 0) {
		return ret;
	}

	ret = writeRegister(0x07, c);

	return ret;
}

int
OSDatxxxx::add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y, int width)
{
	int i = 0;
	int ret = PX4_OK;

	for (; i < width && str[i] != '\0'; ++i) {
		ret |= add_character_to_screen(str[i], pos_x + i, pos_y);
	}

	for (; i < width; ++i) {
		ret |= add_character_to_screen(' ', pos_x + i, pos_y);
	}

	return ret;
}

void
OSDatxxxx::add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length)
{
	int len = strlen(str);

	if (len > max_length) {
		len = max_length;
	}

	int pos = (OSD_CHARS_PER_ROW - max_length) / 2;
	int before = (max_length - len) / 2;

	for (int i = 0; i < before; ++i) {
		add_character_to_screen(' ', pos++, pos_y);
	}

	for (int i = 0; i < len; ++i) {
		add_character_to_screen(str[i], pos++, pos_y);
	}

	while (pos < (OSD_CHARS_PER_ROW + max_length) / 2) {
		add_character_to_screen(' ', pos++, pos_y);
	}
}

void
OSDatxxxx::clear_line(uint8_t pos_x, uint8_t pos_y, int length)
{
	for (int i = 0; i < length; ++i) {
		add_character_to_screen(' ', pos_x + i, pos_y);
	}
}

int
OSDatxxxx::add_battery_info(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	char batt_symbol = OSD_SYMBOL_BATT_EMPTY;

	if (battery.remaining >= 0.875f) {
		batt_symbol = OSD_SYMBOL_BATT_FULL;

	} else if (battery.remaining >= 0.625f) {
		batt_symbol = OSD_SYMBOL_BATT_5;

	} else if (battery.remaining >= 0.375f) {
		batt_symbol = OSD_SYMBOL_BATT_4;

	} else if (battery.remaining >= 0.25f) {
		batt_symbol = OSD_SYMBOL_BATT_3;

	} else if (battery.remaining >= 0.125f) {
		batt_symbol = OSD_SYMBOL_BATT_2;

	} else if (battery.remaining >= 0.f) {
		batt_symbol = OSD_SYMBOL_BATT_1;
	}

	snprintf(buf, sizeof(buf), "%c%5.2f", batt_symbol, (double)battery.voltage_v);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;
	pos_x++;

	snprintf(buf, sizeof(buf), "%5d", (int)battery.discharged_mah);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen(OSD_SYMBOL_MAH, pos_x + 5, pos_y);

	return ret;
}

int
OSDatxxxx::add_altitude(const vehicle_local_position_s &local_position, uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];
	int ret = PX4_OK;

	snprintf(buf, sizeof(buf), "%c%5.1f%c", OSD_SYMBOL_ARROW_NORTH, (double) - local_position.z, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	snprintf(buf, sizeof(buf), "%c%5.1f", OSD_SYMBOL_FLIGHT_TIME, (double)flight_time);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::enable_screen()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data | 0x48);

	return ret;
}

int
OSDatxxxx::disable_screen()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data & 0xF7);

	return ret;
}

int
OSDatxxxx::update_screen()
{
	int ret = PX4_OK;
	const osd::TelemetryData &telemetry = _telemetry.data();
	const int num_rows = _param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL;
	char buf[16] {};

	if (telemetry.battery_valid) {
		ret |= add_battery_info(telemetry.battery, 1, 1);

	} else {
		clear_line(1, 1, 10);
		clear_line(1, 2, 10);
	}

	if (telemetry.local_position.z_valid) {
		ret |= add_altitude(telemetry.local_position, 1, 3);

	} else {
		clear_line(1, 3, 10);
	}

	if (telemetry.input_rc.link_quality >= 0) {
		snprintf(buf, sizeof(buf), "%c%3d%%", OSD_SYMBOL_RSSI, telemetry.input_rc.link_quality);
		ret |= add_string_to_screen(buf, 23, 1, 6);

	} else {
		clear_line(23, 1, 6);
	}

	if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
		snprintf(buf, sizeof(buf), "%c%c%2d %3.0f", OSD_SYMBOL_SAT_L, OSD_SYMBOL_SAT_R,
			 telemetry.gps.satellites_used, (double)telemetry.gps.vel_m_s);
		ret |= add_string_to_screen(buf, 20, 2, 9);

	} else {
		ret |= add_string_to_screen("NO GPS", 23, 2, 6);
	}

	if (telemetry.home_valid && telemetry.attitude_valid) {
		const float relative_bearing = matrix::wrap_pi(telemetry.home_bearing_rad - telemetry.yaw_rad);
		const int arrow_index = (8 + static_cast<int>(lroundf(relative_bearing * 8.f / M_PI_F)) + 16) % 16;
		snprintf(buf, sizeof(buf), "%c%4.0f%c", OSD_SYMBOL_ARROW_SOUTH + arrow_index,
			 (double)telemetry.home_distance_m, OSD_SYMBOL_M);
		ret |= add_string_to_screen(buf, 21, 3, 8);

	} else {
		clear_line(21, 3, 8);
	}

	const int horizon_y = num_rows == OSD_NUM_ROWS_NTSC ? 6 : 7;

	for (int y = horizon_y - 2; y <= horizon_y + 2; ++y) {
		clear_line(10, y, 11);
	}

	if (telemetry.attitude_valid) {
		const float roll = math::constrain(telemetry.roll_rad, -1.2f, 1.2f);
		const float pitch = math::constrain(telemetry.pitch_rad, -0.7f, 0.7f);

		for (int x = -4; x <= 4; ++x) {
			const int subrow = lroundf(pitch * 18.f + tanf(roll) * x * 5.f);
			const int row_offset = floorf(static_cast<float>(subrow) / 9.f);
			const int glyph_offset = subrow - row_offset * 9;

			if (row_offset >= -2 && row_offset <= 2) {
				ret |= add_character_to_screen(OSD_SYMBOL_AH_BAR9_0 + glyph_offset, 15 + x, horizon_y + row_offset);
			}
		}
	}

	ret |= add_character_to_screen(OSD_SYMBOL_AH_LEFT, 13, horizon_y);
	ret |= add_character_to_screen(OSD_SYMBOL_AH_CENTER, 15, horizon_y);
	ret |= add_character_to_screen(OSD_SYMBOL_AH_RIGHT, 17, horizon_y);

	const char *flight_mode = _telemetry.flight_mode();

	if (telemetry.status.timestamp == 0) {
		flight_mode = "NO STATUS";
	}

	add_string_to_screen_centered(flight_mode, num_rows - 2, 14);
	ret |= add_flighttime(_telemetry.flight_time_s(), 1, num_rows - 1);

	_telemetry.update_message_display(OSD_WARNING_LEVEL, _display);
	char message[FULL_MSG_BUFFER] {};
	_display.get(message, hrt_absolute_time());
	ret |= add_string_to_screen(message, 17, num_rows - 1, FULL_MSG_LENGTH);

	return ret;
}

int
OSDatxxxx::reset()
{
	int ret = writeRegister(0x00, 0x02);
	usleep(100);

	return ret;
}

void
OSDatxxxx::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	_telemetry.update();
	update_screen();
}

void
OSDatxxxx::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.

It can be enabled with the OSD_ATXXXX_CFG parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("atxxxx", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
atxxxx_main(int argc, char *argv[])
{
	using ThisDriver = OSDatxxxx;
	BusCLIArguments cli{false, true};
	cli.spi_mode = SPIDEV_MODE0;
	cli.default_spi_frequency = OSD_SPI_BUS_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_OSD_DEVTYPE_ATXXXX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
