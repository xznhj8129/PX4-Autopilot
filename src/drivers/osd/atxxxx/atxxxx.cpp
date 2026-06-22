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

#include <ctype.h>

using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{50_ms};	// 20 Hz

OSDatxxxx::OSDatxxxx(const I2CSPIDriverConfig &config) :
	SPI(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);
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

	memset(_screen, ' ', sizeof(_screen));
	memset(_displayed_screen, 0xff, sizeof(_displayed_screen));
	ret = flush_screen();

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
OSDatxxxx::write_character_to_screen(uint8_t c, uint8_t pos_x, uint8_t pos_y)
{
	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb = 0;
	int ret = PX4_ERROR;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position & 0xff);
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
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	if (pos_x >= OSD_CHARS_PER_ROW || pos_y >= OSD_NUM_ROWS_PAL) {
		return PX4_ERROR;
	}

	_screen[OSD_CHARS_PER_ROW * pos_y + pos_x] = static_cast<uint8_t>(c);
	return PX4_OK;
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
OSDatxxxx::clear_line(uint8_t pos_x, uint8_t pos_y, int length)
{
	for (int i = 0; i < length; ++i) {
		add_character_to_screen(' ', pos_x + i, pos_y);
	}
}

int
OSDatxxxx::flush_screen()
{
	int ret = PX4_OK;
	const int num_rows = _param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL;

	for (int y = 0; y < num_rows; ++y) {
		for (int x = 0; x < OSD_CHARS_PER_ROW; ++x) {
			const int position = OSD_CHARS_PER_ROW * y + x;

			if (_screen[position] != _displayed_screen[position]) {
				const int write_ret = write_character_to_screen(_screen[position], x, y);
				ret |= write_ret;

				if (write_ret == PX4_OK) {
					_displayed_screen[position] = _screen[position];
				}
			}
		}
	}

	return ret;
}

int
OSDatxxxx::add_battery_voltage(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
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

	buf[6] = 'V';
	buf[7] = '\0';

	return add_string_to_screen(buf, pos_x, pos_y, 7);
}

int
OSDatxxxx::add_consumed_mah(const battery_status_s &battery, uint8_t pos_x, uint8_t pos_y)
{
	char buf[7];

	snprintf(buf, sizeof(buf), "%5d", (int)battery.discharged_mah);
	buf[5] = OSD_SYMBOL_MAH;
	buf[6] = '\0';

	return add_string_to_screen(buf, pos_x, pos_y, 6);
}

int
OSDatxxxx::add_altitude(const vehicle_local_position_s &local_position, uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%c%5.1f%c", OSD_SYMBOL_ARROW_NORTH, (double) - local_position.z, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	return add_string_to_screen(buf, pos_x, pos_y, 9);
}

int
OSDatxxxx::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];

	snprintf(buf, sizeof(buf), "%c%5.1f", OSD_SYMBOL_FLIGHT_TIME, (double)flight_time);
	buf[sizeof(buf) - 1] = '\0';

	return add_string_to_screen(buf, pos_x, pos_y, 7);
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
	const int horizon_x = _param_osd_ah_x.get();
	const int horizon_y = _param_osd_ah_y.get();
	memset(_screen, ' ', sizeof(_screen));

	if (enabled(osd::Symbol::ArtificialHorizon)) {
		if (telemetry.attitude_valid) {
			float roll = -0.5f * matrix::wrap_pi(telemetry.roll_rad);

			if (roll > M_PI_2_F) {
				roll -= M_PI_F;

			} else if (roll < -M_PI_2_F) {
				roll += M_PI_F;
			}

			const float camera_pitch = telemetry.pitch_rad + math::radians(static_cast<float>(_param_osd_cam_upt.get()));
			const float half_vertical_fov = math::radians(static_cast<float>(_param_osd_cam_vfov.get())) * 0.5f;
			const float half_horizontal_fov = math::radians(static_cast<float>(_param_osd_cam_hfov.get())) * 0.5f;

			if (fabsf(camera_pitch) < M_PI_2_F) {
				const float screen_height_subrows = num_rows * 9.f;
				const float pitch_subrows = tanf(camera_pitch) / tanf(half_vertical_fov) * screen_height_subrows * 0.5f;
				const float subrows_per_column = screen_height_subrows / OSD_CHARS_PER_ROW *
								 tanf(half_horizontal_fov) / tanf(half_vertical_fov);
				const float sin_roll = sinf(roll);
				const float cos_roll = cosf(roll);

				if (fabsf(cos_roll) >= fabsf(sin_roll)) {
					for (int x = -4; x <= 4; ++x) {
						const int subrow = lroundf(pitch_subrows + sin_roll / cos_roll * x * subrows_per_column);
						const int row_offset = floorf(static_cast<float>(subrow) / 9.f);
						const int glyph_offset = subrow - row_offset * 9;

						if (horizon_y + row_offset >= 0 && horizon_y + row_offset < num_rows) {
							ret |= add_character_to_screen(OSD_SYMBOL_AH_BAR9_0 + glyph_offset, horizon_x + x,
										       horizon_y + row_offset);
						}
					}

				} else {
					for (int y = -horizon_y; y < num_rows - horizon_y; ++y) {
						const int x = lroundf((y * 9.f - pitch_subrows) * cos_roll /
								      (sin_roll * subrows_per_column));

						if (x >= -4 && x <= 4) {
							ret |= add_character_to_screen('|', horizon_x + x, horizon_y + y);
						}
					}
				}
			}
		}
	}

	if (enabled(osd::Symbol::Crosshairs)) {
		const int crosshair_x = _param_osd_cross_x.get();
		const int crosshair_y = _param_osd_cross_y.get();
		ret |= add_character_to_screen(OSD_SYMBOL_AH_CENTER, crosshair_x, crosshair_y);
	}

	if (enabled(osd::Symbol::MainBatteryVoltage)) {
		if (telemetry.battery_valid) {
			ret |= add_battery_voltage(telemetry.battery, _param_osd_bat_volt_x.get(), _param_osd_bat_volt_y.get());

		} else {
			clear_line(_param_osd_bat_volt_x.get(), _param_osd_bat_volt_y.get(), 7);
		}
	}

	if (enabled(osd::Symbol::MahDrawn)) {
		if (telemetry.battery_valid) {
			ret |= add_consumed_mah(telemetry.battery, _param_osd_mah_x.get(), _param_osd_mah_y.get());

		} else {
			clear_line(_param_osd_mah_x.get(), _param_osd_mah_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::AverageCellVoltage)) {
		if (telemetry.battery_valid && telemetry.battery.cell_count > 0) {
			snprintf(buf, sizeof(buf), "C%4.2fV", (double)(telemetry.battery.voltage_v / telemetry.battery.cell_count));
			ret |= add_string_to_screen(buf, _param_osd_cell_v_x.get(), _param_osd_cell_v_y.get(), 7);

		} else {
			clear_line(_param_osd_cell_v_x.get(), _param_osd_cell_v_y.get(), 7);
		}
	}

	if (enabled(osd::Symbol::CurrentDraw)) {
		if (telemetry.battery_valid && PX4_ISFINITE(telemetry.battery.current_a)) {
			snprintf(buf, sizeof(buf), "%c%4.1fA", OSD_SYMBOL_AMP, (double)telemetry.battery.current_a);
			ret |= add_string_to_screen(buf, _param_osd_current_x.get(), _param_osd_current_y.get(), 6);

		} else {
			clear_line(_param_osd_current_x.get(), _param_osd_current_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::Power)) {
		if (telemetry.battery_valid && PX4_ISFINITE(telemetry.battery.current_a)) {
			snprintf(buf, sizeof(buf), "%c%4.0fW", OSD_SYMBOL_WATT,
				 (double)(telemetry.battery.voltage_v * telemetry.battery.current_a));
			ret |= add_string_to_screen(buf, _param_osd_power_x.get(), _param_osd_power_y.get(), 6);

		} else {
			clear_line(_param_osd_power_x.get(), _param_osd_power_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::Rssi)) {
		if (telemetry.input_rc.link_quality >= 0) {
			snprintf(buf, sizeof(buf), "%c%3d%%", OSD_SYMBOL_RSSI, telemetry.input_rc.link_quality);
			ret |= add_string_to_screen(buf, _param_osd_rssi_x.get(), _param_osd_rssi_y.get(), 6);

		} else {
			clear_line(_param_osd_rssi_x.get(), _param_osd_rssi_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::GpsSatellites)) {
		if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
			snprintf(buf, sizeof(buf), "%c%c%2d", OSD_SYMBOL_SAT_L, OSD_SYMBOL_SAT_R,
				 telemetry.gps.satellites_used);
			ret |= add_string_to_screen(buf, _param_osd_gps_sat_x.get(), _param_osd_gps_sat_y.get(), 4);

		} else {
			ret |= add_string_to_screen("GPS?", _param_osd_gps_sat_x.get(), _param_osd_gps_sat_y.get(), 4);
		}

	}

	if (enabled(osd::Symbol::GpsSpeed)) {
		if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
			snprintf(buf, sizeof(buf), "%3.0f", (double)telemetry.gps.vel_m_s);
			ret |= add_string_to_screen(buf, _param_osd_gps_spd_x.get(), _param_osd_gps_spd_y.get(), 4);

		} else {
			clear_line(_param_osd_gps_spd_x.get(), _param_osd_gps_spd_y.get(), 4);
		}
	}

	if (enabled(osd::Symbol::GpsLatitude)) {
		if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
			snprintf(buf, sizeof(buf), "LAT%+.5f", telemetry.gps.latitude_deg);
			ret |= add_string_to_screen(buf, _param_osd_gps_lat_x.get(), _param_osd_gps_lat_y.get(), 13);

		} else {
			clear_line(_param_osd_gps_lat_x.get(), _param_osd_gps_lat_y.get(), 13);
		}
	}

	if (enabled(osd::Symbol::GpsLongitude)) {
		if (telemetry.gps.fix_type >= sensor_gps_s::FIX_TYPE_2D) {
			snprintf(buf, sizeof(buf), "LON%+.5f", telemetry.gps.longitude_deg);
			ret |= add_string_to_screen(buf, _param_osd_gps_lon_x.get(), _param_osd_gps_lon_y.get(), 14);

		} else {
			clear_line(_param_osd_gps_lon_x.get(), _param_osd_gps_lon_y.get(), 14);
		}
	}

	if (enabled(osd::Symbol::Altitude)) {
		if (telemetry.local_position.z_valid) {
			ret |= add_altitude(telemetry.local_position, _param_osd_alt_x.get(), _param_osd_alt_y.get());

		} else {
			clear_line(_param_osd_alt_x.get(), _param_osd_alt_y.get(), 9);
		}
	}

	if (enabled(osd::Symbol::NumericalVario)) {
		if (telemetry.local_position.v_z_valid) {
			snprintf(buf, sizeof(buf), "V%+4.1f", (double) - telemetry.local_position.vz);
			ret |= add_string_to_screen(buf, _param_osd_vario_x.get(), _param_osd_vario_y.get(), 7);

		} else {
			clear_line(_param_osd_vario_x.get(), _param_osd_vario_y.get(), 7);
		}
	}

	if (enabled(osd::Symbol::PitchAngle)) {
		if (telemetry.attitude_valid) {
			snprintf(buf, sizeof(buf), "P%+4.0f", (double)math::degrees(telemetry.pitch_rad));
			ret |= add_string_to_screen(buf, _param_osd_pitch_x.get(), _param_osd_pitch_y.get(), 6);

		} else {
			clear_line(_param_osd_pitch_x.get(), _param_osd_pitch_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::RollAngle)) {
		if (telemetry.attitude_valid) {
			snprintf(buf, sizeof(buf), "R%+4.0f", (double)math::degrees(telemetry.roll_rad));
			ret |= add_string_to_screen(buf, _param_osd_roll_x.get(), _param_osd_roll_y.get(), 6);

		} else {
			clear_line(_param_osd_roll_x.get(), _param_osd_roll_y.get(), 6);
		}
	}

	if (enabled(osd::Symbol::HomeDirection)) {
		if (telemetry.home_valid && telemetry.attitude_valid) {
			const float relative_bearing = matrix::wrap_pi(telemetry.home_bearing_rad - telemetry.yaw_rad);
			const int arrow_index = (8 + static_cast<int>(lroundf(relative_bearing * 8.f / M_PI_F)) + 16) % 16;
			ret |= add_character_to_screen(OSD_SYMBOL_ARROW_SOUTH + arrow_index,
						       _param_osd_home_dir_x.get(), _param_osd_home_dir_y.get());

		} else {
			clear_line(_param_osd_home_dir_x.get(), _param_osd_home_dir_y.get(), 1);
		}
	}

	if (enabled(osd::Symbol::HomeDistance)) {
		if (telemetry.home_valid) {
			snprintf(buf, sizeof(buf), "%4.0f%c", (double)telemetry.home_distance_m, OSD_SYMBOL_M);
			ret |= add_string_to_screen(buf, _param_osd_home_dst_x.get(), _param_osd_home_dst_y.get(), 6);

		} else {
			clear_line(_param_osd_home_dst_x.get(), _param_osd_home_dst_y.get(), 6);
		}
	}

	const char *flight_mode = _telemetry.flight_mode();

	if (telemetry.status.timestamp == 0) {
		flight_mode = "NO STATUS";
	}

	if (enabled(osd::Symbol::FlightMode)) {
		strncpy(buf, flight_mode, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = '\0';

		for (int i = 0; buf[i] != '\0'; ++i) {
			buf[i] = toupper(static_cast<unsigned char>(buf[i]));
		}

		ret |= add_string_to_screen(buf, _param_osd_mode_x.get(), _param_osd_mode_y.get(), 14);
	}

	if (enabled(osd::Symbol::Disarmed)) {
		if (telemetry.status.timestamp != 0) {
			const char *arming_state = telemetry.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
						   ? "ARMED" : "DISARMED";
			ret |= add_string_to_screen(arming_state, _param_osd_arm_x.get(), _param_osd_arm_y.get(), 8);

		} else {
			clear_line(_param_osd_arm_x.get(), _param_osd_arm_y.get(), 8);
		}
	}

	if (enabled(osd::Symbol::Heading)) {
		if (telemetry.attitude_valid) {
			const int heading_deg = static_cast<int>(lroundf(math::degrees(telemetry.yaw_rad))) % 360;
			snprintf(buf, sizeof(buf), "HDG%03d", heading_deg);
			ret |= add_string_to_screen(buf, _param_osd_head_x.get(), _param_osd_head_y.get(), 6);

		} else {
			clear_line(_param_osd_head_x.get(), _param_osd_head_y.get(), 6);
		}
	}

#if defined(CONFIG_DRIVERS_VTX)
	if (enabled(osd::Symbol::VtxInfo)) {
		if (telemetry.vtx.timestamp != 0 && telemetry.vtx.band >= 0 && telemetry.vtx.channel >= 0 &&
		    telemetry.vtx.power_level >= 0 && telemetry.vtx.band_letter != 0) {
			snprintf(buf, sizeof(buf), "VTX %c:%d:%d", telemetry.vtx.band_letter, telemetry.vtx.channel + 1,
				 telemetry.vtx.power_level + 1);
			ret |= add_string_to_screen(buf, _param_osd_vtx_info_x.get(), _param_osd_vtx_info_y.get(), 11);

		} else {
			clear_line(_param_osd_vtx_info_x.get(), _param_osd_vtx_info_y.get(), 11);
		}
	}

	if (enabled(osd::Symbol::VtxFrequency)) {
		if (telemetry.vtx.timestamp != 0 && telemetry.vtx.frequency > 0) {
			snprintf(buf, sizeof(buf), "VTF: %huM", telemetry.vtx.frequency);
			ret |= add_string_to_screen(buf, _param_osd_vtx_freq_x.get(), _param_osd_vtx_freq_y.get(), 11);

		} else {
			clear_line(_param_osd_vtx_freq_x.get(), _param_osd_vtx_freq_y.get(), 11);
		}
	}

	if (enabled(osd::Symbol::VtxPower)) {
		if (telemetry.vtx.timestamp != 0 && telemetry.vtx.power > 0) {
			snprintf(buf, sizeof(buf), "VTW: %hiMW", telemetry.vtx.power);
			ret |= add_string_to_screen(buf, _param_osd_vtx_power_x.get(), _param_osd_vtx_power_y.get(), 12);

		} else {
			clear_line(_param_osd_vtx_power_x.get(), _param_osd_vtx_power_y.get(), 12);
		}
	}
#endif

	if (enabled(osd::Symbol::FlightTime)) {
		ret |= add_flighttime(_telemetry.flight_time_s(), _param_osd_ftime_x.get(), _param_osd_ftime_y.get());
	}

	_telemetry.update_message_display(_param_osd_log_level.get(), _display);
	char message[FULL_MSG_BUFFER] {};
	_display.get(message, hrt_absolute_time());

	if (enabled(osd::Symbol::StatusMessage)) {
		ret |= add_string_to_screen(message, _param_osd_status_x.get(), _param_osd_status_y.get(), FULL_MSG_LENGTH);
	}

	ret |= flush_screen();
	return ret;
}

bool
OSDatxxxx::enabled(osd::Symbol symbol) const
{
	return _param_osd_symbols.get() & (1u << static_cast<uint8_t>(symbol));
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

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update{};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
		_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);

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
