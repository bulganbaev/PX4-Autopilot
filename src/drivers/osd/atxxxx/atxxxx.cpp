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

#include <cmath>
#include <cstring>

using namespace time_literals;

static constexpr uint32_t OSD_UPDATE_RATE{50_ms};	// 20 Hz
static constexpr hrt_abstime OSD_SENSOR_TIMEOUT{500_ms};

static float wrap_360_deg(float deg)
{
	while (deg < 0.f) {
		deg += 360.f;
	}

	while (deg >= 360.f) {
		deg -= 360.f;
	}

	return deg;
}

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
	const int num_rows = (_param_osd_atxxxx_cfg.get() == 1 ? OSD_NUM_ROWS_NTSC : OSD_NUM_ROWS_PAL);

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

	ret |= writeRegister(0x00, 0x01); // disable video output
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
	uint8_t cmd[5]{}; // read up to 4 bytes
	cmd[0] = DIR_READ(reg);

	const int ret = transfer(&cmd[0], &cmd[0], count + 1);

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
	uint8_t cmd[2]{}; // write 1 byte
	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	const int ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;
}

int
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{
	const uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb = 0;
	int ret = PX4_ERROR;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		ret = writeRegister(0x05, 0x01); // DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		ret = writeRegister(0x05, 0x00); // DMAH
	}

	if (ret != PX4_OK) {
		return ret;
	}

	ret = writeRegister(0x06, position_lsb); // DMAL

	if (ret != PX4_OK) {
		return ret;
	}

	ret = writeRegister(0x07, c);
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
	const int before = (max_length - len) / 2;

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
OSDatxxxx::add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y)
{
	for (int i = 0; str[i] != '\0'; ++i) {
		add_character_to_screen(str[i], pos_x + i, pos_y);
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
OSDatxxxx::add_battery_info(uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	// TODO: show battery symbol based on battery fill level
	snprintf(buf, sizeof(buf), "%c%5.2f", OSD_SYMBOL_BATT_3, (double)_battery_voltage_v);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;
	pos_x++;

	snprintf(buf, sizeof(buf), "%5d", (int)_battery_discharge_mah);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen(OSD_SYMBOL_MAH, pos_x + 5, pos_y);
	return ret;
}

int
OSDatxxxx::add_battery_extra(uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];
	int ret = PX4_OK;

	if (!_battery_valid) {
		clear_line(pos_x, pos_y, 12);
		return ret;
	}

	if (_battery_current_a >= 0.f && _battery_remaining >= 0.f) {
		snprintf(buf, sizeof(buf), "%4.1fA %2.0f%%",
			 (double)_battery_current_a,
			 (double)(_battery_remaining * 100.f));

	} else if (_battery_current_a >= 0.f) {
		snprintf(buf, sizeof(buf), "%4.1fA", (double)_battery_current_a);

	} else if (_battery_remaining >= 0.f) {
		snprintf(buf, sizeof(buf), "%2.0f%%", (double)(_battery_remaining * 100.f));

	} else {
		snprintf(buf, sizeof(buf), " ");
	}

	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; ++i) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::add_altitude(uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];
	int ret = PX4_OK;

	snprintf(buf, sizeof(buf), "%c%5.2f%c", OSD_SYMBOL_ARROW_NORTH, (double)_local_position_z, OSD_SYMBOL_M);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; ++i) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::add_heading(uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];
	int ret = PX4_OK;

	if (!_heading_valid) {
		clear_line(pos_x, pos_y, 8);
		return ret;
	}

	snprintf(buf, sizeof(buf), "HDG%03d", (int)(_heading_deg + 0.5f));
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; ++i) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::add_groundspeed(uint8_t pos_x, uint8_t pos_y)
{
	char buf[12];
	int ret = PX4_OK;

	if (!_groundspeed_valid) {
		clear_line(pos_x, pos_y, 8);
		return ret;
	}

	snprintf(buf, sizeof(buf), "GS %4.1f", (double)_groundspeed_m_s);
	buf[sizeof(buf) - 1] = '\0';

	for (int i = 0; buf[i] != '\0'; ++i) {
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

	for (int i = 0; buf[i] != '\0'; ++i) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

const char *
OSDatxxxx::get_warning_text() const
{
	if (_motor_failure) {
		return "MOTOR FAIL";
	}

	if (_battery_warning >= battery_status_s::WARNING_EMERGENCY) {
		return "LAND NOW";
	}

	if (_battery_warning == battery_status_s::WARNING_CRITICAL) {
		return "CRIT BAT";
	}

	if (_battery_warning == battery_status_s::WARNING_LOW) {
		return "LOW BAT";
	}

	if (_battery_unhealthy) {
		return "BAT UNHEALTHY";
	}

	if (_manual_control_lost) {
		return "RC LOST";
	}

	if (_offboard_lost && _nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		return "OFFBOARD LOST";
	}

	if (_home_invalid &&
	    _arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
	    (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND ||
	     _nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET)) {
		return "NO HOME";
	}

	if (_global_position_invalid) {
		return "GLOBAL POS LOST";
	}

	if (_local_position_invalid) {
		return "LOCAL POS LOST";
	}

	if (_gcs_lost) {
		return "GCS LOST";
	}

	if (_geofence_breached) {
		return "GEOFENCE";
	}

	if (_failsafe_active) {
		return "FAILSAFE";
	}

	return "";
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
OSDatxxxx::update_topics()
{
	/* update battery subscription */
	if (_battery_sub.updated()) {
		battery_status_s battery{};
		_battery_sub.copy(&battery);

		if (battery.connected) {
			_battery_voltage_v = battery.voltage_v;
			_battery_discharge_mah = battery.discharged_mah;
			_battery_current_a = std::isfinite((double)battery.current_a) ? battery.current_a : -1.f;
			_battery_remaining = std::isfinite((double)battery.remaining) ? battery.remaining : -1.f;
			_battery_warning = battery.warning;
			_battery_valid = true;

		} else {
			_battery_valid = false;
			_battery_current_a = -1.f;
			_battery_remaining = -1.f;
			_battery_warning = battery_status_s::WARNING_NONE;
		}
	}

	/* update vehicle local position subscription */
	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		_local_position_valid = local_position.z_valid && std::isfinite((double)local_position.z);

		if (_local_position_valid) {
			_local_position_z = -local_position.z;
		}

		_heading_valid = std::isfinite((double)local_position.heading);

		if (_heading_valid) {
			_heading_deg = wrap_360_deg(local_position.heading * 57.2957795f);
		}

		_groundspeed_valid = local_position.v_xy_valid &&
			std::isfinite((double)local_position.vx) &&
			std::isfinite((double)local_position.vy);

		if (_groundspeed_valid) {
			_groundspeed_m_s = sqrtf(local_position.vx * local_position.vx +
						 local_position.vy * local_position.vy);
		}

		_vertical_speed_valid = local_position.v_z_valid && std::isfinite((double)local_position.vz);

		if (_vertical_speed_valid) {
			_vertical_speed_m_s = -local_position.vz; // up positive
		}

	}

	/* update vehicle acceleration subscription */
	if (_vehicle_acceleration_sub.updated()) {
		vehicle_acceleration_s accel{};
		_vehicle_acceleration_sub.copy(&accel);

		if (std::isfinite((double)accel.xyz[0]) &&
		    std::isfinite((double)accel.xyz[1]) &&
		    std::isfinite((double)accel.xyz[2])) {
			constexpr float g = 9.80665f;
			_g_force_h = sqrtf(accel.xyz[0] * accel.xyz[0] + accel.xyz[1] * accel.xyz[1]) / g;
			_g_force_v = fabsf(accel.xyz[2]) / g;
			_g_force_valid = true;

		} else {
			_g_force_valid = false;
		}
	}

	/* update FC temp (IMU temp proxy) */
	if (_sensor_accel_sub.updated()) {
		sensor_accel_s sensor_accel{};
		_sensor_accel_sub.copy(&sensor_accel);

		if (std::isfinite((double)sensor_accel.temperature)) {
			_fc_temp_c = sensor_accel.temperature;
			_fc_temp_valid = true;
		}
	}

	/* update ESC status: temperatures, rpm, power */
	if (_esc_status_sub.updated()) {
		esc_status_s esc_status{};
		_esc_status_sub.copy(&esc_status);

		float power_sum = 0.f;
		int power_count = 0;

		for (int i = 0; i < OSDatxxxx::MAX_ESC_DISPLAY; ++i) {
			_esc_temp_valid[i] = false;
			_esc_rpm_valid[i] = false;
			_esc_power_valid[i] = false;
		}

		for (int i = 0; i < esc_status.esc_count && i < OSDatxxxx::MAX_ESC_DISPLAY; ++i) {
			if (esc_status.esc_online_flags & (1u << i)) {
				if (std::isfinite((double)esc_status.esc[i].esc_temperature)) {
					_esc_temp_c[i] = esc_status.esc[i].esc_temperature;
					_esc_temp_valid[i] = true;
				}

				_esc_rpm[i] = esc_status.esc[i].esc_rpm;
				_esc_rpm_valid[i] = (_esc_rpm[i] > 0);

				if (std::isfinite((double)esc_status.esc[i].esc_power) && esc_status.esc[i].esc_power >= 0.f) {
					_esc_power[i] = esc_status.esc[i].esc_power;
					_esc_power_valid[i] = true;
					power_sum += _esc_power[i];
					power_count++;
				}
			}
		}

		if (power_count > 0) {
			float avg_power = power_sum / (float)power_count;

			if (avg_power < 0.f) { avg_power = 0.f; }
			if (avg_power > 100.f) { avg_power = 100.f; }

			_throttle_pct = (uint8_t)(avg_power + 0.5f);
			_throttle_valid = true;

		} else {
			_throttle_valid = false;
		}
	}

	/* update direct distance sensor / ToF */
	if (_distance_sensor_sub.updated()) {
		distance_sensor_s dist{};
		_distance_sensor_sub.copy(&dist);
		_distance_sensor_timestamp = dist.timestamp;

		if (std::isfinite((double)dist.current_distance) &&
		    dist.current_distance >= dist.min_distance &&
		    dist.current_distance <= dist.max_distance) {
			_tof_m = dist.current_distance;
			_tof_quality = dist.signal_quality;
			_tof_valid = true;
		} else {
			_tof_valid = false;
		}
	}

	/* update radio status */
	if (_radio_status_sub.updated()) {
		radio_status_s rs{};
		_radio_status_sub.copy(&rs);
		_rx_rssi = rs.rssi;
		_rx_remote_rssi = rs.remote_rssi;
		_rx_valid = true;
	}

	/* update manual throttle as fallback if ESC power is unavailable */
	/* cache manual throttle when updated */
	if (_manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual{};
		_manual_control_setpoint_sub.copy(&manual);

		if (manual.valid && std::isfinite((double)manual.throttle)) {
			_throttle_norm = math::constrain(manual.throttle, -1.f, 1.f);
			_manual_throttle_valid = true;
		}
	}

	/* fallback to cached manual throttle if ESC power is unavailable */
	if (!_throttle_valid && _manual_throttle_valid) {
		const float thr01 = (_throttle_norm + 1.f) * 0.5f;
		_throttle_pct = (uint8_t)(thr01 * 100.f + 0.5f);
		_throttle_valid = true;
	}

	/* update visual odometry */
	if (_vehicle_visual_odometry_sub.updated()) {
		vehicle_odometry_s vio{};
		_vehicle_visual_odometry_sub.copy(&vio);
		_vio_timestamp = vio.timestamp;

		if (std::isfinite((double)vio.position[0]) && std::isfinite((double)vio.position[1])) {
			_vio_x = vio.position[0];
			_vio_y = vio.position[1];
			_vio_valid = true;
		} else {
			_vio_valid = false;
		}
	}

	/* update failsafe flags */
	if (_failsafe_flags_sub.updated()) {
		failsafe_flags_s flags{};
		_failsafe_flags_sub.copy(&flags);
		_local_position_invalid = flags.local_position_invalid;
		_global_position_invalid = flags.global_position_invalid;
		_offboard_lost = flags.offboard_control_signal_lost;
		_home_invalid = flags.home_position_invalid;
		_manual_control_lost = flags.manual_control_signal_lost;
		_gcs_lost = flags.gcs_connection_lost;
		_battery_unhealthy = flags.battery_unhealthy;
		_motor_failure = flags.fd_motor_failure;
		_geofence_breached = flags.geofence_breached;
	}

	/* update vehicle status subscription */
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		    _arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			_arming_timestamp = hrt_absolute_time();
		}

		_arming_state = vehicle_status.arming_state;
		_nav_state = vehicle_status.nav_state;
		_failsafe_active = vehicle_status.failsafe;
	}

	const hrt_abstime now = hrt_absolute_time();
	_distance_sensor_present = (_distance_sensor_timestamp != 0) && ((now - _distance_sensor_timestamp) < OSD_SENSOR_TIMEOUT);
	_vio_present = (_vio_timestamp != 0) && ((now - _vio_timestamp) < OSD_SENSOR_TIMEOUT);

	if (!_distance_sensor_present) {
		_tof_valid = false;
	}

	if (!_vio_present) {
		_vio_valid = false;
	}

	return PX4_OK;
}

const char *
OSDatxxxx::get_flight_mode(uint8_t nav_state)
{
	const char *flight_mode = "UNKNOWN";

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		flight_mode = "MANUAL";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		flight_mode = "ALTITUDE";
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		flight_mode = "POSITION";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		flight_mode = "ORBIT";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		flight_mode = "VTOL_TO";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		flight_mode = "RETURN";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		flight_mode = "MISSION";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		flight_mode = "AUTO";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		flight_mode = "ACRO";
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		flight_mode = "TERMINATE";
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		flight_mode = "OFFBOARD";
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		flight_mode = "STABILIZED";
		break;
	}

	return flight_mode;
}

int
OSDatxxxx::update_screen()
{
	int ret = PX4_OK;
	char buf[24];

	// LEFT TOP: battery voltage
	if (_battery_valid) {
		snprintf(buf, sizeof(buf), "B%5.2f", (double)_battery_voltage_v);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 1);
	} else {
		clear_line(1, 1, 7);
	}

	// LEFT TOP: current + throttle
	if (_battery_valid || _throttle_valid) {
		if (_battery_current_a >= 0.f && _throttle_valid) {
			snprintf(buf, sizeof(buf), "C%3.1f T%03u", (double)_battery_current_a, (unsigned)_throttle_pct);
		} else if (_battery_current_a >= 0.f) {
			snprintf(buf, sizeof(buf), "C%3.1f", (double)_battery_current_a);
		} else if (_throttle_valid) {
			snprintf(buf, sizeof(buf), "T%03u", (unsigned)_throttle_pct);
		} else {
			buf[0] = '\0';
		}

		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 2);
	} else {
		clear_line(1, 2, 12);
	}

	// LEFT: ESC temperatures
	if (_esc_temp_valid[0] || _esc_temp_valid[1]) {
		snprintf(buf, sizeof(buf), "E1%02d E2%02d",
			 _esc_temp_valid[0] ? (int)(_esc_temp_c[0] + 0.5f) : 0,
			 _esc_temp_valid[1] ? (int)(_esc_temp_c[1] + 0.5f) : 0);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 3);
	} else {
		add_string_to_screen("E1-- E2--", 1, 3);
	}

	if (_esc_temp_valid[2] || _esc_temp_valid[3]) {
		snprintf(buf, sizeof(buf), "E3%02d E4%02d",
			 _esc_temp_valid[2] ? (int)(_esc_temp_c[2] + 0.5f) : 0,
			 _esc_temp_valid[3] ? (int)(_esc_temp_c[3] + 0.5f) : 0);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 4);
	} else {
		add_string_to_screen("E3-- E4--", 1, 4);
	}

	// LEFT: ESC RPM
	if (_esc_rpm_valid[0] || _esc_rpm_valid[1]) {
		snprintf(buf, sizeof(buf), "1:%04ld 2:%04ld",
			 _esc_rpm_valid[0] ? (long)_esc_rpm[0] : 0L,
			 _esc_rpm_valid[1] ? (long)_esc_rpm[1] : 0L);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 5);
	} else {
		add_string_to_screen("1:---- 2:----", 1, 5);
	}

	if (_esc_rpm_valid[2] || _esc_rpm_valid[3]) {
		snprintf(buf, sizeof(buf), "3:%04ld 4:%04ld",
			 _esc_rpm_valid[2] ? (long)_esc_rpm[2] : 0L,
			 _esc_rpm_valid[3] ? (long)_esc_rpm[3] : 0L);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 6);
	} else {
		add_string_to_screen("3:---- 4:----", 1, 6);
	}

	// LEFT LOWER: FC temp + RX
	if (_fc_temp_valid || _rx_valid) {
		if (_fc_temp_valid && _rx_valid) {
			snprintf(buf, sizeof(buf), "Temp%02d R%03u", (int)(_fc_temp_c + 0.5f), (unsigned)_rx_rssi);
		} else if (_fc_temp_valid) {
			snprintf(buf, sizeof(buf), "Temp%02d", (int)(_fc_temp_c + 0.5f));
		} else {
			snprintf(buf, sizeof(buf), "Rssi%03u", (unsigned)_rx_rssi);
		}
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 1, 7);
	} else {
		clear_line(1, 7, 10);
	}

	// RIGHT TOP: altitude
	if (_local_position_valid) {
		snprintf(buf, sizeof(buf), "A%5.1f", (double)_local_position_z);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 22, 1);
	} else {
		clear_line(22, 1, 8);
	}

	// RIGHT: heading
	if (_heading_valid) {
		snprintf(buf, sizeof(buf), "HDG%03d", (int)(_heading_deg + 0.5f));
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 22, 2);
	} else {
		clear_line(22, 2, 8);
	}

	// RIGHT: horizontal speed
	if (_groundspeed_valid) {
		snprintf(buf, sizeof(buf), "GS%4.1f", (double)_groundspeed_m_s);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 22, 3);
	} else {
		clear_line(22, 3, 8);
	}

	// RIGHT: vertical speed
	if (_vertical_speed_valid) {
		snprintf(buf, sizeof(buf), "VS%4.1f", (double)_vertical_speed_m_s);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 22, 4);
	} else {
		clear_line(22, 4, 8);
	}

	// RIGHT: g-force
	if (_g_force_valid) {
		snprintf(buf, sizeof(buf), "G%1.1f/%1.1f", (double)_g_force_h, (double)_g_force_v);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 20, 5);
	} else {
		clear_line(20, 5, 10);
	}

	// RIGHT LOWER: VIO compact indicator
	if (_vio_present && _vio_valid) {
		snprintf(buf, sizeof(buf), "VO%2.0f/%2.0f", (double)_vio_x, (double)_vio_y);
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 21, 6);
	} else {
		clear_line(21, 6, 9);
	}

	// RIGHT LOWER: distance sensor
	if (_distance_sensor_present && _tof_valid) {
		if (_tof_quality >= 0) {
			snprintf(buf, sizeof(buf), "Dist%3.1f Q%02d", (double)_tof_m, (int)_tof_quality);
		} else {
			snprintf(buf, sizeof(buf), "Dist%3.1f", (double)_tof_m);
		}
		buf[sizeof(buf) - 1] = '\0';
		add_string_to_screen(buf, 20, 7);
	} else {
		clear_line(20, 7, 10);
	}

	// CENTER: mode and warnings
	add_string_to_screen_centered(get_flight_mode(_nav_state), 9, 12);
	add_string_to_screen_centered(get_warning_text(), 10, 18);

	if (!_distance_sensor_present || !_tof_valid) {
		add_string_to_screen_centered("DIST BAD", 11, 18);
	} else {
		add_string_to_screen_centered("", 11, 18);
	}

	if (!_vio_present || !_vio_valid) {
		add_string_to_screen_centered("VIO BAD", 12, 18);
	} else {
		add_string_to_screen_centered("", 12, 18);
	}

	// BOTTOM LEFT: flight time
	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const float flight_time_sec = static_cast<float>((hrt_absolute_time() - _arming_timestamp) / (1e6f));
		ret |= add_flighttime(flight_time_sec, 1, 12);
	} else {
		clear_line(1, 12, 8);			snprintf(buf, sizeof(buf), "F%02d R%03u", (int)(_fc_temp_c + 0.5f), (unsigned)_rx_rssi);

	}

	return ret;
}

int
OSDatxxxx::reset()
{
	const int ret = writeRegister(0x00, 0x02);
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

	update_topics();
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
