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

#pragma once

/**
 * @file atxxxx.h
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#define OSD_SPI_BUS_SPEED (2000000L) /* 2 MHz */

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_CHARS_PER_ROW 30
#define OSD_NUM_ROWS_PAL 16
#define OSD_NUM_ROWS_NTSC 13
#define OSD_ZERO_BYTE 0x00
#define OSD_PAL_TX_MODE 0x40

extern "C" __EXPORT int atxxxx_main(int argc, char *argv[]);

class OSDatxxxx : public device::SPI, public ModuleParams, public I2CSPIDriver<OSDatxxxx>
{
public:
	OSDatxxxx(const I2CSPIDriverConfig &config);
	virtual ~OSDatxxxx() = default;

	static void print_usage();

	int init() override;
	void RunImpl();

protected:
	int probe() override;

private:
	int start();
	int reset();
	int init_osd();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	int add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y);
	void add_string_to_screen(const char *str, uint8_t pos_x, uint8_t pos_y);
	void add_string_to_screen_centered(const char *str, uint8_t pos_y, int max_length);
	void clear_line(uint8_t pos_x, uint8_t pos_y, int length);

	int add_battery_info(uint8_t pos_x, uint8_t pos_y);
	int add_battery_extra(uint8_t pos_x, uint8_t pos_y);
	int add_altitude(uint8_t pos_x, uint8_t pos_y);
	int add_heading(uint8_t pos_x, uint8_t pos_y);
	int add_groundspeed(uint8_t pos_x, uint8_t pos_y);
	int add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y);

	const char *get_warning_text() const;
	static const char *get_flight_mode(uint8_t nav_state);

	int enable_screen();
	int disable_screen();

	int update_topics();
	int update_screen();

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _radio_status_sub{ORB_ID(radio_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};

	// battery
	float _battery_voltage_v{0.f};
	float _battery_discharge_mah{0.f};
	float _battery_current_a{-1.f};
	float _battery_remaining{-1.f};
	uint8_t _battery_warning{battery_status_s::WARNING_NONE};
	bool _battery_valid{false};

	// altitude / local position
	float _local_position_z{0.f};
	bool _local_position_valid{false};
	float _heading_deg{0.f};
	bool _heading_valid{false};
	float _groundspeed_m_s{0.f};
	bool _groundspeed_valid{false};
	float _vertical_speed_m_s{0.f};
	bool _vertical_speed_valid{false};

	// extra telemetry
	float _g_force_h{0.f};
	float _g_force_v{0.f};
	bool _g_force_valid{false};
	float _fc_temp_c{0.f};
	bool _fc_temp_valid{false};

	static constexpr int MAX_ESC_DISPLAY = 4;
	float _esc_temp_c[MAX_ESC_DISPLAY] {0.f, 0.f, 0.f, 0.f};
	bool _esc_temp_valid[MAX_ESC_DISPLAY] {false, false, false, false};
	int32_t _esc_rpm[MAX_ESC_DISPLAY] {0, 0, 0, 0};
	bool _esc_rpm_valid[MAX_ESC_DISPLAY] {false, false, false, false};
	float _esc_power[MAX_ESC_DISPLAY] {0.f, 0.f, 0.f, 0.f};
	bool _esc_power_valid[MAX_ESC_DISPLAY] {false, false, false, false};

	float _tof_m{0.f};
	int8_t _tof_quality{-1};
	bool _tof_valid{false};
	bool _distance_sensor_present{false};
	uint64_t _distance_sensor_timestamp{0};

	uint8_t _rx_rssi{0};
	uint8_t _rx_remote_rssi{0};
	bool _rx_valid{false};

	float _throttle_norm{0.f};
	uint8_t _throttle_pct{0};
	bool _throttle_valid{false};
	bool _manual_throttle_valid{false};

	bool _vio_valid{false};
	bool _vio_present{false};
	uint64_t _vio_timestamp{0};
	float _vio_x{0.f};
	float _vio_y{0.f};

	// warnings / failsafe
	bool _motor_failure{false};
	bool _manual_control_lost{false};
	bool _offboard_lost{false};
	bool _home_invalid{false};
	bool _global_position_invalid{false};
	bool _local_position_invalid{false};
	bool _gcs_lost{false};
	bool _geofence_breached{false};
	bool _failsafe_active{false};
	bool _battery_unhealthy{false};

	// flight time
	uint8_t _arming_state{0};
	uint64_t _arming_timestamp{0};

	// flight mode
	uint8_t _nav_state{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_ATXXXX_CFG>) _param_osd_atxxxx_cfg
	)
};
