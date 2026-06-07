/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/* uorb_to_msp.cpp
 *
 * Implementation file for UORB -> MSP conversion functions.
 */

// includes for mathematical manipulation
#include <climits>
#include <cmath>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/modes/ui.hpp>
#include <matrix/math.hpp>

// clock access
#include <px4_platform_common/defines.h>
using namespace time_literals;

#include "uorb_to_msp.hpp"

namespace msp_osd
{

// Betaflight osd.h OSD_X / OSD_Y (character grid from packed uint16_t position)
static inline uint8_t osd_grid_x_bf(uint16_t pos)
{
	return (uint8_t)((pos & 31u) | ((pos & (1u << 10)) >> 5u));
}

static inline uint8_t osd_grid_y_bf(uint16_t pos)
{
	return (uint8_t)((pos >> 5) & 31u);
}

typedef enum {
	MSP_DP_HEARTBEAT = 0,         // Release the display after clearing and updating
	MSP_DP_RELEASE = 1,         // Release the display after clearing and updating
	MSP_DP_CLEAR_SCREEN = 2,    // Clear the display
	MSP_DP_WRITE_STRING = 3,    // Write a string at given coordinates
	MSP_DP_DRAW_SCREEN = 4,     // Trigger a screen draw
	MSP_DP_OPTIONS = 5,         // Not used by Betaflight. Reserved by Ardupilot and INAV
	MSP_DP_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
	MSP_DP_COUNT,
} displayportMspSubCommand;

msp_name_t construct_display_message(const vehicle_status_s &vehicle_status,
				     const vehicle_attitude_s &vehicle_attitude,
				     const log_message_s &log_message,
				     const int log_level,
				     MessageDisplay &display)
{
	// initialize result
	msp_name_t display_message {0};

	const auto now = hrt_absolute_time();
	static uint64_t last_warning_stamp {0};

	// update arming state, flight mode, and warnings, if current
	if (vehicle_status.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::ARMING, "???");
		display.set(MessageDisplayType::FLIGHT_MODE, "???");

	} else {
		// display armed / disarmed
		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			display.set(MessageDisplayType::ARMING, "ARM");

		} else {
			display.set(MessageDisplayType::ARMING, "DSRM");
		}

		// display flight mode
		display.set(MessageDisplayType::FLIGHT_MODE, mode_util::nav_state_names[vehicle_status.nav_state]);
	}

	// display, if updated
	if (log_message.severity <= log_level) {
		display.set(MessageDisplayType::WARNING, log_message.text);
		last_warning_stamp = now;

	} else if (now - last_warning_stamp > 30_s) {
		// clear warning after timeout
		display.set(MessageDisplayType::WARNING, "");
		last_warning_stamp = now;
	}

	// update heading, if relatively recent
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		display.set(MessageDisplayType::HEADING, "N?");

	} else {
		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
		const auto yaw = math::degrees(euler_attitude.psi());

		// display north direction
		if (yaw <= 22.5f) {
			display.set(MessageDisplayType::HEADING, "N");

		} else if (yaw <= 67.5f) {
			display.set(MessageDisplayType::HEADING, "NE");

		} else if (yaw <= 112.5f) {
			display.set(MessageDisplayType::HEADING, "E");

		} else if (yaw <= 157.5f) {
			display.set(MessageDisplayType::HEADING, "SE");

		} else if (yaw <= 202.5f) {
			display.set(MessageDisplayType::HEADING, "S");

		} else if (yaw <= 247.5f) {
			display.set(MessageDisplayType::HEADING, "SW");

		} else if (yaw <= 292.5f) {
			display.set(MessageDisplayType::HEADING, "W");

		} else if (yaw <= 337.5f) {
			display.set(MessageDisplayType::HEADING, "NW");

		} else if (yaw <= 360.0f) {
			display.set(MessageDisplayType::HEADING, "N");
		}
	}

	// update message and return
	display.get(display_message.craft_name, hrt_absolute_time());
	return display_message;
}

msp_fc_variant_t construct_FC_VARIANT()
{
	// initialize result
	msp_fc_variant_t variant{};

	memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
	return variant;
}

msp_status_BF_t construct_STATUS(const vehicle_status_s &vehicle_status)
{
	// initialize result
	msp_status_BF_t status_BF = {0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status_BF.flight_mode_flags |= ARM_ACRO_BF;

		switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			status_BF.flight_mode_flags |= STAB_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			status_BF.flight_mode_flags |= RESC_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			status_BF.flight_mode_flags |= FS_BF;
			break;

		default:
			status_BF.flight_mode_flags |= 0;
			break;
		}
	}

	status_BF.arming_disable_flags_count = 1;
	status_BF.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	return status_BF;
}

msp_analog_t construct_ANALOG(const battery_status_s &battery_status, const input_rc_s &input_rc)
{
	// initialize result
	msp_analog_t analog {0};

	analog.vbat = battery_status.voltage_v * 10; // bottom right... v * 10
	analog.rssi = (uint16_t)((input_rc.link_quality * 1023.0f) / 100.0f);
	analog.amperage = battery_status.current_a * 100; // main amperage
	analog.mAhDrawn = battery_status.discharged_mah; // unused
	return analog;
}

msp_rendor_rssi_t construct_rendor_RSSI(const input_rc_s &input_rc)
{
	msp_rendor_rssi_t rssi;
	rssi.screenYPosition = 0x02;
	rssi.screenXPosition = 0x02;

	snprintf(&rssi.str[0], sizeof(rssi.str), "%3d", input_rc.link_quality);
	rssi.str[3] = '%';

	return rssi;
}

msp_rendor_crosshairs_t construct_rendor_CROSSHAIRS(uint16_t grid_pos)
{
	msp_rendor_crosshairs_t crosshairs{};
	crosshairs.subCommand = MSP_DP_WRITE_STRING;
	crosshairs.screenYPosition = osd_grid_y_bf(grid_pos);
	crosshairs.screenXPosition = osd_grid_x_bf(grid_pos);
	crosshairs.attr = 0;
	// Betaflight src/main/osd/osd_elements.c osdElementCrosshairs + osd_symbols.h
	crosshairs.sym[0] = (char)0x72; // SYM_AH_CENTER_LINE
	crosshairs.sym[1] = (char)0x73; // SYM_AH_CENTER
	crosshairs.sym[2] = (char)0x74; // SYM_AH_CENTER_LINE_RIGHT
	return crosshairs;
}

msp_battery_state_t construct_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_battery_state_t battery_state = {0};

	// MSP_BATTERY_STATE
	battery_state.amperage = battery_status.current_a * 100.0f; // Used for power element
	battery_state.batteryVoltage = (uint16_t)((battery_status.voltage_v / battery_status.cell_count) * 400.0f);  // OK
	battery_state.mAhDrawn = battery_status.discharged_mah ; // OK
	battery_state.batteryCellCount = battery_status.cell_count;
	battery_state.batteryCapacity = battery_status.capacity; // not used?

	// Voltage color 0==white, 1==red
	if (battery_status.voltage_v < 14.4f) {
		battery_state.batteryState = 1;

	} else {
		battery_state.batteryState = 0;
	}

	battery_state.legacyBatteryVoltage = battery_status.voltage_v * 10;
	return battery_state;
}

msp_rendor_battery_state_t construct_rendor_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_rendor_battery_state_t battery_state = {0};

	battery_state.subCommand = MSP_DP_WRITE_STRING; // 3 write string. fixed
	battery_state.screenYPosition = 0x04;
	battery_state.screenXPosition = 0x02;

	const float cell_v = battery_status.cell_count > 0
			     ? battery_status.voltage_v / battery_status.cell_count
			     : battery_status.voltage_v;

	// Betaflight: BATTERY_WARNING → severity 1, BATTERY_CRITICAL → severity 2
	if (cell_v < 3.2f) {
		battery_state.iconAttrs = 2; // DISPLAYPORT_SEVERITY_CRITICAL

	} else if (cell_v < 3.5f) {
		battery_state.iconAttrs = 1; // DISPLAYPORT_SEVERITY_WARNING

	} else {
		battery_state.iconAttrs = 0;
	}

	if (cell_v > 4.0f) {
		battery_state.iconIndex = 0x91; // SYM_BATT_5 (full)

	} else if (cell_v > 3.5f) {
		battery_state.iconIndex = 0x93; // SYM_BATT_3 (half)

	} else if (cell_v > 3.2f) {
		battery_state.iconIndex = 0x95; // SYM_BATT_1 (low)

	} else {
		battery_state.iconIndex = 0x96; // SYM_BATT_EMPTY
	}

	snprintf(&battery_state.str[0], sizeof(battery_state.str), "%.1fV", (double)cell_v);
	return battery_state;
}

msp_rendor_mah_t construct_rendor_MAH(const battery_status_s &battery_status)
{
	msp_rendor_mah_t m{};
	m.screenYPosition = 0x0C; // row 12
	m.screenXPosition = 0x13; // col 19

	snprintf(m.str, sizeof(m.str), "%4d", (int)battery_status.discharged_mah);
	return m;
}

msp_rendor_current_t construct_rendor_CURRENT(const battery_status_s &battery_status)
{
	msp_rendor_current_t m{};
	m.screenYPosition = 0x05; // row 5
	m.screenXPosition = 0x02; // col 2

	if (!battery_status.connected || !PX4_ISFINITE(battery_status.current_a)) {
		snprintf(m.str, sizeof(m.str), " --A");

	} else {
		snprintf(m.str, sizeof(m.str), "%.1fA", (double)battery_status.current_a);
	}

	return m;
}

msp_rendor_vario_t construct_rendor_VARIO(const vehicle_local_position_s &vehicle_local_position)
{
	msp_rendor_vario_t m{};
	m.screenYPosition = 0x06; // row 6
	m.screenXPosition = 0x17; // col 23 — with est speed

	if (!vehicle_local_position.v_z_valid || !PX4_ISFINITE(vehicle_local_position.vz)) {
		m.iconIndex = 0x75; // SYM_ARROW_SMALL_UP (placeholder)
		snprintf(m.str, sizeof(m.str), " --");

	} else {
		// NED: vz positive = descending; negate for climb rate
		const float climb_ms = -vehicle_local_position.vz;
		m.iconIndex = (climb_ms >= 0.0f) ? 0x75 : 0x76; // up / down
		snprintf(m.str, sizeof(m.str), "%.1f", (double)fabsf(climb_ms));
	}

	return m;
}


msp_raw_gps_t construct_RAW_GPS(const sensor_gps_s &vehicle_gps_position,
				const airspeed_validated_s &airspeed_validated)
{
	// initialize result
	msp_raw_gps_t raw_gps {0};

	if (vehicle_gps_position.fix_type >= 2) {
		raw_gps.lat = static_cast<int32_t>(vehicle_gps_position.latitude_deg * 1e7);
		raw_gps.lon = static_cast<int32_t>(vehicle_gps_position.longitude_deg * 1e7);
		raw_gps.alt = static_cast<int16_t>(vehicle_gps_position.altitude_msl_m * 100.0);

		float course = math::degrees(vehicle_gps_position.cog_rad);

		if (course < 0) {
			course += 360.0f;
		}

		raw_gps.groundCourse = course * 100.0f; // centidegrees

	} else {
		raw_gps.lat = 0;
		raw_gps.lon = 0;
		raw_gps.alt = 0;
		raw_gps.groundCourse = 0; // centidegrees
	}

	if (vehicle_gps_position.fix_type == 0
	    || vehicle_gps_position.fix_type == 1) {
		raw_gps.fixType = MSP_GPS_NO_FIX;

	} else if (vehicle_gps_position.fix_type == 2) {
		raw_gps.fixType = MSP_GPS_FIX_2D;

	} else if (vehicle_gps_position.fix_type >= 3 && vehicle_gps_position.fix_type <= 5) {
		raw_gps.fixType = MSP_GPS_FIX_3D;

	} else {
		raw_gps.fixType = MSP_GPS_NO_FIX;
	}

	//raw_gps.hdop = vehicle_gps_position_struct.hdop
	raw_gps.numSat = vehicle_gps_position.satellites_used;

	// BF "GPS speed" element: receiver ground speed, else pitot (cm/s)
	float speed_m_s = 0.f;

	if (vehicle_gps_position.fix_type >= 2 && PX4_ISFINITE(vehicle_gps_position.vel_m_s)) {
		speed_m_s = vehicle_gps_position.vel_m_s;

	} else if (airspeed_validated.airspeed_source >= airspeed_validated_s::SOURCE_GROUND_MINUS_WIND
		   && PX4_ISFINITE(airspeed_validated.indicated_airspeed_m_s)
		   && airspeed_validated.indicated_airspeed_m_s > 0.f) {
		speed_m_s = airspeed_validated.indicated_airspeed_m_s;
	}

	long gs_cms = lroundf(speed_m_s * 100.f);

	if (gs_cms > INT16_MAX) {
		gs_cms = INT16_MAX;

	} else if (gs_cms < INT16_MIN) {
		gs_cms = INT16_MIN;
	}

	raw_gps.groundSpeed = static_cast<int16_t>(gs_cms);

	return raw_gps;
}

msp_rendor_latitude_t construct_rendor_GPS_LAT(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_latitude_t lat;

	lat.screenYPosition = 0x0A;
	lat.screenXPosition = 0x29;

	if (vehicle_gps_position.fix_type >= 2) {
		const double v = fabs(vehicle_gps_position.latitude_deg);
		const char ns = vehicle_gps_position.latitude_deg >= 0.0 ? 'N' : 'S';
		snprintf(&lat.str[0], sizeof(lat.str), "%.6f%c", v, ns);

	} else {
		snprintf(&lat.str[0], sizeof(lat.str), "0.000000N");
	}

	return lat;
}

msp_rendor_longitude_t construct_rendor_GPS_LON(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_longitude_t lon;

	lon.screenYPosition = 0x09;
	lon.screenXPosition = 0x29;

	if (vehicle_gps_position.fix_type >= 2) {
		const double v = fabs(vehicle_gps_position.longitude_deg);
		const char ew = vehicle_gps_position.longitude_deg >= 0.0 ? 'E' : 'W';
		snprintf(&lon.str[0], sizeof(lon.str), "%.6f%c", v, ew);

	} else {
		snprintf(&lon.str[0], sizeof(lon.str), "0.000000E");
	}

	return lon;
}

msp_rendor_satellites_used_t construct_rendor_GPS_NUM(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_satellites_used_t num;

	num.screenYPosition = 0x08;
	num.screenXPosition = 0x29;

	memset(&num.str[0], 0, sizeof(num.str));

	if (vehicle_gps_position.fix_type < 2) {
		// Betaflight shows "NC" when GPS is unhealthy
		snprintf(&num.str[0], sizeof(num.str), "NC");

	} else {
		snprintf(&num.str[0], sizeof(num.str), "%d", vehicle_gps_position.satellites_used);
	}

	return num;
}


msp_comp_gps_t construct_COMP_GPS(const home_position_s &home_position,
				  const vehicle_global_position_s &vehicle_global_position,
				  const bool heartbeat)
{
	// initialize result
	msp_comp_gps_t comp_gps {0};

	// Calculate distance and direction to home
	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

		float bearing_to_home = math::degrees(get_bearing_to_next_waypoint(vehicle_global_position.lat,
						      vehicle_global_position.lon,
						      home_position.lat, home_position.lon));

		if (bearing_to_home < 0) {
			bearing_to_home += 360.0f;
		}

		float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
					 vehicle_global_position.lon,
					 home_position.lat, home_position.lon);

		comp_gps.distanceToHome = (int16_t)distance_to_home; // meters
		comp_gps.directionToHome = bearing_to_home;

	} else {
		comp_gps.distanceToHome = 0; // meters
		comp_gps.directionToHome = 0;
	}

	comp_gps.heartbeat = heartbeat;
	return comp_gps;
}

msp_rendor_distanceToHome_t construct_rendor_distanceToHome(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position)
{
	msp_rendor_distanceToHome_t distance;

	distance.screenYPosition = 0x08;
	distance.screenXPosition = 0x02;

	float dist_m = 0.0f;

	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

		dist_m = get_distance_to_next_waypoint(vehicle_global_position.lat,
						       vehicle_global_position.lon,
						       home_position.lat, home_position.lon);
	}

	memset(&distance.str[0], 0, sizeof(distance.str));

	if (dist_m < 1000.0f) {
		snprintf(&distance.str[0], sizeof(distance.str), "%dm", (int)dist_m);

	} else {
		snprintf(&distance.str[0], sizeof(distance.str), "%.1fkm", (double)(dist_m / 1000.0f));
	}

	return distance;
}

msp_attitude_t construct_ATTITUDE(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_attitude_t attitude {0};

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	attitude.pitch = math::degrees(euler_attitude.theta()) * 10;
	attitude.roll = math::degrees(euler_attitude.phi()) * 10;
	//attitude.yaw = math::degrees(euler_attitude.psi()) * 10;

	float yaw_fixed = math::degrees(euler_attitude.psi());

	if (yaw_fixed < 0) {
		yaw_fixed += 360.0f;
	}

	attitude.yaw = yaw_fixed;

	//attitude.yaw = 360;

	return attitude;
}

msp_rendor_pitch_t  construct_rendor_PITCH(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_pitch_t pit;

	pit.screenYPosition = 0x0D;
	pit.screenXPosition = 0x29;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	const double pitch_deg = (double)math::degrees(euler_attitude.theta());

	memset(&pit.str[0], 0, sizeof(pit.str));
	// Betaflight: sign char + zero-padded 2-digit integer + 1 decimal, e.g. "-12.3" or " 05.0"
	snprintf(&pit.str[0], sizeof(pit.str), "%c%04.1f",
		 pitch_deg < 0.0 ? '-' : ' ', fabs(pitch_deg));

	return pit;
}

msp_rendor_roll_t  construct_rendor_ROLL(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_roll_t roll;

	roll.screenYPosition = 0x0E;
	roll.screenXPosition = 0x29;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	const double roll_deg = (double)math::degrees(euler_attitude.phi());

	memset(&roll.str[0], 0, sizeof(roll.str));
	snprintf(&roll.str[0], sizeof(roll.str), "%c%04.1f",
		 roll_deg < 0.0 ? '-' : ' ', fabs(roll_deg));

	return roll;
}


msp_altitude_t construct_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
				  const vehicle_local_position_s &vehicle_local_position)
{
	// initialize result
	msp_altitude_t altitude {0};

	if (vehicle_gps_position.fix_type >= 2) {
		altitude.estimatedActualPosition = static_cast<int32_t>(vehicle_gps_position.altitude_msl_m * 100.0);	// cm

	} else {
		altitude.estimatedActualPosition = 0;
	}

	if (vehicle_local_position.v_z_valid) {
		altitude.estimatedActualVelocity = -vehicle_local_position.vz * 100; //m/s to cm/s

	} else {
		altitude.estimatedActualVelocity = 0;
	}

	return altitude;
}

msp_rendor_altitude_t construct_Rendor_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
		const vehicle_local_position_s &vehicle_local_position)
{
	msp_rendor_altitude_t altitude;

	altitude.screenYPosition = 0x06;
	altitude.screenXPosition = 0x02;

	double alt;

	if (vehicle_gps_position.fix_type >= 2) {
		alt = vehicle_gps_position.altitude_msl_m;

	} else {
		alt = (double)(vehicle_local_position.z * -1.0f);
	}

	memset(&altitude.str[0], 0, sizeof(altitude.str));
	snprintf(&altitude.str[0], sizeof(altitude.str), "%.1f", alt);

	return altitude;
}

msp_rendor_esc_tmp_t construct_rendor_ESC_TMP(const esc_status_s &esc_status, uint16_t grid_pos)
{
	msp_rendor_esc_tmp_t esc_tmp{};

	esc_tmp.screenYPosition = osd_grid_y_bf(grid_pos);
	esc_tmp.screenXPosition = osd_grid_x_bf(grid_pos);

	float max_temp = -INFINITY;
	bool any_valid = false;

	for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX && i < esc_status.esc_count; i++) {
		float t = esc_status.esc[i].esc_temperature;
		if (PX4_ISFINITE(t) && t > 0.f) {
			any_valid = true;
			if (t > max_temp) { max_temp = t; }
		}
	}

	memset(&esc_tmp.str[0], 0, sizeof(esc_tmp.str));
	if (any_valid) {
		snprintf(&esc_tmp.str[0], sizeof(esc_tmp.str), "%3.0fC", (double)max_temp);
	} else {
		snprintf(&esc_tmp.str[0], sizeof(esc_tmp.str), " --");
	}

	return esc_tmp;
}

msp_rendor_batt_pct_t construct_rendor_BATT_PCT(const battery_status_s &battery_status, uint16_t grid_pos)
{
	msp_rendor_batt_pct_t m {};

	m.screenYPosition = osd_grid_y_bf(grid_pos);
	m.screenXPosition = osd_grid_x_bf(grid_pos);

	if (!battery_status.connected) {
		m.iconIndex = 0x96; // SYM_BATT_EMPTY
		snprintf(m.str, sizeof(m.str), " --");
		return m;
	}

	const float cell_v = battery_status.cell_count > 0
			     ? battery_status.voltage_v / (float)battery_status.cell_count
			     : battery_status.voltage_v;

	// Same SYM_BATT_* bar icons as construct_rendor_BATTERY_STATE (0x97 "MAIN_BATT" renders wrong on some fonts).
	if (cell_v > 4.0f) {
		m.iconIndex = 0x91; // SYM_BATT_5

	} else if (cell_v > 3.5f) {
		m.iconIndex = 0x93; // SYM_BATT_3

	} else if (cell_v > 3.2f) {
		m.iconIndex = 0x95; // SYM_BATT_1

	} else {
		m.iconIndex = 0x96; // SYM_BATT_EMPTY
	}

	if (PX4_ISFINITE(battery_status.remaining) && battery_status.remaining >= 0.f) {
		float pct = battery_status.remaining * 100.f;

		if (pct > 100.f) {
			pct = 100.f;
		}

		const int ip = math::constrain((int)roundf(pct), 0, 100);
		// Integer only — '%' (0x25) is not a reliable glyph on some MSP DisplayPort fonts and can blank the line.
		snprintf(m.str, sizeof(m.str), "%d", ip);

	} else if (PX4_ISFINITE(battery_status.volt_based_soc_estimate) && battery_status.volt_based_soc_estimate >= 0.f) {
		float pct = battery_status.volt_based_soc_estimate * 100.f;

		if (pct > 100.f) {
			pct = 100.f;
		}

		const int ip = math::constrain((int)roundf(pct), 0, 100);
		snprintf(m.str, sizeof(m.str), "%d", ip);

	} else {
		m.iconIndex = 0x96;
		snprintf(m.str, sizeof(m.str), " --");
	}

	return m;
}


msp_rendor_est_speed_t construct_rendor_EST_SPEED(const vehicle_local_position_s &vehicle_local_position,
		uint16_t grid_pos)
{
	msp_rendor_est_speed_t est {};

	est.screenYPosition = osd_grid_y_bf(grid_pos);
	est.screenXPosition = osd_grid_x_bf(grid_pos);

	if (vehicle_local_position.v_xy_valid) {
		const float kmh = hypotf(vehicle_local_position.vx, vehicle_local_position.vy) * 3.6f;
		snprintf(est.str, sizeof(est.str), "%.0f", (double)kmh);

	} else {
		snprintf(est.str, sizeof(est.str), " --");
	}

	return est;
}

msp_esc_sensor_data_dji_t construct_ESC_SENSOR_DATA()
{
	// initialize result
	msp_esc_sensor_data_dji_t esc_sensor_data {0};

	esc_sensor_data.rpm = 0;
	esc_sensor_data.temperature = 50;

	return esc_sensor_data;
}

msp_rc_t construct_MSP_RC(const input_rc_s &input_rc)
{
	// initialize result
	msp_rc_t rc;

	rc.channelValue[0] = input_rc.values[0]; // roll
	rc.channelValue[1] = input_rc.values[1]; // pitch
	rc.channelValue[2] = input_rc.values[3]; // yaw
	rc.channelValue[3] = input_rc.values[2]; // Throttle
	return rc;
}

msp_status_t construct_MSP_STATUS(const vehicle_status_s &vehicle_status)
{
	// initialize result
	msp_status_t status{0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status.flightModeFlags |= (1 << MSP_MODE_ARM);
	}

	return status;
}



} // namespace msp_osd
