/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file mavlink_tritri.hpp
 * Expand/collapse helpers between lean TRITRI_* (539xx) and full MAVLink-M.
 */

#pragma once

#include <string.h>
#include <math.h>

#if defined(MAVLINK_MSG_ID_TRITRI_TRACK) && defined(MAVLINK_MSG_ID_TRACK_IDENTITY)

static inline void tritri_track_expand(const mavlink_tritri_track_t &lean, mavlink_track_identity_t &full)
{
	memset(&full, 0, sizeof(full));
	full.time_usec = lean.time_usec;
	full.first_detected_usec = lean.first_detected_usec;
	memcpy(full.track_uid, lean.track_uid, sizeof(full.track_uid));
	full.target_set_id = lean.target_set_id;
	full.id_confidence = lean.id_confidence;
	full.origin_sysid = lean.origin_sysid;
	full.origin_sensor = lean.origin_sensor;
	full.id_method = lean.id_method;
	full.pid_status = lean.pid_status;
	full.target_class = lean.target_class;
	full.target_force = lean.target_force;
	full.stanag_identity = lean.stanag_identity;
	full.environment = lean.environment;
	full.atr_confidence_pct = lean.atr_confidence_pct;
	full.atr_model_id = lean.atr_model_id;
	full.atr_conf_tier = lean.atr_conf_tier;
	full.sidc_context = lean.sidc_context;
	/* parent_track_uid, track_rel, id_basis, external_track_* remain zero */
}

static inline void tritri_track_collapse(const mavlink_track_identity_t &full, mavlink_tritri_track_t &lean)
{
	memset(&lean, 0, sizeof(lean));
	lean.time_usec = full.time_usec;
	lean.first_detected_usec = full.first_detected_usec;
	memcpy(lean.track_uid, full.track_uid, sizeof(lean.track_uid));
	lean.target_set_id = full.target_set_id;
	lean.id_confidence = full.id_confidence;
	lean.atr_model_id = full.atr_model_id;
	lean.origin_sysid = full.origin_sysid;
	lean.origin_sensor = full.origin_sensor;
	lean.id_method = full.id_method;
	lean.pid_status = full.pid_status;
	lean.target_class = full.target_class;
	lean.target_force = full.target_force;
	lean.stanag_identity = full.stanag_identity;
	lean.environment = full.environment;
	lean.atr_confidence_pct = full.atr_confidence_pct;
	lean.atr_conf_tier = full.atr_conf_tier;
	lean.sidc_context = full.sidc_context;
}

#endif // TRITRI_TRACK && TRACK_IDENTITY

#if defined(MAVLINK_MSG_ID_TRITRI_TARGET) && defined(MAVLINK_MSG_ID_TARGET)

static inline void tritri_target_expand(const mavlink_tritri_target_t &lean, mavlink_target_t &full)
{
	memset(&full, 0, sizeof(full));
	full.time_usec = lean.time_usec;
	full.target_time_usec = lean.target_time_usec;
	full.target_id = lean.target_id;
	full.target_set_id = lean.target_set_id;
	memcpy(full.target_name, lean.target_name, sizeof(lean.target_name));
	full.lat = lean.lat;
	full.lon = lean.lon;
	full.alt = lean.alt;
	full.vx = lean.vx;
	full.vy = lean.vy;
	full.vz = lean.vz;
	full.cov_pos_x = lean.cov_pos_x;
	full.cov_pos_y = lean.cov_pos_y;
	full.cov_pos_z = lean.cov_pos_z;
	full.cov_vel_x = lean.cov_vel_x;
	full.cov_vel_y = lean.cov_vel_y;
	full.cov_vel_z = lean.cov_vel_z;
	full.cep_desired = NAN;
	full.cep_max = NAN;
	full.flags = lean.flags;
	full.target_class = lean.target_class;
	full.target_domain = lean.target_domain;
	full.target_force = lean.target_force;
	full.confidence = lean.confidence;
	full.sensor_type = lean.sensor_type;
	full.tle_category = lean.tle_category;
	full.restricted_target_flags = lean.restricted_target_flags;
	/* package_*, prf, dmpi, entity_2525d remain 0 */
}

/**
 * Collapse full TARGET + optional track_uid into lean TRITRI_TARGET.
 * track_uid may be nullptr (then all-zero).
 */
static inline void tritri_target_collapse(const mavlink_target_t &full, const uint8_t track_uid[16],
		mavlink_tritri_target_t &lean)
{
	memset(&lean, 0, sizeof(lean));
	lean.time_usec = full.time_usec;
	lean.target_time_usec = full.target_time_usec;

	if (track_uid) {
		memcpy(lean.track_uid, track_uid, sizeof(lean.track_uid));
	}

	memcpy(lean.target_name, full.target_name, sizeof(lean.target_name));
	lean.target_id = full.target_id;
	lean.target_set_id = full.target_set_id;
	lean.flags = full.flags;
	lean.lat = full.lat;
	lean.lon = full.lon;
	lean.alt = full.alt;
	lean.vx = full.vx;
	lean.vy = full.vy;
	lean.vz = full.vz;
	lean.cov_pos_x = full.cov_pos_x;
	lean.cov_pos_y = full.cov_pos_y;
	lean.cov_pos_z = full.cov_pos_z;
	lean.cov_vel_x = full.cov_vel_x;
	lean.cov_vel_y = full.cov_vel_y;
	lean.cov_vel_z = full.cov_vel_z;
	lean.confidence = full.confidence;
	lean.target_class = full.target_class;
	lean.target_domain = full.target_domain;
	lean.target_force = full.target_force;
	lean.sensor_type = full.sensor_type;
	lean.tle_category = full.tle_category;
	lean.restricted_target_flags = full.restricted_target_flags;
}

#endif // TRITRI_TARGET && TARGET
