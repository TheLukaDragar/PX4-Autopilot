/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef TRITRI_TARGET_HPP
#define TRITRI_TARGET_HPP

#include <uORB/topics/mavlink_m_target.h>
#include <uORB/topics/mavlink_m_track_identity.h>

#include "../mavlink_tritri.hpp"

/**
 * Companion mavlink_m_target_send → lean TRITRI_TARGET on C2 air.
 * Copies track_uid from latest own-origin TRACK_IDENTITY when available;
 * otherwise zeros UID and sets track_uid[15] = MAV_SYS_ID (demo convention).
 */
class MavlinkStreamTritriTarget : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTritriTarget(mavlink); }
	static constexpr const char *get_name_static() { return "TRITRI_TARGET"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TRITRI_TARGET; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_TRITRI_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamTritriTarget(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_target_send)};
	uORB::Subscription _track_sub{ORB_ID(mavlink_m_track_identity)};
	uint8_t _track_uid[16] {};
	bool _have_track_uid{false};

	void refresh_track_uid()
	{
		mavlink_m_track_identity_s track;

		while (_track_sub.update(&track)) {
			if (track.origin_sysid == _mavlink->get_system_id()) {
				memcpy(_track_uid, track.track_uid, sizeof(_track_uid));
				_have_track_uid = true;
			}
		}
	}

	bool send() override
	{
		refresh_track_uid();

		mavlink_m_target_s topic;

		if (!_sub.update(&topic)) {
			return false;
		}

		mavlink_target_t full{};
		full.time_usec = topic.time_usec;
		full.target_time_usec = topic.target_time_usec;
		full.target_id = topic.target_id;
		full.target_set_id = topic.target_set_id;
		memcpy(full.target_name, topic.target_name, sizeof(full.target_name));
		full.lat = topic.lat;
		full.lon = topic.lon;
		full.alt = topic.alt;
		full.vx = topic.vx;
		full.vy = topic.vy;
		full.vz = topic.vz;
		full.cov_pos_x = topic.cov_pos_x;
		full.cov_pos_y = topic.cov_pos_y;
		full.cov_pos_z = topic.cov_pos_z;
		full.cov_vel_x = topic.cov_vel_x;
		full.cov_vel_y = topic.cov_vel_y;
		full.cov_vel_z = topic.cov_vel_z;
		full.flags = topic.flags;
		full.target_class = topic.target_class;
		full.target_domain = topic.target_domain;
		full.target_force = topic.target_force;
		full.confidence = topic.confidence;
		full.sensor_type = topic.sensor_type;
		full.tle_category = topic.tle_category;
		full.restricted_target_flags = topic.restricted_target_flags;

		uint8_t uid[16] {};

		if (_have_track_uid) {
			memcpy(uid, _track_uid, sizeof(uid));

		} else {
			uid[15] = _mavlink->get_system_id();
		}

		mavlink_tritri_target_t lean{};
		tritri_target_collapse(full, uid, lean);
		mavlink_msg_tritri_target_send_struct(_mavlink->get_channel(), &lean);
		return true;
	}
};

#endif // TRITRI_TARGET_HPP
