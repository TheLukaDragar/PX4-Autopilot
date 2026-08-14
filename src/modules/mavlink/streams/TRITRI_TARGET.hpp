/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef TRITRI_TARGET_HPP
#define TRITRI_TARGET_HPP

#include <uORB/topics/mavlink_m_tritri_target.h>

/**
 * Companion mavlink_m_tritri_target_send → TRITRI_TARGET on air.
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

	uORB::Subscription _sub{ORB_ID(mavlink_m_tritri_target_send)};

	bool send() override
	{
		mavlink_m_tritri_target_s topic;

		if (!_sub.update(&topic)) {
			return false;
		}

		mavlink_tritri_target_t msg{};
		msg.time_usec = topic.time_usec;
		msg.target_time_usec = topic.target_time_usec;
		memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));
		memcpy(msg.target_name, topic.target_name, sizeof(msg.target_name));
		msg.target_id = topic.target_id;
		msg.target_set_id = topic.target_set_id;
		msg.flags = topic.flags;
		msg.lat = topic.lat;
		msg.lon = topic.lon;
		msg.alt = topic.alt;
		msg.vx = topic.vx;
		msg.vy = topic.vy;
		msg.vz = topic.vz;
		msg.cov_pos_x = topic.cov_pos_x;
		msg.cov_pos_y = topic.cov_pos_y;
		msg.cov_pos_z = topic.cov_pos_z;
		msg.cov_vel_x = topic.cov_vel_x;
		msg.cov_vel_y = topic.cov_vel_y;
		msg.cov_vel_z = topic.cov_vel_z;
		msg.confidence = topic.confidence;
		msg.target_class = topic.target_class;
		msg.target_domain = topic.target_domain;
		msg.target_force = topic.target_force;
		msg.sensor_type = topic.sensor_type;
		msg.tle_category = topic.tle_category;
		msg.restricted_target_flags = topic.restricted_target_flags;

		mavlink_msg_tritri_target_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // TRITRI_TARGET_HPP
