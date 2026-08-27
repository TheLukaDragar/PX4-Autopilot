/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef TRITRI_TRACK_HPP
#define TRITRI_TRACK_HPP

#include <uORB/topics/mavlink_m_tritri_track.h>

/**
 * Own-origin TRITRI_TRACK (companion /fmu/in/mavlink_m_tritri_track).
 */
class MavlinkStreamTritriTrack : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTritriTrack(mavlink); }
	static constexpr const char *get_name_static() { return "TRITRI_TRACK"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TRITRI_TRACK; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_TRITRI_TRACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamTritriTrack(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_tritri_track)};

	bool send() override
	{
		mavlink_m_tritri_track_s topic;

		while (_sub.update(&topic)) {
			if (topic.origin_sysid != _mavlink->get_system_id()) {
				continue;
			}

			mavlink_tritri_track_t msg{};
			msg.time_usec = topic.time_usec;
			msg.first_detected_usec = topic.first_detected_usec;
			memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));
			msg.target_set_id = topic.target_set_id;
			msg.id_confidence = topic.id_confidence;
			msg.atr_model_id = topic.atr_model_id;
			msg.origin_sysid = topic.origin_sysid;
			msg.origin_sensor = topic.origin_sensor;
			msg.id_method = topic.id_method;
			msg.pid_status = topic.pid_status;
			msg.target_class = topic.target_class;
			msg.target_force = topic.target_force;
			msg.stanag_identity = topic.stanag_identity;
			msg.environment = topic.environment;
			msg.atr_confidence_pct = topic.atr_confidence_pct;
			msg.atr_conf_tier = topic.atr_conf_tier;
			msg.sidc_context = topic.sidc_context;

			mavlink_msg_tritri_track_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // TRITRI_TRACK_HPP
