/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef TRITRI_TRACK_HPP
#define TRITRI_TRACK_HPP

#include <uORB/topics/mavlink_m_track_identity.h>

#include "../mavlink_tritri.hpp"

/**
 * Own-origin TRACK → lean TRITRI_TRACK on C2 air.
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

	uORB::Subscription _sub{ORB_ID(mavlink_m_track_identity)};

	bool send() override
	{
		mavlink_m_track_identity_s topic;

		while (_sub.update(&topic)) {
			if (topic.origin_sysid != _mavlink->get_system_id()) {
				continue;
			}

			mavlink_track_identity_t full{};
			full.time_usec = topic.time_usec;
			memcpy(full.track_uid, topic.track_uid, sizeof(full.track_uid));
			full.target_set_id = topic.target_set_id;
			full.first_detected_usec = topic.first_detected_usec;
			full.id_confidence = topic.id_confidence;
			full.origin_sysid = topic.origin_sysid;
			full.origin_sensor = topic.origin_sensor;
			full.id_method = topic.id_method;
			full.pid_status = topic.pid_status;
			full.target_class = topic.target_class;
			full.target_force = topic.target_force;
			full.stanag_identity = topic.stanag_identity;
			full.environment = topic.environment;
			full.atr_confidence_pct = topic.atr_confidence_pct;
			full.atr_model_id = topic.atr_model_id;
			full.atr_conf_tier = topic.atr_conf_tier;
			full.sidc_context = topic.sidc_context;

			mavlink_tritri_track_t lean{};
			tritri_track_collapse(full, lean);
			mavlink_msg_tritri_track_send_struct(_mavlink->get_channel(), &lean);
			return true;
		}

		return false;
	}
};

#endif // TRITRI_TRACK_HPP
