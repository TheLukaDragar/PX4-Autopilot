/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef FIRES_HPP
#define FIRES_HPP

#include <cstring>
#include <uORB/topics/mavlink_m_fires.h>

class MavlinkStreamMavlinkMFires : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMFires(mavlink); }
	static constexpr const char *get_name_static() { return "FIRES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FIRES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_FIRES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMFires(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_fires)};

	bool send() override
	{
		mavlink_m_fires_s topic;

		if (_sub.update(&topic)) {
			mavlink_fires_t msg{};
			msg.time_usec = topic.time_usec;
			msg.time_impact_usec = topic.time_impact_usec;
			msg.lat = topic.lat;
			msg.lon = topic.lon;
			msg.alt = topic.alt;
			msg.effector_id = topic.effector_id;
			msg.sequence = topic.sequence;
			msg.cep_expected = topic.cep_expected;
			msg.prf_code = topic.prf_code;
			msg.store_id = topic.store_id;
			msg.requested_effect = topic.requested_effect;
			msg.munition_class = topic.munition_class;
			msg.fuze_mode = topic.fuze_mode;
			msg.hob_intent = topic.hob_intent;
			msg.fuze_mofa_capable = topic.fuze_mofa_capable;
			memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));
			mavlink_msg_fires_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // FIRES_HPP
