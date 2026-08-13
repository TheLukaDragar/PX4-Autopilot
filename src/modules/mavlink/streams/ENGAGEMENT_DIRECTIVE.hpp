/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#ifndef ENGAGEMENT_DIRECTIVE_HPP
#define ENGAGEMENT_DIRECTIVE_HPP

#include <cstring>
#include <uORB/topics/mavlink_m_engagement_directive.h>

class MavlinkStreamMavlinkMEngagementDirective : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMEngagementDirective(mavlink); }
	static constexpr const char *get_name_static() { return "ENGAGEMENT_DIRECTIVE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ENGAGEMENT_DIRECTIVE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_ENGAGEMENT_DIRECTIVE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMEngagementDirective(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_engagement_directive)};

	bool send() override
	{
		mavlink_m_engagement_directive_s topic;

		if (_sub.update(&topic)) {
			mavlink_engagement_directive_t msg{};
			msg.time_usec = topic.time_usec;
			memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));
			msg.sequence = topic.sequence;
			msg.effector_id = topic.effector_id;
			msg.retarget_lat = topic.retarget_lat;
			msg.retarget_lon = topic.retarget_lon;
			msg.retarget_alt = topic.retarget_alt;
			msg.directive = topic.directive;
			msg.origin_sysid = topic.origin_sysid;
			mavlink_msg_engagement_directive_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // ENGAGEMENT_DIRECTIVE_HPP
