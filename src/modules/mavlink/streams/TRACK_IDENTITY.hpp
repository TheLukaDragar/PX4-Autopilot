/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef TRACK_IDENTITY_HPP
#define TRACK_IDENTITY_HPP

#include <uORB/topics/mavlink_m_track_identity.h>

/**
 * Companion / local-origin TRACK_IDENTITY → MAVLink.
 * Only transmits tracks owned by this system (origin_sysid == MAV_SYS_ID)
 * so peer tracks received on uORB are not rebroadcast.
 */
class MavlinkStreamMavlinkMTrackIdentity : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMTrackIdentity(mavlink); }
	static constexpr const char *get_name_static() { return "TRACK_IDENTITY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TRACK_IDENTITY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_TRACK_IDENTITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMTrackIdentity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_track_identity)};

	bool send() override
	{
		mavlink_m_track_identity_s topic;

		while (_sub.update(&topic)) {
			if (topic.origin_sysid != _mavlink->get_system_id()) {
				continue;
			}

			mavlink_track_identity_t msg{};
			msg.time_usec = topic.time_usec;
			memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));
			memcpy(msg.parent_track_uid, topic.parent_track_uid, sizeof(msg.parent_track_uid));
			msg.target_set_id = topic.target_set_id;
			msg.first_detected_usec = topic.first_detected_usec;
			msg.id_confidence = topic.id_confidence;
			msg.origin_sysid = topic.origin_sysid;
			msg.origin_sensor = topic.origin_sensor;
			msg.id_method = topic.id_method;
			msg.pid_status = topic.pid_status;
			msg.track_rel = topic.track_rel;
			msg.target_class = topic.target_class;
			msg.target_force = topic.target_force;
			memcpy(msg.id_basis, topic.id_basis, sizeof(msg.id_basis));
			memcpy(msg.external_track_number, topic.external_track_number, sizeof(msg.external_track_number));
			msg.external_track_type = topic.external_track_type;
			msg.stanag_identity = topic.stanag_identity;
			msg.environment = topic.environment;
			msg.atr_confidence_pct = topic.atr_confidence_pct;
			msg.atr_model_id = topic.atr_model_id;
			msg.atr_conf_tier = topic.atr_conf_tier;
			msg.sidc_context = topic.sidc_context;

			mavlink_msg_track_identity_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // TRACK_IDENTITY_HPP
