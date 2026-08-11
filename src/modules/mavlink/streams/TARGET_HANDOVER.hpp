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

#ifndef TARGET_HANDOVER_HPP
#define TARGET_HANDOVER_HPP

#include <uORB/topics/mavlink_m_target_handover.h>

/**
 * Companion TARGET_HANDOVER → MAVLink (custody transfer to another shooter).
 * Subscribes to mavlink_m_target_handover_send (/fmu/in) only — never peer RX.
 */
class MavlinkStreamMavlinkMTargetHandover : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMTargetHandover(mavlink); }
	static constexpr const char *get_name_static() { return "TARGET_HANDOVER"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TARGET_HANDOVER; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_TARGET_HANDOVER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMTargetHandover(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_target_handover_send)};

	bool send() override
	{
		mavlink_m_target_handover_s topic;

		if (_sub.update(&topic)) {
			mavlink_target_handover_t msg{};
			msg.time_usec = topic.time_usec;
			msg.detected_first_usec = topic.detected_first_usec;
			msg.valid_until_usec = topic.valid_until_usec;
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
			msg.target_set_id = topic.target_set_id;
			memcpy(msg.target_name, topic.target_name, sizeof(msg.target_name));
			memcpy(msg.match_media_url, topic.match_media_url, sizeof(msg.match_media_url));
			msg.confidence_score = topic.confidence_score;
			memcpy(msg.authorization, topic.authorization, sizeof(msg.authorization));
			msg.target_class = topic.target_class;
			msg.target_force = topic.target_force;
			msg.match_media_type = topic.match_media_type;
			memcpy(msg.track_uid, topic.track_uid, sizeof(msg.track_uid));

			mavlink_msg_target_handover_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // TARGET_HANDOVER_HPP
