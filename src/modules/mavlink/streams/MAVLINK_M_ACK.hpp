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

#ifndef MAVLINK_M_ACK_HPP
#define MAVLINK_M_ACK_HPP

#include <uORB/topics/mavlink_m_ack.h>

class MavlinkStreamMavlinkMAck : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMAck(mavlink); }
	static constexpr const char *get_name_static() { return "MAVLINK_M_ACK"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAVLINK_M_ACK; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mavlink_m_ack_sub.advertised() ? MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMAck(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mavlink_m_ack_sub{ORB_ID(mavlink_m_ack)};

	bool send() override
	{
		mavlink_m_ack_s topic;

		// Only forward ACKs issued by this system (peer ACKs land on the same
		// uORB topic via RX and must not be rebroadcast).
		while (_mavlink_m_ack_sub.update(&topic)) {
			if (topic.ack_sysid != _mavlink->get_system_id()) {
				continue;
			}

			mavlink_mavlink_m_ack_t msg{};

			msg.time_usec = topic.time_usec;
			msg.ack_msgid = topic.ack_msgid;
			msg.ack_instance = topic.ack_instance;
			msg.origin_sysid = topic.origin_sysid;
			msg.ack_sysid = topic.ack_sysid;
			msg.result = topic.result;
			memcpy(msg.reason, topic.reason, sizeof(msg.reason));

			mavlink_msg_mavlink_m_ack_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // MAVLINK_M_ACK_HPP
