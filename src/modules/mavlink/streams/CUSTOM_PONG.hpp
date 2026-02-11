/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef CUSTOM_PONG_HPP
#define CUSTOM_PONG_HPP

#include <uORB/topics/custom_pong.h>

class MavlinkStreamCustomPong : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCustomPong(mavlink); }

	static constexpr const char *get_name_static() { return "CUSTOM_PONG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_PONG; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _custom_pong_sub.advertised() ? MAVLINK_MSG_ID_CUSTOM_PONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCustomPong(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _custom_pong_sub{ORB_ID(custom_pong)};

	bool send() override
	{
		custom_pong_s pong;

		if (_custom_pong_sub.update(&pong)) {
			mavlink_custom_pong_t msg{};

			msg.timestamp_usec = pong.timestamp;
			msg.sequence = pong.sequence;
			msg.response_data1 = pong.response_data1;
			msg.response_data2 = pong.response_data2;
			msg.response_status = pong.response_status;

			mavlink_msg_custom_pong_send_struct(_mavlink->get_channel(), &msg);

			PX4_DEBUG("Custom Pong sent: seq=%u, data1=%.2f, data2=%.2f, status=%u",
				  msg.sequence,
				  (double)msg.response_data1,
				  (double)msg.response_data2,
				  msg.response_status);

			return true;
		}

		return false;
	}
};

#endif // CUSTOM_PONG_HPP
