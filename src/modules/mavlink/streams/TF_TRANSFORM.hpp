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

#ifndef TF_TRANSFORM_HPP
#define TF_TRANSFORM_HPP

#include <uORB/topics/tf_transform.h>

class MavlinkStreamTfTransform : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTfTransform(mavlink); }
	static constexpr const char *get_name_static() { return "TF_TRANSFORM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TF_TRANSFORM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _tf_transform_sub.advertised() ? MAVLINK_MSG_ID_TF_TRANSFORM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamTfTransform(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _tf_transform_sub{ORB_ID(tf_transform)};

	bool send() override
	{
		tf_transform_s tf;

		if (_tf_transform_sub.update(&tf)) {
			mavlink_tf_transform_t msg{};

			msg.timestamp_usec = tf.timestamp;
			msg.frame_id = tf.frame_id;
			msg.child_frame_id = tf.child_frame_id;
			msg.translation_x = tf.translation_x;
			msg.translation_y = tf.translation_y;
			msg.translation_z = tf.translation_z;
			msg.rotation_x = tf.rotation_x;
			msg.rotation_y = tf.rotation_y;
			msg.rotation_z = tf.rotation_z;
			msg.rotation_w = tf.rotation_w;

			mavlink_msg_tf_transform_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // TF_TRANSFORM_HPP
