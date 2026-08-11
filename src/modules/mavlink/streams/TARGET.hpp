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

#ifndef MAVLINK_STREAM_TARGET_HPP
#define MAVLINK_STREAM_TARGET_HPP

#include <uORB/topics/mavlink_m_target.h>

/**
 * Companion TARGET (e.g. onboard seeker) → MAVLink.
 * TARGET has no origin_sysid; enable this stream only on the C2 uplink,
 * not on a broadcast mesh, to avoid rebroadcasting peer/C2 targets.
 */
class MavlinkStreamMavlinkMTarget : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMTarget(mavlink); }
	static constexpr const char *get_name_static() { return "TARGET"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TARGET; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sub.advertised() ? MAVLINK_MSG_ID_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMTarget(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(mavlink_m_target)};

	bool send() override
	{
		mavlink_m_target_s topic;

		if (_sub.update(&topic)) {
			mavlink_target_t msg{};
			msg.time_usec = topic.time_usec;
			msg.target_time_usec = topic.target_time_usec;
			msg.target_id = topic.target_id;
			msg.target_set_id = topic.target_set_id;
			msg.package_id_hash = topic.package_id_hash;
			memcpy(msg.target_name, topic.target_name, sizeof(msg.target_name));
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
			msg.cep_desired = topic.cep_desired;
			msg.cep_max = topic.cep_max;
			msg.flags = topic.flags;
			msg.package_endpoint_ip_1 = topic.package_endpoint_ip_1;
			msg.package_endpoint_ip_2 = topic.package_endpoint_ip_2;
			msg.package_endpoint_ip_3 = topic.package_endpoint_ip_3;
			msg.target_class = topic.target_class;
			msg.target_domain = topic.target_domain;
			msg.target_entity_2525d = topic.target_entity_2525d;
			msg.target_force = topic.target_force;
			msg.confidence = topic.confidence;
			msg.package_endpoint_port = topic.package_endpoint_port;
			msg.sensor_type = topic.sensor_type;
			msg.package_transport = topic.package_transport;
			memcpy(msg.package_path, topic.package_path, sizeof(msg.package_path));
			msg.prf_code = topic.prf_code;
			msg.tle_category = topic.tle_category;
			msg.dmpi_reference_kind = topic.dmpi_reference_kind;
			msg.restricted_target_flags = topic.restricted_target_flags;

			mavlink_msg_target_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // MAVLINK_STREAM_TARGET_HPP
