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

#ifndef TRAJECTORY_WAYPOINT_HPP
#define TRAJECTORY_WAYPOINT_HPP

#include <uORB/topics/trajectory_waypoint.h>

class MavlinkStreamTrajectoryWaypoint : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTrajectoryWaypoint(mavlink); }
	static constexpr const char *get_name_static() { return "TRAJECTORY_WAYPOINT"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TRAJECTORY_WAYPOINT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _trajectory_waypoint_sub.advertised() ? MAVLINK_MSG_ID_TRAJECTORY_WAYPOINT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamTrajectoryWaypoint(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _trajectory_waypoint_sub{ORB_ID(trajectory_waypoint)};

	bool send() override
	{
		trajectory_waypoint_s waypoint;
		bool sent = false;

		// Drain all available waypoints to avoid dropping messages
		while (_trajectory_waypoint_sub.update(&waypoint)) {
			mavlink_trajectory_waypoint_t msg{};

			msg.trajectory_id = waypoint.trajectory_id;
			msg.seq = waypoint.seq;
			msg.pos_x = waypoint.pos_x;
			msg.pos_y = waypoint.pos_y;
			msg.pos_z = waypoint.pos_z;
			msg.quat_w = waypoint.quat_w;
			msg.quat_x = waypoint.quat_x;
			msg.quat_y = waypoint.quat_y;
			msg.quat_z = waypoint.quat_z;

			mavlink_msg_trajectory_waypoint_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

#endif // TRAJECTORY_WAYPOINT_HPP
