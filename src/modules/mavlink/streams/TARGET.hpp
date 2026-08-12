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

#include <climits>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

/**
 * Enemy-surrogate build: FC advertises own-ship kinematics as TARGET
 * (FOE / UAS_MULTIROTOR) from the estimator. target_id = MAV_SYS_ID.
 * Unknown lat/lon = INT32_MAX; unknown alt/vel/cov/CEP = NaN (never fake 0,0).
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
		return MAVLINK_MSG_ID_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamMavlinkMTarget(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		vehicle_global_position_s gpos{};
		vehicle_local_position_s lpos{};
		_gpos_sub.copy(&gpos);
		_lpos_sub.copy(&lpos);

		timespec ts{};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		uint64_t time_usec = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

		// Prefer wall clock; fall back to boot time so we still TX without GPS time.
		if (time_usec <= 978307200000000ULL) {
			time_usec = hrt_absolute_time();
		}

		mavlink_target_t msg{};
		msg.time_usec = time_usec;
		msg.target_time_usec = time_usec;
		msg.target_id = _mavlink->get_system_id();

		if (gpos.lat_lon_valid) {
			msg.lat = (int32_t)(gpos.lat * 1e7);
			msg.lon = (int32_t)(gpos.lon * 1e7);

		} else {
			msg.lat = INT32_MAX;
			msg.lon = INT32_MAX;
		}

		msg.alt = gpos.alt_valid ? gpos.alt : NAN;

		if (lpos.v_xy_valid) {
			msg.vx = lpos.vx;
			msg.vy = lpos.vy;

		} else {
			msg.vx = NAN;
			msg.vy = NAN;
		}

		msg.vz = lpos.v_z_valid ? lpos.vz : NAN;

		// No estimator covariance on this path — mark unknown (global dialect rule).
		msg.cov_pos_x = NAN;
		msg.cov_pos_y = NAN;
		msg.cov_pos_z = NAN;
		msg.cov_vel_x = NAN;
		msg.cov_vel_y = NAN;
		msg.cov_vel_z = NAN;
		msg.cep_desired = NAN;
		msg.cep_max = NAN;

		static constexpr const char name[] = "leseni";
		strncpy(msg.target_name, name, sizeof(msg.target_name) - 1);
		msg.target_name[sizeof(msg.target_name) - 1] = '\0';

		msg.target_class = MAVLINK_M_TARGET_CLASS_UAS_MULTIROTOR;
		msg.target_domain = MAVLINK_M_TARGET_DOMAIN_AIR;
		msg.target_force = MAVLINK_M_TARGET_FORCE_FOE;
		msg.confidence = 10000;
		msg.sensor_type = MAVLINK_M_TARGET_SENSOR_TYPE_FUSED;

		mavlink_msg_target_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // MAVLINK_STREAM_TARGET_HPP
