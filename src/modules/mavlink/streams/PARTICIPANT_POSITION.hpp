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

#ifndef PARTICIPANT_POSITION_HPP
#define PARTICIPANT_POSITION_HPP

#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

/**
 * FC-generated friendly self-report (PPLI).
 * Kinematics from the estimator; identity FRIEND / AIR; callsign "speed0".
 * External track fields left empty until a gateway assigns one.
 */
class MavlinkStreamMavlinkMParticipantPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMavlinkMParticipantPosition(mavlink); }
	static constexpr const char *get_name_static() { return "PARTICIPANT_POSITION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PARTICIPANT_POSITION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _gpos_sub.advertised() ? MAVLINK_MSG_ID_PARTICIPANT_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMavlinkMParticipantPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	bool send() override
	{
		vehicle_global_position_s gpos;
		vehicle_local_position_s lpos;

		if (_gpos_sub.update(&gpos) && _lpos_sub.copy(&lpos)) {
			mavlink_participant_position_t msg{};

			timespec ts{};
			px4_clock_gettime(CLOCK_REALTIME, &ts);
			msg.time_usec = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

			// Skip if wall clock is unset (same gate as SYSTEM_TIME).
			if (msg.time_usec <= 978307200000000ULL) {
				return false;
			}

			if (gpos.lat_lon_valid) {
				msg.lat = (int32_t)(gpos.lat * 1e7);
				msg.lon = (int32_t)(gpos.lon * 1e7);

			} else {
				return false;
			}

			msg.alt = gpos.alt_valid ? gpos.alt : NAN;

			if (lpos.v_xy_valid) {
				msg.vx = lpos.vx;
				msg.vy = lpos.vy;
				// Spec: course over ground, not body heading.
				msg.course = math::degrees(matrix::wrap_2pi(atan2f(lpos.vy, lpos.vx)));

			} else {
				msg.vx = NAN;
				msg.vy = NAN;
				msg.course = NAN;
			}

			msg.vz = lpos.v_z_valid ? lpos.vz : NAN;

			static constexpr const char callsign[] = "speed0";
			strncpy(msg.callsign, callsign, sizeof(msg.callsign) - 1);
			msg.callsign[sizeof(msg.callsign) - 1] = '\0';

			// external_track_number left empty (msg{} zero-init) with type NONE
			msg.origin_sysid = _mavlink->get_system_id();
			msg.external_track_type = MAVLINK_M_TRACK_NUMBER_TYPE_NONE;
			msg.stanag_identity = MAVLINK_M_STANAG_IDENTITY_FRIEND;
			msg.ppli_type = MAVLINK_M_PPLI_TYPE_AIR;

			mavlink_msg_participant_position_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // PARTICIPANT_POSITION_HPP
