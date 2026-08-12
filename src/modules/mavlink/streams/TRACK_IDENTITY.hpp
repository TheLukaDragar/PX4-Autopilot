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

/**
 * Enemy-surrogate build: FC advertises own-ship as HOSTILE TRACK_IDENTITY
 * from the estimator (no companion). track_uid[15] = MAV_SYS_ID.
 * Always streams (no position / wall-clock gate).
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
		return MAVLINK_MSG_ID_TRACK_IDENTITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamMavlinkMTrackIdentity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uint64_t _first_detected_usec{0};

	bool send() override
	{
		timespec ts{};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		uint64_t time_usec = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

		// Prefer wall clock; fall back to boot time so we still TX without GPS time.
		if (time_usec <= 978307200000000ULL) {
			time_usec = hrt_absolute_time();
		}

		if (_first_detected_usec == 0) {
			_first_detected_usec = time_usec;
		}

		mavlink_track_identity_t msg{};
		msg.time_usec = time_usec;
		msg.first_detected_usec = _first_detected_usec;
		msg.track_uid[15] = _mavlink->get_system_id();
		msg.origin_sysid = _mavlink->get_system_id();
		msg.origin_sensor = MAVLINK_M_ID_METHOD_MULTI_SOURCE;
		msg.id_method = MAVLINK_M_ID_METHOD_MULTI_SOURCE;
		msg.pid_status = MAVLINK_M_PID_STATUS_POSITIVE;
		msg.id_confidence = 1.0f;
		msg.target_class = MAVLINK_M_TARGET_CLASS_UAS_MULTIROTOR;
		msg.target_force = MAVLINK_M_TARGET_FORCE_FOE;
		msg.stanag_identity = MAVLINK_M_STANAG_IDENTITY_HOSTILE;
		msg.environment = MAVLINK_M_ENVIRONMENT_AIR;
		msg.external_track_type = MAVLINK_M_TRACK_NUMBER_TYPE_NONE;
		msg.atr_confidence_pct = 255;
		msg.sidc_context = MAVLINK_M_SIDC_CONTEXT_REALITY;

		mavlink_msg_track_identity_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // TRACK_IDENTITY_HPP
