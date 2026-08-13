/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Enemy-surrogate NSH helper: publish one-shot C2 COP onto uORB so Custom
 * streams TX TARGET_HANDOVER / FIRES / ENGAGEMENT_DIRECTIVE.
 *
 ****************************************************************************/

#include "mavlink_main.h"

#include <climits>
#include <cmath>
#include <cstring>

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mavlink_m_ack.h>
#include <uORB/topics/mavlink_m_engagement_directive.h>
#include <uORB/topics/mavlink_m_fires.h>
#include <uORB/topics/mavlink_m_target_handover.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#ifndef MAVLINK_M_ENGAGEMENT_DIRECTIVE_ABORT
#define MAVLINK_M_ENGAGEMENT_DIRECTIVE_ABORT 1
#endif

namespace
{

uint16_t g_cop_seq{1};

uint64_t wall_time_usec()
{
	timespec ts{};
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	uint64_t t = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

	if (t <= 978307200000000ULL) {
		t = hrt_absolute_time();
	}

	return t;
}

uint8_t own_sysid()
{
	int32_t sysid = 1;
	param_get(param_find("MAV_SYS_ID"), &sysid);
	return (uint8_t)sysid;
}

void fill_track_uid(uint8_t track_uid[16])
{
	memset(track_uid, 0, 16);
	track_uid[15] = own_sysid();
}

void pause_periodic_cop(Mavlink *inst)
{
	inst->configure_stream_threadsafe("TRACK_IDENTITY", 0.f);
	inst->configure_stream_threadsafe("TARGET", 0.f);
}

void resume_periodic_cop(Mavlink *inst)
{
	inst->configure_stream_threadsafe("TRACK_IDENTITY", 5.f);
	inst->configure_stream_threadsafe("TARGET", 5.f);
}

void for_each_custom(void (*fn)(Mavlink *))
{
	for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
		Mavlink *inst = Mavlink::get_instance(i);

		if (inst != nullptr && inst->get_mode() == Mavlink::MAVLINK_MODE_CUSTOM) {
			fn(inst);
		}
	}
}

// FIONSPACE on this UART stays 0 even while Custom TX is 1.7 kB/s (DMA ioctl).
// send_start() then drops the one-shot. Encode and write() ourselves.
template<typename EncodeFn>
int send_on_custom(EncodeFn encode_fn)
{
	int sent = 0;

	for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
		Mavlink *inst = Mavlink::get_instance(i);

		if (inst == nullptr || inst->get_mode() != Mavlink::MAVLINK_MODE_CUSTOM) {
			continue;
		}

		mavlink_message_t packed{};
		encode_fn(inst, &packed);
		uint8_t raw[MAVLINK_MAX_PACKET_LEN];
		const uint16_t n = mavlink_msg_to_send_buffer(raw, &packed);

		if (!inst->queue_cop_bytes(raw, n)) {
			PX4_ERR("cop: queue failed on instance #%d", i);
			continue;
		}

		PX4_INFO("cop: queued %u B msgid=%u on instance #%d", (unsigned)n, packed.msgid, i);
		sent++;
	}

	if (sent == 0) {
		PX4_ERR("cop: no Custom TX (set MAV_1_MODE=1)");
	}

	return sent;
}

// Companion (speedo_c2_example) publishes /fmu/in/mavlink_m_ack with
// origin_sysid=c2_sysid (their yaml is 200, not MAV_SYS_ID) and
// ack_sysid=own_sysid (1). Do not filter on origin_sysid.
bool wait_for_ack(uint32_t ack_msgid, uint32_t ack_instance, int timeout_ms)
{
	uORB::Subscription ack_sub{ORB_ID(mavlink_m_ack)};
	mavlink_m_ack_s stale{};

	while (ack_sub.update(&stale)) {}

	const hrt_abstime deadline = hrt_absolute_time() + (hrt_abstime)timeout_ms * 1000ULL;

	while (hrt_absolute_time() < deadline) {
		mavlink_m_ack_s ack{};

		if (ack_sub.update(&ack)) {
			if (ack.ack_msgid != ack_msgid) {
				continue;
			}

			if (ack_instance != 0 && ack.ack_instance != ack_instance) {
				continue;
			}

			PX4_INFO("cop: ACK msgid=%u instance=%u result=%u origin=%u ack_sysid=%u",
				 (unsigned)ack.ack_msgid, (unsigned)ack.ack_instance,
				 (unsigned)ack.result, (unsigned)ack.origin_sysid,
				 (unsigned)ack.ack_sysid);
			return true;
		}

		px4_usleep(20000);
	}

	PX4_ERR("cop: no ACK msgid=%u instance=%u in %d ms",
		(unsigned)ack_msgid, (unsigned)ack_instance, timeout_ms);
	return false;
}

void fill_kinematics(int32_t &lat, int32_t &lon, float &alt, float &vx, float &vy, float &vz)
{
	vehicle_global_position_s gpos{};
	vehicle_local_position_s lpos{};
	uORB::Subscription gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription lpos_sub{ORB_ID(vehicle_local_position)};
	gpos_sub.copy(&gpos);
	lpos_sub.copy(&lpos);

	if (gpos.lat_lon_valid) {
		lat = (int32_t)(gpos.lat * 1e7);
		lon = (int32_t)(gpos.lon * 1e7);

	} else {
		lat = INT32_MAX;
		lon = INT32_MAX;
	}

	alt = gpos.alt_valid ? gpos.alt : NAN;
	vx = lpos.v_xy_valid ? lpos.vx : NAN;
	vy = lpos.v_xy_valid ? lpos.vy : NAN;
	vz = lpos.v_z_valid ? lpos.vz : NAN;
}

int publish_handover()
{
	mavlink_m_target_handover_s t{};
	t.timestamp = hrt_absolute_time();
	t.time_usec = wall_time_usec();
	t.detected_first_usec = t.time_usec;
	t.valid_until_usec = 0; // companion nowUs() is ROS UNIX µs; FC hrt is not comparable
	fill_kinematics(t.lat, t.lon, t.alt, t.vx, t.vy, t.vz);
	t.cov_pos_x = t.cov_pos_y = t.cov_pos_z = NAN;
	t.cov_vel_x = t.cov_vel_y = t.cov_vel_z = NAN;
	strncpy(t.target_name, "leseni", sizeof(t.target_name) - 1);
	t.confidence_score = 1.0f;
	t.target_class = MAVLINK_M_TARGET_CLASS_UAS_MULTIROTOR;
	t.target_force = MAVLINK_M_TARGET_FORCE_FOE;
	fill_track_uid(t.track_uid);

	uORB::Publication<mavlink_m_target_handover_s> pub{ORB_ID(mavlink_m_target_handover_send)};
	pub.publish(t);

	mavlink_target_handover_t msg{};
	msg.time_usec = t.time_usec;
	msg.detected_first_usec = t.detected_first_usec;
	msg.valid_until_usec = t.valid_until_usec;
	msg.lat = t.lat;
	msg.lon = t.lon;
	msg.alt = t.alt;
	msg.vx = t.vx;
	msg.vy = t.vy;
	msg.vz = t.vz;
	msg.cov_pos_x = t.cov_pos_x;
	msg.cov_pos_y = t.cov_pos_y;
	msg.cov_pos_z = t.cov_pos_z;
	msg.cov_vel_x = t.cov_vel_x;
	msg.cov_vel_y = t.cov_vel_y;
	msg.cov_vel_z = t.cov_vel_z;
	msg.target_set_id = t.target_set_id;
	memcpy(msg.target_name, t.target_name, sizeof(msg.target_name));
	memcpy(msg.match_media_url, t.match_media_url, sizeof(msg.match_media_url));
	msg.confidence_score = t.confidence_score;
	memcpy(msg.authorization, t.authorization, sizeof(msg.authorization));
	msg.target_class = t.target_class;
	msg.target_force = t.target_force;
	msg.match_media_type = t.match_media_type;
	memcpy(msg.track_uid, t.track_uid, sizeof(msg.track_uid));

	auto send_once = [&]() {
		return send_on_custom([&](Mavlink * inst, mavlink_message_t * packed) {
			mavlink_msg_target_handover_encode(inst->get_system_id(), inst->get_component_id(), packed, &msg);
		});
	};

	for_each_custom(pause_periodic_cop);

	if (send_once() <= 0) {
		for_each_custom(resume_periodic_cop);
		return 1;
	}

	PX4_INFO("cop: TARGET_HANDOVER 53002 uid[15]=%u lat=%d lon=%d alt=%.1f — wait ACK (retry 500 ms)",
		 (unsigned)t.track_uid[15], (int)t.lat, (int)t.lon, (double)t.alt);

	const hrt_abstime deadline = hrt_absolute_time() + 8000000;
	hrt_abstime next_tx = hrt_absolute_time() + 500000;
	uORB::Subscription ack_sub{ORB_ID(mavlink_m_ack)};
	mavlink_m_ack_s stale{};

	while (ack_sub.update(&stale)) {}

	while (hrt_absolute_time() < deadline) {
		mavlink_m_ack_s ack{};

		if (ack_sub.update(&ack) && ack.ack_msgid == MAVLINK_MSG_ID_TARGET_HANDOVER) {
			PX4_INFO("cop: ACK msgid=%u instance=%u result=%u origin=%u ack_sysid=%u",
				 (unsigned)ack.ack_msgid, (unsigned)ack.ack_instance,
				 (unsigned)ack.result, (unsigned)ack.origin_sysid,
				 (unsigned)ack.ack_sysid);
			for_each_custom(resume_periodic_cop);
			return 0;
		}

		if (hrt_absolute_time() >= next_tx) {
			send_once();
			next_tx = hrt_absolute_time() + 500000;
		}

		px4_usleep(20000);
	}

	PX4_ERR("cop: no HANDOVER ACK — interceptor listener mavlink_m_target_handover / ROS /px4_0/fmu/out/mavlink_m_target_handover");
	for_each_custom(resume_periodic_cop);
	return 1;
}

int publish_fires()
{
	mavlink_m_fires_s t{};
	t.timestamp = hrt_absolute_time();
	t.time_usec = wall_time_usec();
	t.time_impact_usec = t.time_usec + 30ULL * 1000000ULL;
	float vx{}, vy{}, vz{};
	fill_kinematics(t.lat, t.lon, t.alt, vx, vy, vz);
	t.effector_id = 1;
	t.sequence = g_cop_seq;
	t.cep_expected = NAN;
	fill_track_uid(t.track_uid);

	uORB::Publication<mavlink_m_fires_s> pub{ORB_ID(mavlink_m_fires)};
	pub.publish(t);

	mavlink_fires_t msg{};
	msg.time_usec = t.time_usec;
	msg.time_impact_usec = t.time_impact_usec;
	msg.lat = t.lat;
	msg.lon = t.lon;
	msg.alt = t.alt;
	msg.effector_id = t.effector_id;
	msg.sequence = t.sequence;
	msg.cep_expected = t.cep_expected;
	msg.prf_code = t.prf_code;
	msg.store_id = t.store_id;
	msg.requested_effect = t.requested_effect;
	msg.munition_class = t.munition_class;
	msg.fuze_mode = t.fuze_mode;
	msg.hob_intent = t.hob_intent;
	msg.fuze_mofa_capable = t.fuze_mofa_capable;
	memcpy(msg.track_uid, t.track_uid, sizeof(msg.track_uid));

	for_each_custom(pause_periodic_cop);
	const int n = send_on_custom([&](Mavlink * inst, mavlink_message_t * packed) {
		mavlink_msg_fires_encode(inst->get_system_id(), inst->get_component_id(), packed, &msg);
	});
	PX4_INFO("cop: FIRES 53020 sequence=%u uid[15]=%u links=%d — wait ACK",
		 (unsigned)t.sequence, (unsigned)t.track_uid[15], n);

	const int ret = (n > 0 && wait_for_ack(MAVLINK_MSG_ID_FIRES, t.sequence, 5000)) ? 0 : 1;
	for_each_custom(resume_periodic_cop);
	return ret;
}

int publish_abort()
{
	mavlink_m_engagement_directive_s t{};
	t.timestamp = hrt_absolute_time();
	t.time_usec = wall_time_usec();
	fill_track_uid(t.track_uid);
	t.sequence = g_cop_seq;
	t.effector_id = 0;
	t.retarget_lat = INT32_MAX;
	t.retarget_lon = INT32_MAX;
	t.retarget_alt = NAN;
	t.directive = MAVLINK_M_ENGAGEMENT_DIRECTIVE_ABORT;
	t.origin_sysid = own_sysid();

	uORB::Publication<mavlink_m_engagement_directive_s> pub{ORB_ID(mavlink_m_engagement_directive)};
	pub.publish(t);

	mavlink_engagement_directive_t msg{};
	msg.time_usec = t.time_usec;
	memcpy(msg.track_uid, t.track_uid, sizeof(msg.track_uid));
	msg.sequence = t.sequence;
	msg.effector_id = t.effector_id;
	msg.retarget_lat = t.retarget_lat;
	msg.retarget_lon = t.retarget_lon;
	msg.retarget_alt = t.retarget_alt;
	msg.directive = t.directive;
	msg.origin_sysid = t.origin_sysid;

	for_each_custom(pause_periodic_cop);
	const int n = send_on_custom([&](Mavlink * inst, mavlink_message_t * packed) {
		mavlink_msg_engagement_directive_encode(inst->get_system_id(), inst->get_component_id(), packed, &msg);
	});
	PX4_INFO("cop: ENGAGEMENT_DIRECTIVE 53023 ABORT(%u) sequence=%u links=%d — wait ACK",
		 (unsigned)t.directive, (unsigned)t.sequence, n);

	const int ret = (n > 0 && wait_for_ack(MAVLINK_MSG_ID_ENGAGEMENT_DIRECTIVE, t.sequence, 5000)) ? 0 : 1;
	for_each_custom(resume_periodic_cop);
	return ret;
}

void cop_usage()
{
	PX4_INFO("mavlink cop handover | fires | abort | workflow");
	PX4_INFO("  handover  TARGET_HANDOVER 53002  then wait MAVLINK_M_ACK 5s");
	PX4_INFO("  fires     FIRES 53020 seq=%u     then wait ACK", (unsigned)g_cop_seq);
	PX4_INFO("  abort     ENGAGEMENT_DIRECTIVE 53023 ABORT then wait ACK");
	PX4_INFO("  workflow  handover ACK, then fires ACK");
}

} // namespace

int Mavlink::cop_command(int argc, char *argv[])
{
	if (argc < 3) {
		cop_usage();
		return 1;
	}

	const char *cmd = argv[2];

	if (!strcmp(cmd, "handover")) {
		return publish_handover();
	}

	if (!strcmp(cmd, "fires")) {
		return publish_fires();
	}

	if (!strcmp(cmd, "abort")) {
		return publish_abort();
	}

	if (!strcmp(cmd, "workflow")) {
		if (publish_handover() != 0) {
			PX4_ERR("cop: workflow stop — no HANDOVER ACK");
			return 1;
		}

		if (publish_fires() != 0) {
			PX4_ERR("cop: workflow stop — no FIRES ACK");
			return 1;
		}

		g_cop_seq++;
		PX4_INFO("cop: workflow done (HANDOVER+FIRES ACKed)");
		return 0;
	}

	cop_usage();
	return 1;
}
