@###############################################
@#
@# EmPy template
@#
@###############################################
@# Generates publications and subscriptions for XRCE
@#
@# Context:
@#  - msgs (List) list of all RTPS messages
@#  - topics (List) list of all topic names
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import os


}@

#include <utilities.hpp>

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/uORB.h>
@[for include in type_includes]@
#include <uORB/ucdr/@(include).h>
#include <uORB/topics/@(include).h>
@[end for]@

#define UXRCE_DEFAULT_POLL_INTERVAL_MS 10

typedef bool (*UcdrSerializeMethod)(const void* data, ucdrBuffer& buf, int64_t time_offset);

static constexpr int max_topic_size = 512;
@[    for pub in publications]@
static_assert(sizeof(@(pub['simple_base_type'])_s) <= max_topic_size, "topic too large, increase max_topic_size");
@[    end for]@



// SFINAE to use R::MESSAGE_VERSION if it exists, and 0 otherwise
template <typename R>
class MessageVersionHelper
{
	template <typename C>
	static constexpr uint32_t get(decltype(&C::MESSAGE_VERSION)) { return C::MESSAGE_VERSION; }
	template <typename C>
	static constexpr uint32_t get(...) { return 0; }
public:
	static constexpr uint32_t m = get<R>(0);
};

template <typename T>
static constexpr uint32_t get_message_version() {
	return MessageVersionHelper<T>::m;
}

struct SendSubscription {
	const struct orb_metadata *orb_meta;
	uxrObjectId data_writer;
	const char* dds_type_name;
	const char* topic;
	uint32_t message_version;
	uint32_t topic_size;
	UcdrSerializeMethod ucdr_serialize_method;
	uint64_t publish_interval_ms;
	bool event_driven;
};

// Callback for event-driven topics
class EventDrivenCallback : public uORB::SubscriptionCallback
{
public:
	EventDrivenCallback(void *owner, const orb_metadata *meta) :
		uORB::SubscriptionCallback(meta), _owner(owner), _callback_count(0) {}

	void call() override {
		_callback_count++;
		// Notify owner that event-driven topic updated
		if (_owner) {
			px4_sem_post((px4_sem_t*)_owner);
		}
	}

	uint32_t get_callback_count() const { return _callback_count; }

private:
	void *_owner;
	uint32_t _callback_count;
};

// Subscribers for messages to send
struct SendTopicsSubs {
	SendSubscription send_subscriptions[@(len(publications))] = {
@[    for pub in publications]@
			{ ORB_ID(@(pub['topic_simple'])),
			  uxr_object_id(0, UXR_INVALID_ID),
			  "@(pub['dds_type'])",
			  "@(pub['topic'])",
			  get_message_version<@(pub['simple_base_type'])_s>(),
			  ucdr_topic_size_@(pub['simple_base_type'])(),
			  &ucdr_serialize_@(pub['simple_base_type']),
			  static_cast<uint64_t>(@('true' if pub.get('event_driven', False) else 'false') ? 0 : ((@(pub.get('rate_limit', 0)) > 0) ? (1e3 / @(pub.get('rate_limit', 1e3))) : UXRCE_DEFAULT_POLL_INTERVAL_MS)),
			  @('true' if pub.get('event_driven', False) else 'false'),
			},
@[    end for]@
	};

	px4_pollfd_struct_t fds[@(len(publications))] {};
	EventDrivenCallback *callbacks[@(len(publications))] {};
	px4_sem_t event_sem;

	uint32_t num_payload_sent{};

	bool init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void update_event_driven(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace);
	void reset();
	bool has_event_driven_topics() const;
	int get_poll_timeout_ms() const;
	void print_statistics() const;
};

bool SendTopicsSubs::init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace) {
	bool ret = true;

	// Initialize semaphore for event-driven topics
	px4_sem_init(&event_sem, 0, 0);

	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (send_subscriptions[idx].event_driven) {
			// Event-driven: only use callback subscription, no poll
			fds[idx].fd = -1;
			fds[idx].events = 0;

			callbacks[idx] = new EventDrivenCallback(&event_sem, send_subscriptions[idx].orb_meta);
			if (callbacks[idx]) {
				callbacks[idx]->subscribe();
				callbacks[idx]->registerCallback();
			}
		} else {
			// Rate-limited: only use poll subscription, no callback
			if (fds[idx].events == 0) {
				fds[idx].fd = orb_subscribe(send_subscriptions[idx].orb_meta);
				fds[idx].events = POLLIN;
				orb_set_interval(fds[idx].fd, send_subscriptions[idx].publish_interval_ms);
			}
			callbacks[idx] = nullptr;
		}

		if (!create_data_writer(session, reliable_out_stream_id, participant_id, static_cast<ORB_ID>(send_subscriptions[idx].orb_meta->o_id), client_namespace, send_subscriptions[idx].topic,
								   send_subscriptions[idx].message_version, 0,
								   send_subscriptions[idx].dds_type_name, send_subscriptions[idx].data_writer)) {
			ret = false;
		}
	}
	return ret;
}

void SendTopicsSubs::reset() {
	num_payload_sent = 0;
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		send_subscriptions[idx].data_writer = uxr_object_id(0, UXR_INVALID_ID);

		// Only unsubscribe valid poll subscriptions
		if (fds[idx].fd >= 0) {
			orb_unsubscribe(fds[idx].fd);
			fds[idx].fd = -1;
		}

		// Clean up callbacks
		if (callbacks[idx]) {
			callbacks[idx]->unregisterCallback();
			delete callbacks[idx];
			callbacks[idx] = nullptr;
		}
	}
	px4_sem_destroy(&event_sem);
};

void SendTopicsSubs::update(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us

	alignas(sizeof(uint64_t)) char topic_data[max_topic_size];

	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		// Skip event-driven topics (they use callback path, not poll path)
		if (send_subscriptions[idx].event_driven) {
			continue;
		}

		if (fds[idx].revents & POLLIN) {
			// Topic updated, copy data and send
			orb_copy(send_subscriptions[idx].orb_meta, fds[idx].fd, &topic_data);

			if (send_subscriptions[idx].data_writer.id != UXR_INVALID_ID) {

				ucdrBuffer ub;
				uint32_t topic_size = send_subscriptions[idx].topic_size;
				if (uxr_prepare_output_stream(session, best_effort_stream_id, send_subscriptions[idx].data_writer, &ub, topic_size) != UXR_INVALID_REQUEST_ID) {
					send_subscriptions[idx].ucdr_serialize_method(&topic_data, ub, time_offset_us);
					// TODO: fill up the MTU and then flush, which reduces the packet overhead
					uxr_flash_output_streams(session);
					num_payload_sent += topic_size;

				} else {
					//PX4_ERR("Error uxr_prepare_output_stream UXR_INVALID_REQUEST_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
				}

			} else {
				//PX4_ERR("Error UXR_INVALID_ID %s", send_subscriptions[idx].subscription.get_topic()->o_name);
			}

		}
	}
}

void SendTopicsSubs::update_event_driven(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId best_effort_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
	int64_t time_offset_us = session->time_offset / 1000; // ns -> us
	alignas(sizeof(uint64_t)) char topic_data[max_topic_size];

	// Process all event-driven topics that have updates
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (send_subscriptions[idx].event_driven && callbacks[idx]) {
			// Check if there are updates available
			while (callbacks[idx]->updated()) {
				if (callbacks[idx]->copy(&topic_data) && send_subscriptions[idx].data_writer.id != UXR_INVALID_ID) {
					ucdrBuffer ub;
					uint32_t topic_size = send_subscriptions[idx].topic_size;
					if (uxr_prepare_output_stream(session, best_effort_stream_id, send_subscriptions[idx].data_writer, &ub, topic_size) != UXR_INVALID_REQUEST_ID) {
						send_subscriptions[idx].ucdr_serialize_method(&topic_data, ub, time_offset_us);
						uxr_flash_output_streams(session);
						num_payload_sent += topic_size;
					}
				}
			}
		}
	}
}

bool SendTopicsSubs::has_event_driven_topics() const
{
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (send_subscriptions[idx].event_driven) {
			return true;
		}
	}
	return false;
}

int SendTopicsSubs::get_poll_timeout_ms() const
{
	// If we have event-driven topics, use semaphore timedwait instead of poll timeout
	// Otherwise use default 10ms poll
	return has_event_driven_topics() ? 0 : 10;
}

void SendTopicsSubs::print_statistics() const
{
	PX4_INFO("=== Event-Driven Statistics ===");
	for (unsigned idx = 0; idx < sizeof(send_subscriptions)/sizeof(send_subscriptions[0]); ++idx) {
		if (send_subscriptions[idx].event_driven && callbacks[idx]) {
			PX4_INFO("  %s: %" PRIu32 " callbacks", send_subscriptions[idx].orb_meta->o_name,
			         callbacks[idx]->get_callback_count());
		}
	}
}

// Publishers for received messages
struct RcvTopicsPubs {
@[    for sub in subscriptions]@
	uORB::Publication<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
@[    end for]@

@[    for sub in subscriptions_multi]@
@[        if sub.get('route_field')]@
	uORB::PublicationMulti<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pubs[@(sub['max_instances'])] {
@[            for idx in range(sub['max_instances'])]@
		{ORB_ID(@(sub['topic_simple']))}@('' if idx == sub['max_instances']-1 else ',')
@[            end for]@
	};
	// Maps route_field values (arbitrary, not bounded to [0, max_instances)) to uORB instance indices
	struct {
		uint32_t assigned_ids[@(sub['max_instances'])] {};
		uint8_t num_assigned {0};
	} @(sub['topic_simple'])_demux;
@[        else]@
	uORB::PublicationMulti<@(sub['simple_base_type'])_s> @(sub['topic_simple'])_pub{ORB_ID(@(sub['topic_simple']))};
@[        end if]@
@[    end for]@

	uint32_t num_payload_received{};

	bool init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace);
};

static void on_topic_update(uxrSession *session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id,
		     struct ucdrBuffer *ub, uint16_t length, void *args)
{
	RcvTopicsPubs *pubs = (RcvTopicsPubs *)args;
	const int64_t time_offset_us = session->time_offset / 1000; // ns -> us
	pubs->num_payload_received += length;

	switch (object_id.id) {
@[    for idx, sub in enumerate(subscriptions)]@
	case @(idx)+ (65535U / 32U) + 1: {
			@(sub['simple_base_type'])_s data;

			if (ucdr_deserialize_@(sub['simple_base_type'])(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(@(sub['simple_base_type'])), data);
				pubs->@(sub['topic_simple'])_pub.publish(data);
			}
		}
		break;

@[    end for]@
@[    for idx, sub in enumerate(subscriptions_multi)]@
	case @(idx + len(subscriptions))+ (65535U / 32U) + 1: {
			@(sub['simple_base_type'])_s data;

			if (ucdr_deserialize_@(sub['simple_base_type'])(*ub, data, time_offset_us)) {
				//print_message(ORB_ID(@(sub['simple_base_type'])), data);
@[        if sub.get('route_field')]@
				int instance = -1;

				for (uint8_t i = 0; i < pubs->@(sub['topic_simple'])_demux.num_assigned; i++) {
					if (pubs->@(sub['topic_simple'])_demux.assigned_ids[i] == data.@(sub['route_field'])) {
						instance = i;
						break;
					}
				}

				if (instance < 0 && pubs->@(sub['topic_simple'])_demux.num_assigned < @(sub['max_instances'])) {
					instance = pubs->@(sub['topic_simple'])_demux.num_assigned++;
					pubs->@(sub['topic_simple'])_demux.assigned_ids[instance] = data.@(sub['route_field']);
				}

				if (instance >= 0) {
					pubs->@(sub['topic_simple'])_pubs[instance].publish(data);
				}
@[        else]@
				pubs->@(sub['topic_simple'])_pub.publish(data);
@[        end if]@
			}
		}
		break;

@[    end for]@

	default:
		PX4_ERR("unknown object id: %i", object_id.id);
		break;
	}
}

bool RcvTopicsPubs::init(uxrSession *session, uxrStreamId reliable_out_stream_id, uxrStreamId reliable_in_stream_id, uxrStreamId best_effort_in_stream_id, uxrObjectId participant_id, const char *client_namespace)
{
@[    for idx, sub in enumerate(subscriptions)]@
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(@(sub['simple_base_type']))) * 2; // use a bit larger queue size than internal
			uint32_t message_version = get_message_version<@(sub['simple_base_type'])_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, @(idx), client_namespace, "@(sub['topic'])", message_version, "@(sub['dds_type'])", queue_depth);
	}
@[    end for]@
@[    for idx, sub in enumerate(subscriptions_multi)]@
	{
			uint16_t queue_depth = orb_get_queue_size(ORB_ID(@(sub['topic_simple']))) * @(sub.get('max_instances', 2)); // scale queue for multiple sources
			uint32_t message_version = get_message_version<@(sub['simple_base_type'])_s>();
			create_data_reader(session, reliable_out_stream_id, best_effort_in_stream_id, participant_id, @(idx + len(subscriptions)), client_namespace, "@(sub['topic'])", message_version, "@(sub['dds_type'])", queue_depth);
	}
@[    end for]@

	uxr_set_topic_callback(session, on_topic_update, this);

	return true;
}
