#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_beep_control.h>
#include <uORB/topics/vehicle_command.h>

#include "esc_beep_sequences.h"

class EscBeep : public ModuleBase<EscBeep>, public px4::ScheduledWorkItem
{
public:
	EscBeep();
	~EscBeep() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

private:
	bool init();
	void Run() override;

	uint8_t  _active_beep_id{0};
	uint8_t  _current_step{0};
	uint8_t  _current_repeat{0};
	bool     _is_playing{false};

	uORB::Publication<vehicle_command_s> _cmd_pub{ORB_ID(vehicle_command)};
	uORB::SubscriptionCallbackWorkItem _beep_sub{this, ORB_ID(esc_beep_control)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
};
