#include <px4_platform_common/log.h>

#include "esc_beep.hpp"

ModuleBase::Descriptor EscBeep::desc{task_spawn, custom_command, print_usage};

EscBeep::EscBeep() :
	ScheduledWorkItem("esc_beep", px4::wq_configurations::lp_default)
{
}

bool EscBeep::init()
{
	if (!_beep_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void EscBeep::Run()
{
	if (should_exit()) {
		_beep_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// Always check arm state first — abort any in-progress sequence immediately if armed
	actuator_armed_s armed{};
	_armed_sub.copy(&armed);

	if (armed.armed) {
		if (_is_playing) {
			PX4_WARN("esc_beep: armed mid-sequence, stopping");
			_is_playing = false;
		}

		// drain any queued requests so they don't fire immediately on disarm
		esc_beep_control_s discard{};

		while (_beep_sub.update(&discard)) {}

		return;
	}

	esc_beep_control_s beep_msg;

	if (_beep_sub.update(&beep_msg)) {
		if (beep_msg.beep_id == esc_beep_control_s::BEEP_ID_STOP) {
			_is_playing = false;

		} else if (beep_msg.beep_id < NUM_BEEP_SEQUENCES) {
			_active_beep_id = beep_msg.beep_id;
			_current_step   = 0;
			_current_repeat = 0;
			_is_playing     = true;
		}
	}

	if (!_is_playing) {
		return;
	}

	const BeepSequence &seq          = beep_sequences[_active_beep_id];
	const BeepStep     &current_step = seq.steps[_current_step];

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command   = vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR;
	cmd.param1    = current_step.beacon_type;
	cmd.param5    = current_step.motor_function;
	_cmd_pub.publish(cmd);

	_current_step++;

	if (_current_step >= seq.num_steps) {
		_current_step = 0;
		_current_repeat++;
	}

	if (_current_repeat > seq.repeat_count) {
		_is_playing = false;

	} else {
		ScheduleDelayed(current_step.duration_us);
	}
}

int EscBeep::task_spawn(int argc, char *argv[])
{
	EscBeep *instance = new EscBeep();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!instance->init()) {
		delete instance;
		return PX4_ERROR;
	}

	desc.task_id = task_id_is_work_queue;
	return PX4_OK;
}

int EscBeep::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("esc_beep", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int esc_beep_main(int argc, char *argv[])
{
	if (argc >= 3 && !strcmp(argv[1], "test")) {
		uint8_t id = (uint8_t)atoi(argv[2]);

		uORB::Publication<esc_beep_control_s> test_pub{ORB_ID(esc_beep_control)};
		esc_beep_control_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.beep_id   = id;
		test_pub.publish(msg);

		PX4_INFO("Test: sent beep_id %d", id);
		return 0;
	}

	return ModuleBase::main(EscBeep::desc, argc, argv);
}
