#pragma once

#include <cstdint>
#include <drivers/drv_dshot.h>

struct BeepStep {
	float    beacon_type;    // DShot tone command (DSHOT_CMD_TONE1..TONE8 = 22..29)
	float    motor_function; // MAVLink motor function (1=Motor1, 2=Motor2, ...)
	uint32_t duration_us;    // delay before next step (microseconds)
};

struct BeepSequence {
	const BeepStep *steps;
	uint8_t         num_steps;
	uint8_t         repeat_count; // 0 = play once, N = play N+1 times total
};

// Motor function for all sequences — Motor2 (MAVLink standard index 2)
static constexpr float BEEP_MOTOR = 2.0f;

// Helper macro to cast enum to float for vehicle_command param1
#define TONE(t) static_cast<float>(t)

// --- placeholder sequences (define real tunes here) ---

static const BeepStep steps_placeholder1[] = {
	{TONE(DSHOT_CMD_TONE1), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE2), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE3), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE4), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE5), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE6), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE7), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 200000},
};

static const BeepStep steps_placeholder2[] = {
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 150000},
	{TONE(DSHOT_CMD_TONE1), BEEP_MOTOR, 150000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 150000},
	{TONE(DSHOT_CMD_TONE1), BEEP_MOTOR, 150000},
};

static const BeepStep steps_placeholder3[] = {
	{TONE(DSHOT_CMD_TONE1), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE4), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 100000},
};

static const BeepStep steps_placeholder4[] = {
	{TONE(DSHOT_CMD_TONE4), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE4), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE3), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE4), BEEP_MOTOR, 200000},
	{TONE(DSHOT_CMD_TONE8), BEEP_MOTOR, 200000},
};

static const BeepSequence beep_sequences[] = {
	/* 0: STOP         */ {nullptr,            0,                                                              0},
	/* 1: PLACEHOLDER1 */ {steps_placeholder1, sizeof(steps_placeholder1) / sizeof(steps_placeholder1[0]), 0},
	/* 2: PLACEHOLDER2 */ {steps_placeholder2, sizeof(steps_placeholder2) / sizeof(steps_placeholder2[0]), 1},
	/* 3: PLACEHOLDER3 */ {steps_placeholder3, sizeof(steps_placeholder3) / sizeof(steps_placeholder3[0]), 0},
	/* 4: PLACEHOLDER4 */ {steps_placeholder4, sizeof(steps_placeholder4) / sizeof(steps_placeholder4[0]), 2},
};

static constexpr uint8_t NUM_BEEP_SEQUENCES = sizeof(beep_sequences) / sizeof(beep_sequences[0]);
