#pragma once

#include <cstdint>

struct BeepStep {
	float beacon_type;   // param1: beacon type (1=beacon1, 9=beacon2, 7=beacon3, 0=silence)
	float motor_function; // param5: motor function
	uint32_t duration_us; // how long before next step
};

struct BeepSequence {
	const BeepStep *steps;
	uint8_t num_steps;
	uint8_t repeat_count; // 0 = play once, N = play N+1 times
};

static const BeepStep steps_placeholder1[] = {
	{22.0f, 2.0f, 200000},
	{23.0f, 2.0f, 200000},
	{24.0f, 2.0f, 200000},
	{25.0f, 2.0f, 200000},
	{26.0f, 2.0f, 200000},
	{27.0f, 2.0f, 200000},
	{28.0f, 2.0f, 200000},
	{29.0f, 2.0f, 200000},
};

static const BeepStep steps_placeholder2[] = {
	{29.0f, 2.0f, 150000},
	{22.0f, 2.0f, 150000},
	{29.0f, 2.0f, 150000},
	{22.0f, 2.0f, 150000},
};

static const BeepStep steps_placeholder3[] = {
	{22.0f, 2.0f, 200000},
	{25.0f, 2.0f, 200000},
	{29.0f, 2.0f, 100000},
};

static const BeepStep steps_placeholder4[] = {
	{25.0f, 2.0f, 200000},
	{29.0f, 2.0f, 200000},
	{25.0f, 2.0f, 200000},
	{24.0f, 2.0f, 200000},
	{29.0f, 2.0f, 200000},
	{25.0f, 2.0f, 200000},
	{29.0f, 2.0f, 200000},
};


static const BeepSequence beep_sequences[] = {
	/* 0: STOP         */ {nullptr,               0, 0},
	/* 1: PLACEHOLDER1 */ {steps_placeholder1,    sizeof(steps_placeholder1) / sizeof(steps_placeholder1[0]), 0},
	/* 2: PLACEHOLDER2 */ {steps_placeholder2,    sizeof(steps_placeholder2) / sizeof(steps_placeholder2[0]), 1},
	/* 3: PLACEHOLDER3 */ {steps_placeholder3,    sizeof(steps_placeholder3) / sizeof(steps_placeholder3[0]), 0},
	/* 4: PLACEHOLDER4 */ {steps_placeholder4,    sizeof(steps_placeholder4) / sizeof(steps_placeholder4[0]), 2},
};

static constexpr uint8_t NUM_BEEP_SEQUENCES = sizeof(beep_sequences) / sizeof(beep_sequences[0]);
