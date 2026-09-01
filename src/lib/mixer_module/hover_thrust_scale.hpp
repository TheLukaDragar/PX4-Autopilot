/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <mathlib/math/Limits.hpp>
#include <parameters/param.h>

/**
 * Scale MPC_THR_HOVER into the current actuator 0..1 frame when DSHOT_MOT_LIM < 1.
 *
 * Convention: MPC_THR_HOVER is the hover fraction of *full* (unscaled) ESC output.
 * With DSHOT_MOT_LIM, mixer 1.0 only reaches that fraction of full DShot, so the
 * hover fraction used by controllers / HTE is MPC_THR_HOVER / DSHOT_MOT_LIM.
 * If DSHOT_MOT_LIM is absent or 1.0, the value is unchanged.
 */
inline float scaleHoverThrustForMotorLimit(float mpc_thr_hover)
{
	float lim = 1.f;
	const param_t p = param_find("DSHOT_MOT_LIM");

	if (p != PARAM_INVALID) {
		param_get(p, &lim);
	}

	lim = math::constrain(lim, 0.1f, 1.f);
	return mpc_thr_hover / lim;
}
