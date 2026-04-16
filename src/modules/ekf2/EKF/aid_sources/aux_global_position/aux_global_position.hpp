/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#ifndef EKF_AUX_GLOBAL_POSITION_HPP
#define EKF_AUX_GLOBAL_POSITION_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/aux_global_position.h>

class Ekf;
class AgpSource;

class AuxGlobalPosition : public ModuleParams
{
public:
	static constexpr int MAX_AGP_IDS = MAX_AGP_INSTANCES;

	AuxGlobalPosition();
	~AuxGlobalPosition() override;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void paramsUpdated();

	float testRatioFiltered() const;
	bool anySourceFusing() const;
	uint8_t sourceFusingBitmask() const;

private:
	int32_t getAgpParamInt32(const char *param_suffix, int instance) const;
	bool setAgpParamInt32(const char *param_suffix, int instance, int32_t value);

	int32_t getIdParam(int instance);
	void setIdParam(int instance, int32_t sensor_id);

	int mapSensorIdToSlot(int32_t sensor_id);

	int32_t _id_param_values[MAX_AGP_IDS] {};
	AgpSource *_sources[MAX_AGP_IDS] {};
	int _n_sources{0};

	uORB::Subscription _agp_sub[MAX_AGP_IDS] {};

	int8_t _instance_slot_map[MAX_AGP_IDS] {};
};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME

#endif // !EKF_AUX_GLOBAL_POSITION_HPP
