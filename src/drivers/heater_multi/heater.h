/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file heater.h
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 * @author CaFeZn <1837781998@qq.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/heater_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

#define CONTROLLER_PERIOD_DEFAULT    10000
#define TEMPERATURE_TARGET_THRESHOLD 2.5f
#define IMU_INSTANCES_NUMBER 2

class Heater : public ModuleBase<Heater>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Heater();

	virtual ~Heater();

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 * @brief Initializes the class in the same context as the work queue
	 *        and starts the background listener.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Initiates the heater driver work queue, starts a new background task,
	 *        and fails if it is already running.
	 * @return Returns 1 iff start was successful.
	 */
	int start();

private:

	/** Disables the heater (by GPIO). */
	void disable_heater();

	/** Turns the heater on (by GPIO). */
	void heater_on(uint8_t idx);

	/** Turns the heater off (by GPIO). */
	void heater_off(uint8_t idx);

	void initialize();

	/** Enables / configures the heater (by GPIO). */
	void initialize_heater_io();

	/** @brief Called once to initialize uORB topics. */
	bool initialize_topics();

	void publish_status();

	/** @brief Calculates the heater element on/off time and schedules the next cycle. */
	void Run() override;

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	void heater_set(uint8_t idx, bool enable);

	/** Work queue struct for the scheduler. */
	static struct work_s _work;

	/** File descriptor for PX4IO for heater ioctl's */
#if defined(PX4IO_HEATER_ENABLED)
	int _io_fd {-1};
#endif

	bool _heater_initialized[IMU_INSTANCES_NUMBER] = {false, false};
   	bool _heater_on[IMU_INSTANCES_NUMBER] = {false, false};
   	bool _temperature_target_met[IMU_INSTANCES_NUMBER] = {false, false};

	int _controller_period_usec = CONTROLLER_PERIOD_DEFAULT;
    	int _controller_time_on_usec[IMU_INSTANCES_NUMBER] = {0, 0};

	float _integrator_value[IMU_INSTANCES_NUMBER] = {0.0f, 0.0f};
   	float _proportional_value[IMU_INSTANCES_NUMBER] = {0.0f, 0.0f};

	uORB::PublicationMulti<heater_status_s> _heater_status_pub[IMU_INSTANCES_NUMBER]{
		{ORB_ID(heater_status)},
		{ORB_ID(heater_status)}
		};


	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _sensor_accel_sub[IMU_INSTANCES_NUMBER] {{ORB_ID(sensor_accel)}, {ORB_ID(sensor_accel)}};

	uint32_t _sensor_device_id[IMU_INSTANCES_NUMBER] {0, 0}; // 记录 IMU 1/2 实际的 device_id
	uint8_t _current_imu_param_index[IMU_INSTANCES_NUMBER] {0, 1}; // 记录 IMU 1/2 对应参数数组的索引

	float _temperature_last[IMU_INSTANCES_NUMBER] {NAN, NAN};

	// 参数句柄
	param_t _param_imu_temp_f[IMU_INSTANCES_NUMBER]{PARAM_INVALID, PARAM_INVALID};		// 前馈句柄
	param_t _param_imu_temp_i[IMU_INSTANCES_NUMBER]{PARAM_INVALID, PARAM_INVALID};		// KP句柄
	param_t _param_imu_temp_p[IMU_INSTANCES_NUMBER]{PARAM_INVALID, PARAM_INVALID};		// KI句柄
	param_t _param_imu_temp[IMU_INSTANCES_NUMBER]{PARAM_INVALID, PARAM_INVALID};		// 设定温度句柄
	param_t _param_sens_temp_id[IMU_INSTANCES_NUMBER]{PARAM_INVALID, PARAM_INVALID};	// IMU ID句柄

	// 参数缓存值（在update_params中更新）
	float _param_cached_temp_f[IMU_INSTANCES_NUMBER]{0.05f, 0.05f};		// 前馈 缓存值
	float _param_cached_temp_i[IMU_INSTANCES_NUMBER]{0.025f, 0.025f};	// KP 缓存值
	float _param_cached_temp_p[IMU_INSTANCES_NUMBER]{1.0f, 1.0f};		// KI 缓存值
	float _param_cached_temp[IMU_INSTANCES_NUMBER]{45.0f, 45.0f};		// 设定温度 缓存值
	int32_t _param_cached_sens_id[IMU_INSTANCES_NUMBER]{0, 0};		// IMU ID 缓存值

};

