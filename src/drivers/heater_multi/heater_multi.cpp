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
 * @file heater.cpp
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 * @author Jacob Crabill <jacob@flyvoly.com>
 * @author CaFeZn <1837781998@qq.com>
 */

#include "heater.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>

#if defined(BOARD_USES_PX4IO_VERSION) and defined(PX4IO_HEATER_ENABLED)
// Use direct calls to turn GPIO pin on/off
#  ifndef GPIO_HEATER_OUTPUT
#  error "To use the heater driver, the board_config.h must define and initialize GPIO_HEATER_OUTPUT"
#  endif
#  define HEATER_GPIO
#endif

// Multiple IMU heating pins (must be defined in board_config.h)
#ifndef GPIO_HEATER1
#  error "Please define GPIO_HEATER1 in board_config.h"
#endif
#ifndef GPIO_HEATER2
#  error "Please define GPIO_HEATER2 in board_config.h"
#endif

Heater::Heater() :
	ModuleParams(nullptr),
	ScheduledWorkItem("HEATER_MULTI", px4::wq_configurations::lp_default)
{
	_param_imu_temp_f[0] = param_find("SENS_IMU_TEMP_F1");
	_param_imu_temp_i[0] = param_find("SENS_IMU_TEMP_I1");
	_param_imu_temp_p[0] = param_find("SENS_IMU_TEMP_P1");
	_param_imu_temp[0] = param_find("SENS_IMU_TEMP1");
	_param_sens_temp_id[0] = param_find("SENS_IMU_ID1");

	_param_imu_temp_f[1] = param_find("SENS_IMU_TEMP_F2");
	_param_imu_temp_i[1] = param_find("SENS_IMU_TEMP_I2");
	_param_imu_temp_p[1] = param_find("SENS_IMU_TEMP_P2");
	_param_imu_temp[1] = param_find("SENS_IMU_TEMP2");
	_param_sens_temp_id[1] = param_find("SENS_IMU_ID2");

	// 初始化参数缓存值
	update_params(true);

	for (uint8_t i = 0; i < 2; i++)
	{
		_heater_status_pub[i].advertise();
	}
}

Heater::~Heater()
{
	disable_heater();
}

int Heater::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}
	if (strcmp(argv[1], "starts") == 0) {
			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			return start_command_base(argc - 1, argv + 1);
		}

	return print_usage("Unrecognized command.");
}

void Heater::disable_heater()
{
	px4_arch_configgpio(GPIO_HEATER1);
	px4_arch_gpiowrite(GPIO_HEATER1, false);

	px4_arch_configgpio(GPIO_HEATER2);
	px4_arch_gpiowrite(GPIO_HEATER2, false);
}

void Heater::initialize_heater_io()
{
	// Initialize heater to off state.
	px4_arch_configgpio(GPIO_HEATER1);
	px4_arch_gpiowrite(GPIO_HEATER1, false);

	px4_arch_configgpio(GPIO_HEATER2);
	px4_arch_gpiowrite(GPIO_HEATER2, false);
}

void Heater::heater_on(uint8_t idx)
{
    heater_set(idx, true);
}

void Heater::heater_off(uint8_t idx)
{
    heater_set(idx, false);
}


bool Heater::initialize_topics()
{
    uint8_t found_imus = 0;

    // 遍历所有可能的 uORB 实例（0 和 1）
    for (uint8_t i = 0; i < IMU_INSTANCES_NUMBER; i++) {

        // 临时订阅来检查实例 i 的数据
        uORB::SubscriptionData<sensor_accel_s> temp_sub{ORB_ID(sensor_accel), i};

        if (temp_sub.get().timestamp != 0 &&
            temp_sub.get().device_id != 0 &&
            PX4_ISFINITE(temp_sub.get().temperature)) {

            // 检查这个 IMU 实例的 device_id 是否与配置的任何 IMU ID 参数匹配 (SENS_IMU_ID1 或 SENS_IMU_ID2)
            for (uint8_t param_idx = 0; param_idx < IMU_INSTANCES_NUMBER; param_idx++) {

                if ((uint32_t)_param_cached_sens_id[param_idx] == temp_sub.get().device_id) {

                    // 找到匹配的 IMU，使用 found_imus 作为数组索引
                    const uint8_t current_idx = found_imus;

                    // 将 **第 found_imus 个订阅器** 锁定到 uORB 实例 i
                    // 注意：这里的 i 是 uORB 实例 ID (0 或 1)，current_idx 是我们内部的 IMU 索引 (0 或 1)
                    _sensor_accel_sub[current_idx].ChangeInstance(i);

                    _sensor_device_id[current_idx] = temp_sub.get().device_id;

                    // 记录 IMU 实例 i 使用的参数索引 param_idx
                    _current_imu_param_index[current_idx] = param_idx;

                    PX4_INFO("IMU %u (Device ID: %lu) found on uORB instance %u, using parameter set %u",
                        current_idx + 1, _sensor_device_id[current_idx], i, param_idx + 1);

                    found_imus++;

                    // 如果找到了两个 IMU，则完成初始化
                    if (found_imus == IMU_INSTANCES_NUMBER) {
                        initialize_heater_io();
                        return true;
                    }
                }
            }
        }
    }

    // 如果两个 IMU ID 都是 0，则假定只有一个 IMU，并且它已在 IMU 0/1 上找到
    if (found_imus > 0) {
        initialize_heater_io();
        return true;
    }

    return false;
}

void Heater::Run()
{
    if (should_exit()) {
        exit_and_cleanup();
        return;
    }

    update_params();

    // 检查是否所有需要的 IMU 都已找到 (通过检查 device_id)
    if (_sensor_device_id[0] == 0 || _sensor_device_id[1] == 0) {
        if (!initialize_topics()) {
            // 如果至少有一个 IMU 没找到，延迟 1 秒后重试
            ScheduleDelayed(1_s);
            return;
        }
    }

    // --- 开始控制循环 ---
    // 使用 for 循环同时控制 IMU 0 和 IMU 1
    for (uint8_t i = 0; i < IMU_INSTANCES_NUMBER; i++) {

        // 获取当前 IMU 对应的 PID 参数索引
        const uint8_t param_idx = _current_imu_param_index[i];

        // PID 控制器核心逻辑
        if (_heater_on[i]) {
            // 关闭加热器，
            _heater_on[i] = false;
            heater_off(i); // 关闭 IMU i 对应的加热器

            // 由于 ScheduleDelayed 只能设置一次，我们在这里**不能**使用它来调度 OFF TIME。
            // 相反，我们将 ScheduleDelayed 留在循环之外，使用最短的 ON/OFF 时间来调度下一次 Run。

        } else {
            // 目标：计算 PID 并尝试打开加热器 (ON TIME)

            sensor_accel_s sensor_accel;

            // 检查当前 IMU 订阅是否有新数据
            if (_sensor_accel_sub[i].update(&sensor_accel)) {

                // 确保温度数据有效
                if (PX4_ISFINITE(sensor_accel.temperature)) {
                    // 使用缓存的目标温度
                    float temperature_delta = _param_cached_temp[param_idx] - sensor_accel.temperature;
                    _temperature_last[i] = sensor_accel.temperature;

                    // 使用缓存的 PID 系数进行计算
                    _proportional_value[i] = temperature_delta * _param_cached_temp_p[param_idx];

                    _integrator_value[i] += temperature_delta * _param_cached_temp_i[param_idx];

                    _integrator_value[i] = math::constrain(_integrator_value[i], -0.25f, 0.25f);

                    _controller_time_on_usec[i] = static_cast<int>((_param_cached_temp_f[param_idx] + _proportional_value[i] +
                                       _integrator_value[i]) * static_cast<float>(_controller_period_usec));

                    _controller_time_on_usec[i] = math::constrain(_controller_time_on_usec[i], 0, _controller_period_usec);

                    if (fabsf(temperature_delta) < TEMPERATURE_TARGET_THRESHOLD) {
                        _temperature_target_met[i] = true;
                    } else {
                        _temperature_target_met[i] = false;
                    }

                    // 打开加热器 (如果需要开启时间大于 0)
                    if (_controller_time_on_usec[i] > 0) {
                        _heater_on[i] = true;
                        heater_on(i); // 打开 IMU i 对应的加热器
                    } else {
                        // 如果计算出的 ON TIME 是 0，确保它是 OFF 的
                        heater_off(i);
                    }

                } else {
                    // 如果温度无效，确保它是 OFF 的
                    heater_off(i);
                }
            } else {
                // 如果没有新数据，且它本来是 ON 的，则跳过本次循环，保持状态不变。
                // 如果它本来是 OFF 的，则跳过本次循环，保持状态不变。
            }
        }
    } // --- 控制循环结束 ---

    // --- 调度下一个 Run() ---

    // 因为我们是周期性控制 (PWM 占空比)，我们必须找到两个 IMU 中最短的 ON TIME 或 OFF TIME
    // 来调度下一次 Run()，以确保下一个最短的事件能够被及时处理。

    int next_run_delay_usec = _controller_period_usec; // 默认是最大周期

    for (uint8_t i = 0; i < IMU_INSTANCES_NUMBER; i++) {
        if (_sensor_device_id[i] != 0) { // 仅对已找到的 IMU 计算调度
            int delay = 0;
            if (_heater_on[i]) {
                // 如果 IMU i 当前是 ON，下一个事件是 OFF，延迟时间 = ON time
                delay = _controller_time_on_usec[i];
            } else {
                // 如果 IMU i 当前是 OFF，下一个事件是 ON，延迟时间 = 周期 - ON time
                delay = _controller_period_usec - _controller_time_on_usec[i];
            }

            // 找到最短的延迟时间
            if (delay > 0 && delay < next_run_delay_usec) {
                next_run_delay_usec = delay;
            }
        }
    }

    // 确保最小延迟为 1ms，以防止系统过载
    if (next_run_delay_usec < 1000) {
        next_run_delay_usec = 1000;
    }

    ScheduleDelayed(next_run_delay_usec);

    publish_status();
}

void Heater::publish_status()
{
	// 遍历两个 IMU，为每个 IMU 发布一次状态
	for (uint8_t i = 0; i < IMU_INSTANCES_NUMBER; i++) {
		if (_sensor_device_id[i] != 0) {
			heater_status_s status{};
			status.device_id               = _sensor_device_id[i];
			status.heater_on               = _heater_on[i];
			status.temperature_sensor      = _temperature_last[i];

			const uint8_t param_idx = _current_imu_param_index[i];

			status.temperature_target      = _param_cached_temp[param_idx]; // 使用缓存目标温度
			status.temperature_target_met  = _temperature_target_met[i];
			status.controller_period_usec  = _controller_period_usec;
			status.controller_time_on_usec = _controller_time_on_usec[i];
			status.proportional_value      = _proportional_value[i];
			status.integrator_value        = _integrator_value[i];
			status.feed_forward_value      = _param_cached_temp_f[param_idx]; // 使用缓存前馈值

			status.mode = heater_status_s::MODE_GPIO;
			status.timestamp = hrt_absolute_time();

			_heater_status_pub[i].publish(status);
		}
    }
}

int Heater::start()
{
	update_params(true);
	ScheduleNow();
	return PX4_OK;
}

int Heater::task_spawn(int argc, char *argv[])
{
	Heater *heater = new Heater();

	if (!heater) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(heater);
	_task_id = task_id_is_work_queue;

	heater->start();
	return 0;
}

void Heater::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();

		// 更新缓存的参数值
		for(uint8_t i = 0; i < IMU_INSTANCES_NUMBER; i++) {
			if (_param_imu_temp_f[i] != PARAM_INVALID) {
				param_get(_param_imu_temp_f[i], &_param_cached_temp_f[i]);
			}
			if (_param_imu_temp_i[i] != PARAM_INVALID) {
				param_get(_param_imu_temp_i[i], &_param_cached_temp_i[i]);
			}
			if (_param_imu_temp_p[i] != PARAM_INVALID) {
				param_get(_param_imu_temp_p[i], &_param_cached_temp_p[i]);
			}
			if (_param_imu_temp[i] != PARAM_INVALID) {
				param_get(_param_imu_temp[i], &_param_cached_temp[i]);
			}
			if (_param_sens_temp_id[i] != PARAM_INVALID) {
				param_get(_param_sens_temp_id[i], &_param_cached_sens_id[i]);
			}
		}
	}
}

int Heater::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to regulate IMU temperature at a setpoint.

This task can be started at boot from the startup scripts by setting SENS_EN_THERMAL or via CLI.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("heater_multi", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void Heater::heater_set(uint8_t idx, bool enable)
{
    // 根据索引选择对应的 GPIO
    if (idx == 0) {
        px4_arch_gpiowrite(GPIO_HEATER1, enable);
    } else if (idx == 1) {
        px4_arch_gpiowrite(GPIO_HEATER2, enable);
    }
}


extern "C" __EXPORT int heater_multi_main(int argc, char *argv[])
{
	return Heater::main(argc, argv);
}
