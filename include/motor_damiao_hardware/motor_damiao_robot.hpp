// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "motor_damiao_hardware/visibility_control.h"
#include "motor_damiao_hardware/DaMiaoSocketCanDriver.hpp"
#include "motor_damiao_hardware/DaMiaoMotionTranslation.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace motor_damiao_hardware
{

struct MotorConfig{
    uint32_t can_id;
    uint32_t feedback_id;
    double kp;
    double kd;
    std::string can_name;
    DaMiaoMotion::Config damiao_config;
};

class MotorDamiaoRobot : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MotorDamiaoRobot);

  ~MotorDamiaoRobot() override;

  MOTOR_DAMIAO_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  MOTOR_DAMIAO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MOTOR_DAMIAO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MOTOR_DAMIAO_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  MOTOR_DAMIAO_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // 命令接口
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  
  // 状态接口
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  
  std::vector<MotorConfig> motor_configs_;

  // 存储不同CAN接口的驱动程序实例
  std::map<std::string, std::shared_ptr<DaMiaoMotion::DaMiaoSocketCanDriver>> can_drivers_;
  
  // 位置累加相关变量
  std::vector<double> previous_raw_position_;  // 上一次的原始位置值
  std::vector<double> accumulated_position_;   // 累加的位置值
  std::vector<bool> position_initialized_;     // 位置是否已初始化
  std::vector<bool> enable_position_expansion_; // 是否启用位置扩展
};

}  // namespace motor_damiao_hardware
