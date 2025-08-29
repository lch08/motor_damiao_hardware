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

#include "motor_damiao_hardware/motor_damiao_robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>
#include <set>
#include <cmath>

namespace motor_damiao_hardware {
// ------------------------------------------------------------------------------------------
CallbackReturn MotorDamiaoRobot::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 默认电机反馈超时时间1秒
    feedback_timeout_ = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(1.0));


    if (info.hardware_parameters.find("feedback_timeout") != info.hardware_parameters.end()) {
        double timeout_sec = std::stod(info.hardware_parameters.at("feedback_timeout"));
        feedback_timeout_ = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(timeout_sec));
    }

    // Allocate memory
    hw_states_position_.resize(info_.joints.size(), 0.0);
    hw_states_velocity_.resize(info_.joints.size(), 0.0);
    hw_states_effort_.resize(info_.joints.size(), 0.0);
    hw_states_mos_temperature_.resize(info_.joints.size(), 0.0);
    hw_states_motor_temperature_.resize(info_.joints.size(), 0.0);
    hw_states_error_code_.resize(info_.joints.size(), 0.0);
    hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    motor_configs_.resize(info_.joints.size());

    // 初始化位置累加相关变量
    previous_raw_position_.resize(info_.joints.size(), 0.0);
    accumulated_position_.resize(info_.joints.size(), 0.0);
    position_initialized_.resize(info_.joints.size(), false);
    enable_position_expansion_.resize(info_.joints.size(), false);

    // 检查硬件参数是否存在
    std::map<std::string, uint32_t> can_bitrates;
    if (info.hardware_parameters.find("can0_baud_rate") != info.hardware_parameters.end()) {
        can_bitrates["can0"] = std::stoi(info.hardware_parameters.at("can0_baud_rate"));
    }
    if (info.hardware_parameters.find("can1_baud_rate") != info.hardware_parameters.end()) {
        can_bitrates["can1"] = std::stoi(info.hardware_parameters.at("can1_baud_rate"));
    }
    if (info.hardware_parameters.find("can2_baud_rate") != info.hardware_parameters.end()) {
        can_bitrates["can2"] = std::stoi(info.hardware_parameters.at("can2_baud_rate"));
    }
    if (info.hardware_parameters.find("can3_baud_rate") != info.hardware_parameters.end()) {
        can_bitrates["can3"] = std::stoi(info.hardware_parameters.at("can3_baud_rate"));
    }

    // 验证关节配置
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const hardware_interface::ComponentInfo& joint = info_.joints[i];

        // 只检查必须的3个接口是否存在
        bool has_position = false, has_velocity = false, has_effort = false;
        for (const auto& iface : joint.state_interfaces) {
            if (iface.name == hardware_interface::HW_IF_POSITION) has_position = true;
            if (iface.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
            if (iface.name == hardware_interface::HW_IF_EFFORT) has_effort = true;
        }
        if (!(has_position && has_velocity && has_effort)) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing required state interfaces (position, velocity, effort).",
                joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        // 检查必需的参数
        if (joint.parameters.find("can_name") == joint.parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing parameter: can_name", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.parameters.find("can_id") == joint.parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing parameter: can_id", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.parameters.find("feedback_id") == joint.parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing parameter: feedback_id", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.parameters.find("kp") == joint.parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing parameter: kp", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.parameters.find("kd") == joint.parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("MotorDamiaoRobot"),
                "Joint '%s' missing parameter: kd", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        // 配置电机参数
        motor_configs_[i].can_name = joint.parameters.at("can_name");
        motor_configs_[i].can_id = std::stoi(joint.parameters.at("can_id"), nullptr, 0);
        motor_configs_[i].feedback_id = std::stoi(joint.parameters.at("feedback_id"), nullptr, 0);
        motor_configs_[i].kp = std::stod(joint.parameters.at("kp"));
        motor_configs_[i].kd = std::stod(joint.parameters.at("kd"));

        motor_configs_[i].err_code = DaMiaoMotion::ErrorCode::NOT_ENABLE;
        motor_configs_[i].damiao_config = DaMiaoMotion::Config{};

        // 从joint参数读取电机特定的限制值
        if (joint.parameters.find("position_max") != joint.parameters.end()) {
            double position_max = std::stod(joint.parameters.at("position_max"));
            motor_configs_[i].damiao_config.position_max = position_max;
            motor_configs_[i].damiao_config.position_min = -position_max;  // 假设对称
        }

        if (joint.parameters.find("velocity_max") != joint.parameters.end()) {
            double velocity_max = std::stod(joint.parameters.at("velocity_max"));
            motor_configs_[i].damiao_config.velocity_max = velocity_max;
            motor_configs_[i].damiao_config.velocity_min = -velocity_max;  // 假设对称
        }

        if (joint.parameters.find("torque_max") != joint.parameters.end()) {
            double torque_max = std::stod(joint.parameters.at("torque_max"));
            motor_configs_[i].damiao_config.torque_max = torque_max;
            motor_configs_[i].damiao_config.torque_min = -torque_max;  // 假设对称
        }

        // 检查是否配置了反向参数
        if (joint.parameters.find("reverse") != joint.parameters.end()) {
            std::string reverse_str = joint.parameters.at("reverse");
            motor_configs_[i].damiao_config.reverse = (reverse_str == "true" || reverse_str == "1");
        } else {
            motor_configs_[i].damiao_config.reverse = false;  // 默认不反向
        }

        // 检查是否启用位置扩展
        if (joint.parameters.find("enable_position_expansion") != joint.parameters.end()) {
            std::string expansion_str = joint.parameters.at("enable_position_expansion");
            enable_position_expansion_[i] = (expansion_str == "true" || expansion_str == "1");
        } else {
            enable_position_expansion_[i] = false;  // 默认不启用
        }

        RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"),
                    "Motor %s config: pos[%.2f,%.2f] vel[%.2f,%.2f] torque[%.2f,%.2f] reverse=%s expansion=%s",
                    joint.name.c_str(),
                    motor_configs_[i].damiao_config.position_min,
                    motor_configs_[i].damiao_config.position_max,
                    motor_configs_[i].damiao_config.velocity_min,
                    motor_configs_[i].damiao_config.velocity_max,
                    motor_configs_[i].damiao_config.torque_min,
                    motor_configs_[i].damiao_config.torque_max,
                    motor_configs_[i].damiao_config.reverse ? "true" : "false",
                    enable_position_expansion_[i] ? "true" : "false");
    }

    // 创建并初始化CAN驱动程序
    std::set<std::string> can_interfaces;
    for (const auto& config : motor_configs_) {
        can_interfaces.insert(config.can_name);
    }

    for (const std::string& can_name : can_interfaces) {
        // 设置CAN波特率
        if (can_bitrates.find(can_name) != can_bitrates.end()) {
            uint32_t bitrate = can_bitrates[can_name];
            RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"),
                        "Setting bitrate for %s to %u", can_name.c_str(), bitrate);
            DaMiaoMotion::DaMiaoSocketCanDriver driver_for_bitrate(can_name);
            driver_for_bitrate.setCanBitrate(can_name, bitrate);
        }

        // 创建CAN驱动程序实例
        auto driver = std::make_shared<DaMiaoMotion::DaMiaoSocketCanDriver>(
            can_name, rclcpp::get_logger("MotorDamiaoRobot"));

        if (!driver->initialize()) {
            RCLCPP_FATAL(rclcpp::get_logger("MotorDamiaoRobot"),
                         "Failed to initialize CAN driver for %s", can_name.c_str());
            return CallbackReturn::ERROR;
        }

        can_drivers_[can_name] = driver;
        RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"),
                    "CAN driver initialized for %s", can_name.c_str());
    }

    

    RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "MotorDamiaoRobot initialized successfully");
    return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn MotorDamiaoRobot::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // 配置电机
    for (size_t i = 0; i < motor_configs_.size(); ++i) {
        const auto& config = motor_configs_[i];
        auto driver = can_drivers_[config.can_name];

        driver->setMotorConfig(config.can_id, config.feedback_id, config.damiao_config);
    }

    RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "MotorDamiaoRobot configured successfully");
    return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn MotorDamiaoRobot::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    enableAllMotor();
    is_active_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "MotorDamiaoRobot activated successfully");
    return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
CallbackReturn MotorDamiaoRobot::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    disableAllMotor();
    is_active_ = false;
    RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "MotorDamiaoRobot deactivated successfully");
    return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------------------------------------
MotorDamiaoRobot::~MotorDamiaoRobot() {
    RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "~MotorDamiaoRobot");

    disableAllMotor();

    // 关闭所有CAN驱动程序
    for (auto& driver_pair : can_drivers_) {
        driver_pair.second->shutdown();
    }
    can_drivers_.clear();
}
std::vector<hardware_interface::StateInterface>
MotorDamiaoRobot::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "mos_temperature", &hw_states_mos_temperature_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "motor_temperature", &hw_states_motor_temperature_[i]));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "error_code", &hw_states_error_code_[i]));
    }

    return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
MotorDamiaoRobot::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
    }

    return command_interfaces;
}

// ------------------------------------------------------------------------------------------
hardware_interface::return_type MotorDamiaoRobot::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    // 从各个CAN驱动程序读取反馈数据
    for (size_t i = 0; i < motor_configs_.size(); ++i) {
        MotorErrorCode new_err_code = static_cast<MotorErrorCode>(hw_states_error_code_[i]);

        const auto& config = motor_configs_[i];
        auto driver_it = can_drivers_.find(config.can_name);

        if (driver_it != can_drivers_.end()) {
            DaMiaoMotion::FeedbackData feedback;
            if (driver_it->second->getFeedback(config.can_id, feedback)) {
                // 更新最后反馈时间
                motor_configs_[i].last_feedback_time = rclcpp::Clock().now();
                motor_configs_[i].err_code = feedback.error_code;

                new_err_code = convertErrorCode(feedback.error_code);

                // 处理位置数据
                if (enable_position_expansion_[i]) {
                    // 启用位置扩展
                    if (!position_initialized_[i]) {
                        // 第一次读取，初始化
                        previous_raw_position_[i] = feedback.position;
                        accumulated_position_[i] = feedback.position;
                        position_initialized_[i] = true;
                    } else {
                        // 检测位置跳跃
                        double raw_position = feedback.position;
                        double position_diff = raw_position - previous_raw_position_[i];

                        // 获取位置范围用于跳跃检测
                        double position_range = config.damiao_config.position_max - config.damiao_config.position_min;
                        double half_range = position_range / 2.0;

                        // 检测正向跳跃（从最大值跳到最小值附近）
                        if (position_diff < -half_range) {
                            // 正向跨越边界，位置增加一个完整范围
                            accumulated_position_[i] += position_range + position_diff;
                            //   RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"),
                            //               "Motor %s: Forward wrap detected, diff=%.3f, accumulated=%.3f",
                            //               info_.joints[i].name.c_str(), position_diff, accumulated_position_[i]);
                        }
                        // 检测反向跳跃（从最小值跳到最大值附近）
                        else if (position_diff > half_range) {
                            // 反向跨越边界，位置减少一个完整范围
                            accumulated_position_[i] += position_diff - position_range;
                            //   RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"),
                            //               "Motor %s: Backward wrap detected, diff=%.3f, accumulated=%.3f",
                            //               info_.joints[i].name.c_str(), position_diff, accumulated_position_[i]);
                        }
                        // 正常情况，直接累加差值
                        else {
                            accumulated_position_[i] += position_diff;
                        }

                        previous_raw_position_[i] = raw_position;
                    }

                    hw_states_position_[i] = accumulated_position_[i];
                } else {
                    // 不启用位置扩展，直接使用原始位置
                    hw_states_position_[i] = feedback.position;
                }

                hw_states_velocity_[i] = feedback.velocity;
                hw_states_effort_[i] = feedback.torque;

                hw_states_mos_temperature_[i] = feedback.mos_temperature;
                hw_states_motor_temperature_[i] = feedback.coil_temperature;
            } else {
                // 检查是否超时
                rclcpp::Duration time_since_last_feedback = rclcpp::Clock().now() - motor_configs_[i].last_feedback_time;
                if (time_since_last_feedback > *feedback_timeout_) {
                    // 超时，设置为断开连接状态
                    if (new_err_code != MotorErrorCode::DISCONNECT &&
                        (new_err_code == MotorErrorCode::ENABLE ||
                         new_err_code == MotorErrorCode::DISABLE)) {
                        new_err_code = MotorErrorCode::DISCONNECT;
                    }
                }
            }
            hw_states_error_code_[i] = static_cast<double>(new_err_code);
        }
    }

    return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type MotorDamiaoRobot::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
    for (size_t i = 0; i < motor_configs_.size(); ++i) {
        const auto& config = motor_configs_[i];
        auto driver_it = can_drivers_.find(config.can_name);
        if (driver_it == can_drivers_.end()) {
            continue;
        }

        // 仅在active时更改电机状态
        if (is_active_) {
            // 检查电机是否在失能状态，如果是则重新使能
            if (motor_configs_[i].err_code == DaMiaoMotion::ErrorCode::NOT_ENABLE &&
                motor_configs_[i].enable_time < rclcpp::Clock().now() - rclcpp::Duration::from_seconds(0.1)) {
                RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "Re-enabling motor 0x%X on %s",
                            config.can_id, config.can_name.c_str());

                if (!driver_it->second->enableMotor(config.can_id)) {
                    RCLCPP_WARN(rclcpp::get_logger("MotorDamiaoRobot"),
                                "Failed to re-enable motor 0x%X on %s", config.can_id, config.can_name.c_str());
                } else {
                    motor_configs_[i].enable_time = rclcpp::Clock().now();
                }
            }
            // 如果电机回报通信超时,则自动清除错误
            if (motor_configs_[i].err_code == DaMiaoMotion::ErrorCode::CommLoss) {
                RCLCPP_INFO(rclcpp::get_logger("MotorDamiaoRobot"), "Clear command loss motor 0x%X error on %s",
                            config.can_id, config.can_name.c_str());
                if (!driver_it->second->clearMotorError(config.can_id)) {
                    RCLCPP_WARN(rclcpp::get_logger("MotorDamiaoRobot"),
                                "Failed to clear motor 0x%X error on %s", config.can_id, config.can_name.c_str());
                }
            }
        }

        // 检查哪些命令接口有有效值
        bool has_position_cmd = !std::isnan(hw_commands_position_[i]);
        bool has_velocity_cmd = !std::isnan(hw_commands_velocity_[i]);
        bool has_effort_cmd = !std::isnan(hw_commands_effort_[i]);

        // 准备MIT控制参数
        DaMiaoMotion::MITControlParams mit_params;
        mit_params.id = config.can_id;

        // 根据有效的命令类型设置控制参数
        if (has_position_cmd) {
            // 获取位置限制
            double min_position = config.damiao_config.position_min;
            double max_position = config.damiao_config.position_max;

            // 尝试从关节配置获取限制
            for (const auto& cmd_interface : info_.joints[i].command_interfaces) {
                if (cmd_interface.name == hardware_interface::HW_IF_POSITION) {
                    if (!cmd_interface.min.empty()) {
                        min_position = std::stod(cmd_interface.min);
                    }
                    if (!cmd_interface.max.empty()) {
                        max_position = std::stod(cmd_interface.max);
                    }
                    break;
                }
            }

            double target_position = hw_commands_position_[i];

            // 如果启用了位置扩展，需要将扩展位置转换为电机的有限范围
            if (enable_position_expansion_[i]) {
                double position_range = max_position - min_position;
                // 将目标位置映射到电机的有效范围内
                target_position = std::fmod(target_position - min_position, position_range);
                if (target_position < 0) {
                    target_position += position_range;
                }
                target_position += min_position;
            } else {
                // 限制命令位置到有效范围
                if (target_position > max_position) {
                    target_position = max_position;
                }
                if (target_position < min_position) {
                    target_position = min_position;
                }
            }

            mit_params.position = target_position;
            mit_params.kp = config.kp;
        } else {
            mit_params.position = 0.0;
            mit_params.kp = 0.0;  // 禁用位置控制
        }

        if (has_velocity_cmd) {
            // 获取速度限制
            double min_velocity = -30.0;  // 默认值
            double max_velocity = 30.0;

            // 尝试从关节配置获取限制
            for (const auto& cmd_interface : info_.joints[i].command_interfaces) {
                if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY) {
                    if (!cmd_interface.min.empty()) {
                        min_velocity = std::stod(cmd_interface.min);
                    }
                    if (!cmd_interface.max.empty()) {
                        max_velocity = std::stod(cmd_interface.max);
                    }
                    break;
                }
            }

            // 限制命令速度
            double target_velocity = hw_commands_velocity_[i];
            if (target_velocity > max_velocity) {
                target_velocity = max_velocity;
            }
            if (target_velocity < min_velocity) {
                target_velocity = min_velocity;
            }

            mit_params.velocity = target_velocity;
            mit_params.kd = config.kd;
        } else {
            mit_params.velocity = 0.0;
            mit_params.kd = 0.0;  // 禁用速度控制
        }

        if (has_effort_cmd) {
            // 获取力矩限制
            double min_torque = -10.0;  // 默认值
            double max_torque = 10.0;

            // 尝试从关节配置获取限制
            for (const auto& cmd_interface : info_.joints[i].command_interfaces) {
                if (cmd_interface.name == hardware_interface::HW_IF_EFFORT) {
                    if (!cmd_interface.min.empty()) {
                        min_torque = std::stod(cmd_interface.min);
                    }
                    if (!cmd_interface.max.empty()) {
                        max_torque = std::stod(cmd_interface.max);
                    }
                    break;
                }
            }

            // 限制命令力矩
            double target_torque = hw_commands_effort_[i];
            if (target_torque > max_torque) {
                target_torque = max_torque;
            }
            if (target_torque < min_torque) {
                target_torque = min_torque;
            }

            mit_params.torque = target_torque;
        } else {
            mit_params.torque = 0.0;
        }

        // 发送控制命令
        if (!driver_it->second->sendMITControl(config.can_id, mit_params)) {
            //   RCLCPP_WARN(rclcpp::get_logger("MotorDamiaoRobot"),
            //               "Failed to send control command to motor 0x%X on %s",
            //               config.can_id, config.can_name.c_str());
        }
    }

    return hardware_interface::return_type::OK;
}

// ------------------------------------------------------------------------------------------
MotorErrorCode MotorDamiaoRobot::convertErrorCode(const DaMiaoMotion::ErrorCode& damiao_error) const {
    switch (damiao_error) {
        case DaMiaoMotion::ErrorCode::ENABLE:
            return MotorErrorCode::ENABLE;

        case DaMiaoMotion::ErrorCode::NOT_ENABLE:
            return MotorErrorCode::DISABLE;

        case DaMiaoMotion::ErrorCode::Overvoltage:
            return MotorErrorCode::OVER_VOLTAGE;

        case DaMiaoMotion::ErrorCode::Undervoltage:
            return MotorErrorCode::LOW_VOLTAGE;

        case DaMiaoMotion::ErrorCode::Overcurrent:
            return MotorErrorCode::OVER_CURRENT;

        case DaMiaoMotion::ErrorCode::MOSOvertemp:
            return MotorErrorCode::MOS_OVER_TEMP;

        case DaMiaoMotion::ErrorCode::CoilOvertemp:
            return MotorErrorCode::MOTOR_OVER_TEMP;

        case DaMiaoMotion::ErrorCode::CommLoss:
            return MotorErrorCode::DISCONNECT;

        case DaMiaoMotion::ErrorCode::Overload:
            return MotorErrorCode::OVER_LOAD;

        default:
            // 未知错误代码，返回断开连接状态
            RCLCPP_WARN(rclcpp::get_logger("MotorDamiaoRobot"),
                        "Unknown DaMiao error code: 0x%X, treating as disconnect",
                        static_cast<uint8_t>(damiao_error));
            return MotorErrorCode::DISCONNECT;
    }
}

// ------------------------------------------------------------------------------------------
void MotorDamiaoRobot::enableAllMotor() {
    for (size_t i = 0; i < motor_configs_.size(); ++i) {
        const auto& config = motor_configs_[i];
        auto driver = can_drivers_[config.can_name];

        // 清除错误
        if (!driver->clearMotorError(config.can_id)) {
            RCLCPP_WARN(rclcpp::get_logger("MotorDamiaoRobot"),
                        "Failed to clear motor 0x%X error on %s", config.can_id, config.can_name.c_str());
        }

        // 启用电机
        if (!driver->enableMotor(config.can_id)) {
            RCLCPP_ERROR(rclcpp::get_logger("MotorDamiaoRobot"),
                         "Failed to enable motor 0x%X on %s", config.can_id, config.can_name.c_str());
        } else {
            motor_configs_[i].enable_time        = rclcpp::Clock().now();
            motor_configs_[i].last_feedback_time = rclcpp::Clock().now();

            hw_states_error_code_[i] = static_cast<double>(MotorErrorCode::ENABLE);
        }
    }
}

// ------------------------------------------------------------------------------------------
void MotorDamiaoRobot::disableAllMotor() {
    for (size_t i = 0; i < motor_configs_.size(); ++i) {
        const auto& config = motor_configs_[i];
        auto driver_it = can_drivers_.find(config.can_name);
        if (driver_it != can_drivers_.end()) {
            driver_it->second->disableMotor(config.can_id);
            hw_states_error_code_[i] = static_cast<double>(MotorErrorCode::DISABLE);
        }
    }
}

}  // namespace motor_damiao_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
#include "motor_damiao_hardware/motor_damiao_robot.hpp"

PLUGINLIB_EXPORT_CLASS(
    motor_damiao_hardware::MotorDamiaoRobot, hardware_interface::SystemInterface)
