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

#include "motor_damiao_hardware/DaMiaoSocketCanDriver.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cstdlib>

namespace DaMiaoMotion {

DaMiaoSocketCanDriver::DaMiaoSocketCanDriver(const std::string& interface_name, rclcpp::Logger logger)
    : interface_name_(interface_name)
    , socket_fd_(-1)
    , logger_(logger)
    , running_(false)
    , translator_(std::make_unique<MotionTranslation>())
{
    RCLCPP_INFO(logger_, "DaMiaoSocketCanDriver created for interface: %s", interface_name_.c_str());
}

DaMiaoSocketCanDriver::~DaMiaoSocketCanDriver() {
    shutdown();
    RCLCPP_INFO(logger_, "DaMiaoSocketCanDriver destroyed");
}

bool DaMiaoSocketCanDriver::initialize() {
    if (running_) {
        RCLCPP_WARN(logger_, "Driver already initialized");
        return true;
    }

    // 创建CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to create CAN socket: %s", strerror(errno));
        return false;
    }

    // 设置socket超时
    struct timeval timeout;
    timeout.tv_sec = SOCKET_TIMEOUT_MS / 1000;
    timeout.tv_usec = (SOCKET_TIMEOUT_MS % 1000) * 1000;
    
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        RCLCPP_WARN(logger_, "Failed to set socket receive timeout: %s", strerror(errno));
    }

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0) {
        RCLCPP_WARN(logger_, "Failed to set socket send timeout: %s", strerror(errno));
    }

    // 绑定到CAN接口
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(logger_, "Failed to get interface index for %s: %s", 
                    interface_name_.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(logger_, "Failed to bind CAN socket to %s: %s", 
                    interface_name_.c_str(), strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 启动接收线程
    running_ = true;
    receive_thread_ = std::make_unique<std::thread>(&DaMiaoSocketCanDriver::receiveThreadFunction, this);

    RCLCPP_INFO(logger_, "CAN driver initialized successfully on interface: %s", interface_name_.c_str());
    return true;
}

void DaMiaoSocketCanDriver::shutdown() {
    if (!running_) {
        return;
    }

    RCLCPP_INFO(logger_, "Shutting down CAN driver...");
    
    // 停止接收线程
    running_ = false;
    if (receive_thread_ && receive_thread_->joinable()) {
        receive_thread_->join();
    }

    // 关闭socket
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    // 清空所有队列
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_queues_.clear();
}

bool DaMiaoSocketCanDriver::setCanBitrate(const std::string& can_interface, uint32_t bitrate) {
    // 构建ip link命令字符串
    std::ostringstream command_stream;
    command_stream << "ip link set " << can_interface << " type can bitrate " << bitrate;
    std::string command = command_stream.str();
    
    RCLCPP_INFO(logger_, "Setting CAN bitrate: %s", command.c_str());
    
    // 执行系统命令
    std::system("ip link set can0 down");
    int result = std::system(command.c_str());
    std::system("ip link set can0 up");
    
    if (result == 0) {
        RCLCPP_INFO(logger_, "Successfully set CAN bitrate for %s to %u", 
                    can_interface.c_str(), bitrate);
        return true;
    } else {
        RCLCPP_ERROR(logger_, "Failed to set CAN bitrate for %s to %u, command returned: %d", 
                     can_interface.c_str(), bitrate, result);
        return false;
    }
}

void DaMiaoSocketCanDriver::setMotorConfig(uint32_t motor_id, uint32_t feedback_id, const Config& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    motor_configs_[motor_id] = config;
    motor_feedback_ids_[feedback_id] = motor_id;
    RCLCPP_INFO(logger_, "Motor 0x%X config updated", motor_id);
}

Config DaMiaoSocketCanDriver::getMotorConfig(uint32_t motor_id) const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    auto it = motor_configs_.find(motor_id);
    if (it != motor_configs_.end()) {
        return it->second;
    }
    
    RCLCPP_WARN(logger_, "Motor 0x%X config not found, using default", motor_id);
    return Config{}; // 返回默认配置
}

bool DaMiaoSocketCanDriver::enableMotor(uint32_t motor_id) {
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "CAN driver not connected");
        return false;
    }

    auto enable_packet = translator_->generateEnablePacket();
    
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = std::min(static_cast<size_t>(8), enable_packet.size());
    
    for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = enable_packet[i];
    }

    bool success = sendCANFrame(frame);
    if (success) {
        RCLCPP_INFO(logger_, "Motor 0x%X enabled", motor_id);
    } else {
        RCLCPP_ERROR(logger_, "Failed to enable motor 0x%X", motor_id);
    }
    
    return success;
}

bool DaMiaoSocketCanDriver::disableMotor(uint32_t motor_id) {
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "CAN driver not connected");
        return false;
    }

    auto disable_packet = translator_->generateUnenablePacket();
    
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = std::min(static_cast<size_t>(8), disable_packet.size());
    
    for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = disable_packet[i];
    }

    bool success = sendCANFrame(frame);
    if (success) {
        RCLCPP_INFO(logger_, "Motor 0x%X disabled", motor_id);
    } else {
        RCLCPP_ERROR(logger_, "Failed to disable motor 0x%X", motor_id);
    }
    
    return success;
}

bool DaMiaoSocketCanDriver::sendMITControl(uint32_t motor_id, const MITControlParams& params) {
    if (!isConnected()) {
        RCLCPP_ERROR(logger_, "CAN driver not connected");
        return false;
    }

    // 获取电机配置
    Config config = getMotorConfig(motor_id);
    
    // 生成控制数据包
    std::vector<uint8_t> control_packet;
    if (!translator_->generateMITControlPacket(config, params, control_packet)) {
        RCLCPP_ERROR(logger_, "Failed to generate MIT control packet for motor 0x%X", motor_id);
        return false;
    }

    // 发送CAN帧
    struct can_frame frame;
    frame.can_id = motor_id;
    frame.can_dlc = std::min(static_cast<size_t>(8), control_packet.size());
    
    for (size_t i = 0; i < frame.can_dlc; ++i) {
        frame.data[i] = control_packet[i];
    }

    bool success = sendCANFrame(frame);
    if (success) {
        // RCLCPP_INFO(logger_, "MIT control sent to motor 0x%X (pos: %.3f, vel: %.3f, kp: %.3f, kd: %.3f, torque: %.3f)", 
        //             motor_id, params.position, params.velocity, params.kp, params.kd, params.torque);
    } else {
        RCLCPP_ERROR(logger_, "Failed to send MIT control to motor 0x%X", motor_id);
    }
    
    return success;
}

bool DaMiaoSocketCanDriver::getFeedback(uint32_t motor_id, FeedbackData& data) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    
    auto it = feedback_queues_.find(motor_id);
    if (it == feedback_queues_.end() || it->second.empty()) {
        return false;
    }
    
    data = it->second.front();
    it->second.pop();
    
    return true;
}

void DaMiaoSocketCanDriver::clearFeedbackQueue(uint32_t motor_id) {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    
    auto it = feedback_queues_.find(motor_id);
    if (it != feedback_queues_.end()) {
        std::queue<FeedbackData> empty;
        it->second.swap(empty);
        RCLCPP_INFO(logger_, "Feedback queue cleared for motor 0x%X", motor_id);
    }
}

void DaMiaoSocketCanDriver::receiveThreadFunction() {
    RCLCPP_INFO(logger_, "CAN receive thread started");
    
    while (running_) {
        struct can_frame frame;
        if (receiveCANFrame(frame)) {
            // 解析反馈数据
            uint32_t feedback_id = frame.can_id;

            uint32_t motor_id;;
            {
                // 查找反馈ID对应的电机CAN ID
                std::lock_guard<std::mutex> config_lock(config_mutex_);
                auto feedback_it = motor_feedback_ids_.find(feedback_id);
                if (feedback_it == motor_feedback_ids_.end()) {
                    RCLCPP_WARN(logger_, "Unknown feedback ID 0x%X", feedback_id);
                    continue;
                }
                motor_id = feedback_it->second;
            }
            Config config = getMotorConfig(motor_id);
            
            std::vector<uint8_t> packet_data(frame.data, frame.data + frame.can_dlc);
            FeedbackData feedback;
            
            if (translator_->parseFeedbackPacket(config, packet_data, feedback)) {
                feedback.id = motor_id;
                
                // 将反馈数据加入队列
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                auto& queue = feedback_queues_[motor_id];
                
                // 限制队列大小
                if (queue.size() >= MAX_QUEUE_SIZE) {
                    queue.pop();
                    RCLCPP_WARN(logger_, "Feedback queue full for motor 0x%X, dropping oldest data", motor_id);
                }
                
                queue.push(feedback);
                
                // RCLCPP_INFO(logger_, "Feedback received for motor 0x%X (error: %u, pos: %.3f, vel: %.3f, torque: %.3f)", 
                //             motor_id, static_cast<uint8_t>(feedback.errorCode), 
                //             feedback.position, feedback.velocity, feedback.torque);
            } else {
                RCLCPP_WARN(logger_, "Failed to parse feedback packet from motor 0x%X", motor_id);
            }
        }
    }
    
    RCLCPP_INFO(logger_, "CAN receive thread stopped");
}

bool DaMiaoSocketCanDriver::sendCANFrame(const struct can_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }

    ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(frame));
    if (bytes_sent == sizeof(frame)) {
        return true;
    } else {
        RCLCPP_ERROR(logger_, "Failed to send CAN frame: %s", strerror(errno));
        return false;
    }
}

bool DaMiaoSocketCanDriver::receiveCANFrame(struct can_frame& frame) {
    if (socket_fd_ < 0) {
        return false;
    }

    ssize_t bytes_received = read(socket_fd_, &frame, sizeof(frame));
    if (bytes_received == sizeof(frame)) {
        return true;
    } else if (bytes_received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // 超时，这是正常的
        return false;
    } else {
        RCLCPP_ERROR(logger_, "Failed to receive CAN frame: %s", strerror(errno));
        return false;
    }
}

} // namespace DaMiaoMotion
