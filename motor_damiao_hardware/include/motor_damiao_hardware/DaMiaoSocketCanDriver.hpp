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

#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "DaMiaoMotionTranslation.hpp"

namespace DaMiaoMotion {

class DaMiaoSocketCanDriver {
public:
    /**
     * @brief 构造函数
     * @param interface_name CAN接口名称 (例如: "can0")
     * @param logger ROS2日志记录器
     */
    explicit DaMiaoSocketCanDriver(const std::string& interface_name, 
                                  rclcpp::Logger logger = rclcpp::get_logger("DaMiaoSocketCanDriver"));

    /**
     * @brief 析构函数
     */
    ~DaMiaoSocketCanDriver();

    /**
     * @brief 初始化CAN总线连接
     * @return 是否成功初始化
     */
    bool initialize();

    /**
     * @brief 关闭CAN总线连接
     */
    void shutdown();

    /**
     * @brief 设置CAN总线波特率
     * @param can_interface CAN接口名称 (例如: "can0")
     * @param bitrate 波特率 (例如: 1000000)
     * @return 是否成功设置
     */
    bool setCanBitrate(const std::string& can_interface, uint32_t bitrate);

    /**
     * @brief 设置电机配置
     * @param motor_id 电机ID
     * @param feedback_id 电机反馈ID
     * @param config 电机配置参数
     */
    void setMotorConfig(uint32_t motor_id, uint32_t feedback_id, const Config& config);

    /**
     * @brief 获取电机配置
     * @param motor_id 电机ID
     * @return 电机配置参数，如果不存在则返回默认配置
     */
    Config getMotorConfig(uint32_t motor_id) const;

    /**
     * @brief 启用电机
     * @param motor_id 电机ID
     * @return 是否成功发送
     */
    bool enableMotor(uint32_t motor_id);

    /**
     * @brief 禁用电机
     * @param motor_id 电机ID
     * @return 是否成功发送
     */
    bool disableMotor(uint32_t motor_id);

    /**
     * @brief 清除电机错误
     * @param motor_id 电机ID
     * @return 是否成功发送
     */
    bool clearMotorError(uint32_t motor_id);

    /**
     * @brief 发送MIT控制命令
     * @param motor_id 电机ID
     * @param params 控制参数
     * @return 是否成功发送
     */
    bool sendMITControl(uint32_t motor_id, const MITControlParams& params);

    /**
     * @brief 获取反馈数据
     * @param motor_id 电机ID
     * @param data 输出的反馈数据
     * @return 是否成功获取到数据
     */
    bool getFeedback(uint32_t motor_id, FeedbackData& data);

    /**
     * @brief 清空指定电机的反馈队列
     * @param motor_id 电机ID
     */
    void clearFeedbackQueue(uint32_t motor_id);

    /**
     * @brief 检查是否连接
     * @return 连接状态
     */
    bool isConnected() const { return socket_fd_ >= 0 && running_; }

private:
    /**
     * @brief 接收线程函数
     */
    void receiveThreadFunction();

    /**
     * @brief 发送CAN帧
     * @param frame CAN帧数据
     * @return 是否成功发送
     */
    bool sendCANFrame(const struct can_frame& frame);

    /**
     * @brief 接收CAN帧
     * @param frame 输出的CAN帧数据
     * @return 是否成功接收
     */
    bool receiveCANFrame(struct can_frame& frame);

    // 成员变量
    std::string interface_name_;                             // CAN接口名称
    int socket_fd_;                                          // CAN socket文件描述符
    rclcpp::Logger logger_;                                  // 日志记录器

    // 线程相关
    std::atomic<bool> running_;                              // 运行状态
    std::unique_ptr<std::thread> receive_thread_;            // 接收线程

    // 电机配置
    std::map<uint32_t, Config> motor_configs_;               // 电机ID到配置的映射
    std::map<uint32_t, uint32_t> motor_feedback_ids_;        // 电机反馈ID到电机CAN ID的映射
    mutable std::mutex config_mutex_;                        // 配置互斥锁

    // 反馈数据队列
    std::map<uint32_t, std::queue<FeedbackData>> feedback_queues_;  // 每个电机的反馈队列
    std::mutex feedback_mutex_;                              // 反馈数据互斥锁
    std::condition_variable feedback_cv_;                    // 反馈数据条件变量

    // 消息翻译器
    std::unique_ptr<MotionTranslation> translator_;          // 消息翻译器

    // 常量
    static constexpr size_t MAX_QUEUE_SIZE = 10;             // 最大队列大小
    static constexpr int SOCKET_TIMEOUT_MS = 50;             // Socket超时时间(毫秒)
};

} // namespace DaMiaoMotion
