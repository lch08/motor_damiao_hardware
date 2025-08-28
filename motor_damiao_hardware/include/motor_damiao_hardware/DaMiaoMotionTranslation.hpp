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

#include <vector>
#include <memory>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

namespace DaMiaoMotion{

enum class ErrorCode : uint8_t {
    NOT_ENABLE   = 0x0, // 正常
    ENABLE       = 0x1,
    Overvoltage  = 0x8, // 异常
    Undervoltage = 0x9,
    Overcurrent  = 0xA,
    MOSOvertemp  = 0xB,
    CoilOvertemp = 0xC,
    CommLoss     = 0xD,
    Overload     = 0xE
};

struct MITControlParams {
    uint32_t id;
    double   position;
    double   velocity;
    double   kp;
    double   kd;
    double   torque;
};

struct FeedbackData {
    uint32_t  id;
    ErrorCode error_code;
    double    position;
    double    velocity;
    double    torque;
    uint8_t   mos_temperature;
    uint8_t   coil_temperature;
};

struct Config {
    double position_min{-12.5};
    double position_max{12.5};
    double velocity_min{-30.0};
    double velocity_max{30.0};
    double kp_min{0.0};
    double kp_max{500.0};
    double kd_min{0.0};
    double kd_max{5.0};
    double torque_min{-10.0};
    double torque_max{10.0};
    bool reverse{false};  // 是否反向
};

class MotionTranslation {
public:
    /**
     * @brief 构造函数
     */
    MotionTranslation(){}

    inline std::vector<uint8_t> generateEnablePacket() const {
        std::vector<uint8_t> buffer{0xff, 0xff, 0xff, 0xff,
                                    0xff, 0xff, 0xff, 0xfc};
        return buffer;
    }

    inline std::vector<uint8_t> generateUnenablePacket() const {
        std::vector<uint8_t> buffer{0xff, 0xff, 0xff, 0xff,
                                    0xff, 0xff, 0xff, 0xfd};
        return buffer;
    }

    //设定零点
    inline std::vector<uint8_t> generateSetZeroPacket() const {
        std::vector<uint8_t> buffer{0xff, 0xff, 0xff, 0xff,
                                    0xff, 0xff, 0xff, 0xfe};
        return buffer;
    }

    //清除错误
    inline std::vector<uint8_t> generateClearErrorPacket() const {
        std::vector<uint8_t> buffer{0xff, 0xff, 0xff, 0xff,
                                    0xff, 0xff, 0xff, 0xfb};
        return buffer;
    }

    /**
     * @brief 生成控制数据包
     * @param config 配置参数
     * @param params 控制参数
     * @param out 输出数据包
     * @return 是否成功
     */
    bool generateMITControlPacket(const Config& config, const MITControlParams& params, std::vector<uint8_t>& out);

    /**
     * @brief 解析反馈数据包
     * @param in 输入数据包
     * @param data 反馈数据
     * @return 是否成功
     */
    bool parseFeedbackPacket(const Config& config, const std::vector<uint8_t>& in, FeedbackData& data);

private:
    template<typename T>
    static void packBits(std::vector<uint8_t>& buffer, T value, int& bitOffset, int bits);

    template<typename T>
    inline T clamp(T value, T min_val, T max_val) const {
        return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
    }

    inline uint32_t double_to_uint(double value, double min_val, double max_val, int bits) const {
        value = clamp(value, min_val, max_val);
        double scale = (static_cast<double>(1 << bits) - 1) / (max_val - min_val);
        return static_cast<uint32_t>((value - min_val) * scale);
    }

    inline double uint_to_double(uint32_t value, double min_val, double max_val, int bits) const {
        double scale = (max_val - min_val) / (static_cast<double>(1 << bits) - 1);
        return min_val + value * scale;
    }
};

template<typename T>
void MotionTranslation::packBits(std::vector<uint8_t>& buffer, T value, int& bitOffset, int bits) {
    while (bits > 0) {
        int byteIndex = bitOffset / 8;
        int bitIndex = bitOffset % 8;
        int availableBits = 8 - bitIndex;
        int bitsToWrite = std::min(availableBits, bits);

        uint8_t mask = (1 << bitsToWrite) - 1;
        uint8_t shiftedValue = (value >> (bits - bitsToWrite)) & mask;

        buffer[byteIndex] |= (shiftedValue << (8 - bitIndex - bitsToWrite));

        bitOffset += bitsToWrite;
        bits      -= bitsToWrite;
    }
}



} // namespace DaMiaoMotion