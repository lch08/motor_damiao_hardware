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

#include "motor_damiao_hardware/DaMiaoMotionTranslation.hpp"

namespace DaMiaoMotion {

bool MotionTranslation::generateMITControlPacket(const Config& config, const MITControlParams& params, std::vector<uint8_t>& out) {
    out.resize(8, 0x00);
    int bitOffset = 0;

    // 根据反向配置处理参数
    double position = config.reverse ? -params.position : params.position;
    double velocity = config.reverse ? -params.velocity : params.velocity;
    double torque = config.reverse ? -params.torque : params.torque;

    // Position (16 bits)
    uint32_t p_des = double_to_uint(position, config.position_min, config.position_max, 16);
    packBits(out, p_des, bitOffset, 16);

    // Velocity (12 bits)
    uint32_t v_des = double_to_uint(velocity, config.velocity_min, config.velocity_max, 12);
    packBits(out, v_des, bitOffset, 12);

    // Kp (12 bits)
    uint32_t kp = double_to_uint(params.kp, config.kp_min, config.kp_max, 12);
    packBits(out, kp, bitOffset, 12);

    // Kd (12 bits)
    uint32_t kd = double_to_uint(params.kd, config.kd_min, config.kd_max, 12);
    packBits(out, kd, bitOffset, 12);

    // Torque (12 bits)
    uint32_t t_ff = double_to_uint(torque, config.torque_min, config.torque_max, 12);
    packBits(out, t_ff, bitOffset, 12);

    return true;
}

bool MotionTranslation::parseFeedbackPacket(const Config& config, const std::vector<uint8_t>& in, FeedbackData& data) {
    if (in.size() != 8) {
        return false; // 输入数据包大小不正确
    }

    // ID
    data.id = in[0] & 0x0F;

    // Error
    data.error_code = static_cast<ErrorCode>((in[0] & 0xF0) >> 4);
     

    // Position (16 bits)
    uint16_t pos = (in[1] << 8) | in[2];
    data.position = uint_to_double(pos, config.position_min, config.position_max, 16);
    if (config.reverse) {
        data.position = -data.position;
    }

    // Velocity (12 bits)
    uint16_t vel = ((in[3] << 8) | (in[4] & 0xF0)) >> 4;
    data.velocity = uint_to_double(vel, config.velocity_min, config.velocity_max, 12);
    if (config.reverse) {
        data.velocity = -data.velocity;
    }

    // Torque (12 bits)
    uint16_t torque = ((in[4] & 0x0F) << 8 | in[5] ) | in[6];
    data.torque = uint_to_double(torque, config.torque_min, config.torque_max, 12);
    if (config.reverse) {
        data.torque = -data.torque;
    }

    // Temperatures
    data.mos_temperature = in[6];
    data.coil_temperature = in[7];

    return true;
}

} // namespace DaMiaoMotion
