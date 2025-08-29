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

#include <stdint.h>

enum class MotorErrorCode: int {
	ENABLE          = 0x00,  // 使能
	DISABLE         = 0x01,  // 失能
	DISCONNECT      = 0x02,  // 断开连接
	OVER_LOAD       = 0x03,  // 过载
	OVER_CURRENT    = 0x04,  // 过流
	OVER_VOLTAGE    = 0x05,  // 过电压
	LOW_VOLTAGE     = 0x06,  // 低电压
	MOS_OVER_TEMP   = 0x07,  // mos超温
	MOTOR_OVER_TEMP = 0x08   // 电机超温
};



