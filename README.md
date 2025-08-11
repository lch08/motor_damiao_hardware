# Motor Damiao Hardware

一个基于ROS2的达妙电机硬件接口包，为使用达妙电机的机器人提供ros2_control硬件接口。

## 简介

这个包实现了达妙电机的硬件接口，使其能够与ROS2的控制系统无缝集成。支持通过CAN总线进行电机控制和状态反馈，提供位置、速度和力矩控制接口。

## 特性

- 🔧 **完整的硬件接口**: 实现了`hardware_interface::SystemInterface`，提供标准的ROS2控制接口
- 🚗 **CAN总线通信**: 基于SocketCAN的高效通信实现
- 📊 **多种控制模式**: 支持位置、速度和力矩控制
- 🔄 **实时反馈**: 提供电机位置、速度、力矩和温度反馈
- ⚙️ **灵活配置**: 支持多电机配置和参数调节
- 🔄 **位置扩展**: 支持连续旋转的位置累加功能
- 🛡️ **错误处理**: 完善的错误检测和处理机制

## 系统要求

- Ubuntu 20.04/22.04
- ROS2 Humble/Iron/Rolling
- Linux CAN支持 (can-utils)

## 安装

### 1. 安装依赖

```bash
# 安装ROS2控制相关包
./scripts/install_ros2_control.sh

# 或手动安装
sudo apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher \
                        ros-${ROS_DISTRO}-xacro \
                        ros-${ROS_DISTRO}-ros2-control*

# 安装CAN工具
sudo apt-get install can-utils
```

### 2. 编译包

```bash
# 在工作空间根目录
colcon build --packages-select motor_damiao_hardware
source install/setup.bash
```

## 配置

### CAN总线设置

首先需要配置CAN接口：

```bash
# 设置CAN接口 (以can0为例，波特率1Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 验证接口状态
ip link show can0
```

### URDF配置

在您的URDF文件中使用此硬件接口：

```xml
<ros2_control name="motor_damiao" type="system">
    <hardware>
        <plugin>motor_damiao_hardware/MotorDamiaoRobot</plugin>
        <param name="can0_baud_rate">1000000</param>
    </hardware>
    
    <joint name="wheel_joint">
        <!-- 命令接口 -->
        <command_interface name="position">
            <param name="min">-1.57</param>
            <param name="max">1.57</param>
        </command_interface>
        <command_interface name="velocity">
            <param name="min">-1.0</param>
            <param name="max">1.0</param>
        </command_interface>
        <command_interface name="effort">
            <param name="min">-5.0</param>
            <param name="max">5.0</param>
        </command_interface>
        
        <!-- 状态接口 -->
        <state_interface name="position" />
        <state_interface name="velocity" />
        <state_interface name="effort" />

        <!-- 电机参数 -->
        <param name="can_name">can0</param>
        <param name="can_id">0x01</param>
        <param name="feedback_id">0x11</param>
        <param name="position_max">12.5</param>
        <param name="velocity_max">45.0</param>
        <param name="kp">5.0</param>
        <param name="kd">1.0</param>
        <param name="torque_max">10.0</param>
        <param name="reverse">false</param>
        <param name="enable_position_expansion">true</param>
    </joint>
</ros2_control>
```

## 参数说明

### 硬件级参数
- `can0_baud_rate`: CAN总线波特率 (默认: 1000000)

### 关节级参数
- `can_name`: CAN接口名称 (如: "can0")
- `can_id`: 电机CAN ID (十六进制，如: 0x01)
- `feedback_id`: 电机反馈ID (十六进制，如: 0x11)
- `position_max`: 最大位置限制 (弧度)
- `velocity_max`: 最大速度限制 (弧度/秒)
- `kp`: 比例增益
- `kd`: 微分增益
- `torque_max`: 最大力矩限制 (Nm)
- `reverse`: 是否反向 (true/false)
- `enable_position_expansion`: 是否启用位置扩展 (true/false)

## 使用示例

### 1. 基本使用

```bash
# 启动控制器管理器
ros2 run controller_manager ros2_control_node --ros-args --params-file config/controllers.yaml

# 列出可用控制器
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# 启动关节状态控制器
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{start_controllers: ['joint_state_broadcaster']}"
```

### 2. 控制器配置

创建控制器配置文件 `config/controllers.yaml`：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

position_controller:
  ros__parameters:
    joints:
      - left_wheel_joint
      - right_wheel_joint
```

## 架构设计

### 核心组件

1. **MotorDamiaoRobot**: 主要的硬件接口类
   - 实现`hardware_interface::SystemInterface`
   - 管理多个电机的配置和通信

2. **DaMiaoSocketCanDriver**: CAN总线驱动
   - 处理CAN帧的发送和接收
   - 管理电机命令和反馈数据队列

3. **DaMiaoMotionTranslation**: 协议转换器
   - 处理MIT模式控制协议
   - 编码/解码电机控制命令和反馈数据

### 数据流

```
ROS2 Controllers → MotorDamiaoRobot → DaMiaoSocketCanDriver → CAN Bus → Damiao Motors
                                  ←                        ←         ←
```

## 故障排除

### 常见问题

1. **CAN接口无法打开**
   ```bash
   # 检查CAN接口状态
   ip link show can0
   
   # 重新配置接口
   sudo ip link set can0 down
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```

2. **电机无响应**
   ```bash
   # 检查CAN总线流量
   candump can0
   
   # 发送测试帧
   cansend can0 123#DEADBEEF
   ```

3. **编译错误**
   ```bash
   # 清理并重新编译
   colcon build --packages-select motor_damiao_hardware --cmake-clean-cache
   ```

### 调试信息

启用详细日志输出：

```bash
ros2 run controller_manager ros2_control_node --ros-args --log-level DEBUG
```

## 贡献

欢迎提交问题报告和功能请求！

## 许可证

Apache License 2.0

## 更新日志

### v0.0.0
- 初始版本
- 实现基本的达妙电机硬件接口
- 支持位置、速度、力矩控制
- CAN总线通信实现
- 位置扩展功能
