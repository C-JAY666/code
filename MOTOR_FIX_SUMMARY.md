# 电机问题修复总结

## 问题描述
VESC电机不动

## 根本原因
**电机不动的主要原因是系统没有收到速度命令**

系统设计为：只有当接收到 `/cmd_vel` 话题的速度命令时，才会计算并输出电机扭矩。如果没有命令，系统会输出零扭矩，电机保持静止。

## 代码修复

### 1. 修复了 `vesc_motor.hpp` 中的Bug
**文件**: `/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/device/vesc_motor.hpp`

**问题**: 在 `generate_command()` 函数中，`return` 语句位置错误，导致后续的调试代码永远不会执行。

**修复前**:
```cpp
double generate_command(double max_current = 68) const {
    double torque = control_torque();
    return torque_to_current(torque, max_current);
    static int count = 0;  // 永远不会执行
    if (++count % 100 == 0 && torque != 0.0) {
        std::cout << "[VescMotor] torque=" << torque << std::endl;
    }
}
```

**修复后**:
```cpp
double generate_command(double max_current = 68) const {
    double torque = control_torque();
    return torque_to_current(torque, max_current);
}
```

### 2. 添加调试输出到 `Test.cpp`
**文件**: `/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/hardware/Test.cpp`

在 `command_update()` 函数中添加了调试输出，每1000次循环输出一次电机电流值（当电流非零时）：

```cpp
if (++debug_counter % 1000 == 0 && current_A != 0.0) {
    std::cout << "[Motor " << (int)motor_id << "] current=" << current_A 
              << "A, CAN_ID=0x" << std::hex << can_id << std::dec << std::endl;
}
```

同时修复了代码缩进问题。

### 3. 添加调试输出到 `steering_wheel_controller.cpp`
**文件**: `/workspaces/RMCS/rmcs_ws/src/rmcs_core/src/controller/chassis/steering_wheel_controller.cpp`

在 `update_control_torques()` 函数中添加了扭矩输出调试：

```cpp
static int debug_counter = 0;
if (++debug_counter % 1000 == 0) {
    double torque_sum = wheel_torques.array().abs().sum();
    if (torque_sum > 0.001) {
        RCLCPP_INFO(get_logger(), "[Wheel Torques] LF=%.3f LB=%.3f RB=%.3f RF=%.3f", 
            wheel_torques[0], wheel_torques[1], wheel_torques[2], wheel_torques[3]);
    }
}
```

## 新增工具

### 1. 电机调试指南
**文件**: `/workspaces/RMCS/MOTOR_DEBUG_GUIDE.md`

详细的问题排查和调试指南，包括：
- 系统架构说明
- 快速测试步骤
- 诊断检查清单
- 常见问题和解决方案
- 参数调整建议

### 2. 电机测试脚本
**文件**: `/workspaces/RMCS/test_motors.sh`

交互式测试脚本，提供9种预设的运动模式：
```bash
bash /workspaces/RMCS/test_motors.sh
```

选项包括：
1. 前进运动
2. 后退运动
3. 左移运动
4. 右移运动
5. 顺时针旋转
6. 逆时针旋转
7. 组合运动
8. 小速度测试
9. 停止所有电机
0. 自定义速度

### 3. 电机诊断脚本
**文件**: `/workspaces/RMCS/diagnose_motors.sh`

自动诊断系统状态的脚本：
```bash
bash /workspaces/RMCS/diagnose_motors.sh
```

检查内容：
- ROS2节点运行状态
- 关键话题存在性
- 当前话题数据
- 自动诊断建议

## 快速使用指南

### 第一步：编译修改后的代码
```bash
cd /workspaces/RMCS/rmcs_ws
colcon build --packages-select rmcs_core
source install/setup.bash
```

### 第二步：启动系统
```bash
ros2 launch rmcs_bringup Test.launch.py
```

### 第三步：诊断系统状态（可选）
在另一个终端：
```bash
source /workspaces/RMCS/rmcs_ws/install/setup.bash
bash /workspaces/RMCS/diagnose_motors.sh
```

### 第四步：测试电机
在另一个终端：
```bash
source /workspaces/RMCS/rmcs_ws/install/setup.bash
bash /workspaces/RMCS/test_motors.sh
```

或者直接发送ROS命令：
```bash
# 前进测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 第五步：观察调试输出
在启动系统的终端中，应该能看到：
```
[Wheel Torques] LF=0.123 LB=0.456 RB=0.789 RF=0.012
[Motor 1] current=2.46A, CAN_ID=0x101
[Motor 2] current=9.12A, CAN_ID=0x102
```

## 系统工作流程

```
用户发送速度命令
    ↓
ros2 topic pub /cmd_vel ...
    ↓
SimpleChassisCommander 接收命令
    ↓
输出 /chassis/control_velocity
    ↓
SteeringWheelController 计算所需加速度和扭矩
    ↓
输出 /chassis/xxx_wheel/control_torque
    ↓
VescMotor 将扭矩转换为电流
    ↓
通过CAN总线发送电流命令
    ↓
VESC电机执行
```

## 关键配置参数

### VESC电机配置 (在 Test.cpp 中)
```cpp
device::VescMotor::Config()
    .set_reversed()           // 反转方向
    .enable_multi_turn_angle() // 启用多圈角度
```

### 默认参数 (在 librmcs/device/vesc_motor.hpp 中)
```cpp
pole_pairs(7)      // 极对数
kt_rough(0.05)     // 转矩常数，影响电流计算
```

### 控制器参数 (在 Test.yaml 中)
```yaml
steering_wheel_controller:
  ros__parameters:
    mess: 19.0                    # 车体质量 (kg)
    wheel_radius: 0.055           # 轮子半径 (m)
    vehicle_radius: 0.24678       # 车体半径 (m)
```

## 如果电机仍然不动

1. **检查CAN通信**:
   ```bash
   ip link show can0
   candump can0
   ```

2. **检查电机配置**:
   - 使用VESC Tool连接电机
   - 确认电机ID (应该是 1, 2, 3, 4)
   - 测试手动控制

3. **调整转矩常数**:
   如果电流太小，可以临时降低 `kt_rough` 值来增加电流

4. **查看完整日志**:
   ```bash
   ros2 launch rmcs_bringup Test.launch.py 2>&1 | tee motor_test.log
   ```

## 相关文件清单

### 修改的文件
- `rmcs_ws/src/rmcs_core/src/hardware/device/vesc_motor.hpp`
- `rmcs_ws/src/rmcs_core/src/hardware/Test.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/steering_wheel_controller.cpp`

### 新增的文件
- `MOTOR_DEBUG_GUIDE.md` - 详细调试指南
- `test_motors.sh` - 电机测试脚本
- `diagnose_motors.sh` - 系统诊断脚本
- `MOTOR_FIX_SUMMARY.md` - 本文件

## 总结

电机不动的问题主要是因为**系统设计需要外部速度命令**。修复了代码中的一个小bug，并添加了完善的调试工具和文档。现在您可以使用提供的脚本快速测试电机，并通过诊断工具排查任何问题。

如有任何问题，请参考 `MOTOR_DEBUG_GUIDE.md` 获取更详细的信息。

