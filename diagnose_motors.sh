#!/bin/bash
# 电机诊断脚本

echo "================================"
echo "VESC电机诊断工具"
echo "================================"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未加载"
    exit 1
else
    echo "✅ ROS2环境: $ROS_DISTRO"
fi

echo ""
echo "正在检查系统状态..."
echo ""

# 检查节点是否运行
echo "📋 检查ROS2节点:"
if ros2 node list | grep -q "test_hardware"; then
    echo "  ✅ test_hardware 节点正在运行"
else
    echo "  ❌ test_hardware 节点未运行"
    echo "     请先启动: ros2 launch rmcs_bringup Test.launch.py"
fi

if ros2 node list | grep -q "simple_chassis_commander"; then
    echo "  ✅ simple_chassis_commander 节点正在运行"
else
    echo "  ❌ simple_chassis_commander 节点未运行"
fi

if ros2 node list | grep -q "steering_wheel_controller"; then
    echo "  ✅ steering_wheel_controller 节点正在运行"
else
    echo "  ❌ steering_wheel_controller 节点未运行"
fi

echo ""
echo "📋 检查关键话题:"

# 检查话题
topics=(
    "/cmd_vel"
    "/chassis/control_velocity"
    "/chassis/left_front_wheel/control_torque"
    "/chassis/left_front_wheel/velocity"
    "/chassis/left_front_wheel/current"
)

for topic in "${topics[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo "  ✅ $topic"
    else
        echo "  ❌ $topic (未找到)"
    fi
done

echo ""
echo "📋 当前话题数据 (采样1秒):"
echo ""

echo "➤ 速度命令 (/cmd_vel):"
timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "  (无数据)"

echo ""
echo "➤ 底盘控制速度 (/chassis/control_velocity):"
timeout 1 ros2 topic echo /chassis/control_velocity --once 2>/dev/null || echo "  (无数据)"

echo ""
echo "➤ 左前轮扭矩 (/chassis/left_front_wheel/control_torque):"
timeout 1 ros2 topic echo /chassis/left_front_wheel/control_torque --once 2>/dev/null || echo "  (无数据)"

echo ""
echo "➤ 左前轮速度反馈 (/chassis/left_front_wheel/velocity):"
timeout 1 ros2 topic echo /chassis/left_front_wheel/velocity --once 2>/dev/null || echo "  (无数据)"

echo ""
echo "➤ 左前轮电流反馈 (/chassis/left_front_wheel/current):"
timeout 1 ros2 topic echo /chassis/left_front_wheel/current --once 2>/dev/null || echo "  (无数据)"

echo ""
echo "================================"
echo "诊断建议:"
echo "================================"

# 检查/cmd_vel
cmd_vel_data=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null)
if [ -z "$cmd_vel_data" ]; then
    echo ""
    echo "⚠️  问题: 没有收到 /cmd_vel 命令"
    echo "   原因: 电机不动最可能的原因是没有速度命令"
    echo "   解决: 运行测试脚本发送命令"
    echo "         bash /workspaces/RMCS/test_motors.sh"
    echo ""
fi

# 检查控制扭矩
torque_data=$(timeout 1 ros2 topic echo /chassis/left_front_wheel/control_torque --once 2>/dev/null)
if [ -n "$torque_data" ]; then
    if echo "$torque_data" | grep -q "data: 0.0"; then
        echo ""
        echo "⚠️  问题: 控制扭矩为0"
        echo "   原因: 可能是没有收到速度命令，或者速度命令太小"
        echo "   解决: 发送更大的速度命令测试"
        echo ""
    fi
fi

# 检查电机反馈
velocity_data=$(timeout 1 ros2 topic echo /chassis/left_front_wheel/velocity --once 2>/dev/null)
if [ -z "$velocity_data" ]; then
    echo ""
    echo "⚠️  问题: 没有电机速度反馈"
    echo "   原因: CAN通信可能有问题，或电机未正确连接"
    echo "   解决:"
    echo "         1. 检查CAN接口: ip link show can0"
    echo "         2. 检查CAN总线: candump can0"
    echo "         3. 检查电机连接和供电"
    echo ""
fi

echo ""
echo "================================"
echo "完成诊断"
echo "================================"
echo ""
echo "更多信息请查看: /workspaces/RMCS/MOTOR_DEBUG_GUIDE.md"

