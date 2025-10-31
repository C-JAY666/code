#!/bin/bash
# 电机测试脚本

echo "================================"
echo "VESC电机测试脚本"
echo "================================"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未加载，正在加载..."
    source /workspaces/RMCS/rmcs_ws/install/setup.bash
else
    echo "✅ ROS2环境已加载: $ROS_DISTRO"
fi

echo ""
echo "请选择测试模式："
echo "1) 前进运动 (x=0.5 m/s)"
echo "2) 后退运动 (x=-0.5 m/s)"
echo "3) 左移运动 (y=0.5 m/s)"
echo "4) 右移运动 (y=-0.5 m/s)"
echo "5) 顺时针旋转 (z=0.5 rad/s)"
echo "6) 逆时针旋转 (z=-0.5 rad/s)"
echo "7) 组合运动 (前进+旋转)"
echo "8) 小速度测试 (x=0.1 m/s)"
echo "9) 停止所有电机"
echo "0) 自定义速度"
echo ""
read -p "请输入选项 (1-9, 0): " choice

case $choice in
    1)
        echo "发送命令: 前进 0.5 m/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    2)
        echo "发送命令: 后退 0.5 m/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    3)
        echo "发送命令: 左移 0.5 m/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    4)
        echo "发送命令: 右移 0.5 m/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    5)
        echo "发送命令: 顺时针旋转 0.5 rad/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
        ;;
    6)
        echo "发送命令: 逆时针旋转 0.5 rad/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
        ;;
    7)
        echo "发送命令: 前进+旋转"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
        ;;
    8)
        echo "发送命令: 小速度前进 0.1 m/s"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    9)
        echo "停止所有电机"
        ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        ;;
    0)
        echo "自定义速度:"
        read -p "  线速度 x (m/s): " vx
        read -p "  线速度 y (m/s): " vy
        read -p "  角速度 z (rad/s): " vz
        echo "发送命令: x=$vx, y=$vy, z=$vz"
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
            "{linear: {x: $vx, y: $vy, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $vz}}"
        ;;
    *)
        echo "❌ 无效选项"
        exit 1
        ;;
esac

echo ""
echo "================================"
echo "提示:"
echo "- 按 Ctrl+C 停止发送命令"
echo "- 运行 'bash test_motors.sh' 再次测试"
echo "- 查看调试输出以了解电机状态"
echo "================================"

