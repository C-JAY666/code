#!/bin/bash

# 简单的RMCS测试启动脚本

cd "$(dirname "$0")"

echo "================================"
echo "RMCS 测试启动"
echo "================================"
echo ""

# 步骤 1: 修复 USB 权限
echo "步骤 1: 修复 USB 设备权限..."
sudo /workspaces/RMCS/fix_usb.sh > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ USB 权限已设置"
else
    echo "⚠ 无法设置 USB 权限，可能需要手动运行: sudo /workspaces/RMCS/fix_usb.sh"
fi

echo ""
echo "步骤 2: 启动 RMCS..."
echo "================================"
echo ""

# 使用 bash 来 source 避免 zsh 兼容性问题
bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch rmcs_bringup rmcs.launch.py robot:=Test"

