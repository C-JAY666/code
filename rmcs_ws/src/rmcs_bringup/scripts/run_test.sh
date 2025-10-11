#!/bin/bash

# RMCS测试启动脚本 - 自动处理USB权限和驱动detach

echo "==== RMCS 启动脚本 ====\n"

# 1. 设置USB权限
echo "步骤 1: 设置USB设备权限..."
/workspaces/RMCS/fix_usb.sh | grep -E "(✓|❌|找到以下设备)"

# 2. 手动detach内核驱动（确保成功）
echo -e "\n步骤 2: 确保内核驱动已detach..."
USB_DEV_PATH=$(ls -d /sys/bus/usb/devices/*-* 2>/dev/null | while read dev; do
    if [ -f "$dev/idVendor" ]; then
        vendor=$(cat "$dev/idVendor" 2>/dev/null)
        if [ "$vendor" = "a11c" ]; then
            echo "$dev"
            break
        fi
    fi
done)

if [ -n "$USB_DEV_PATH" ]; then
    echo "  USB设备: $(basename $USB_DEV_PATH)"
    for iface in "$USB_DEV_PATH":1.*; do
        if [ -d "$iface" ]; then
            iface_num=$(basename "$iface")
            if [ -L "$iface/driver" ]; then
                driver=$(basename $(readlink "$iface/driver"))
                echo "  接口 $iface_num 有驱动 $driver，正在解绑..."
                echo "$iface_num" | sudo tee "$iface/driver/unbind" > /dev/null 2>&1
                if [ $? -eq 0 ]; then
                    echo "  ✓ 已解绑"
                fi
            else
                echo "  接口 $iface_num 没有驱动 ✓"
            fi
        fi
    done
else
    echo "  ⚠ 未找到RMCS设备"
fi

# 3. 启动RMCS
echo -e "\n步骤 3: 启动RMCS..."
echo "====================================\n"

cd /workspaces/RMCS/rmcs_ws
exec ros2 launch rmcs_bringup rmcs.launch.py robot:=Test

