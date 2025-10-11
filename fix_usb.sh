#!/bin/bash

# 为RMCS USB设备添加权限并解除内核驱动占用
# vendor_id: 0xa11c

echo "==== RMCS USB设备权限修复工具 ===="
echo ""

# 检查是否在容器中
if [ -f /.dockerenv ]; then
    echo "检测到正在Docker容器中运行"
    echo ""
    echo "方案1: 直接修改当前连接的USB设备权限（临时方案）"
    echo "----------------------------------------"
    
    # 查找vendor id为0xa11c的USB设备
    USB_DEVICES=$(lsusb | grep "a11c:")
    
    if [ -z "$USB_DEVICES" ]; then
        echo "未检测到vendor ID为0xa11c的USB设备"
        echo "请确保设备已连接"
    else
        echo "找到以下设备："
        echo "$USB_DEVICES"
        echo ""
        
        # 提取Bus和Device编号
        BUS=$(echo "$USB_DEVICES" | awk '{print $2}')
        DEV=$(echo "$USB_DEVICES" | awk '{print $4}' | tr -d ':')
        
        DEVICE_PATH="/dev/bus/usb/$BUS/$DEV"
        
        if [ -e "$DEVICE_PATH" ]; then
            echo "设置设备权限: $DEVICE_PATH"
            sudo chmod 666 "$DEVICE_PATH"
            echo "✓ 权限已设置为666"
            echo ""
            
            # 尝试解绑内核驱动
            echo "检查并解绑可能占用的内核驱动..."
            
            # 查找对应的USB设备在sysfs中的路径
            USB_SYS_PATH="/sys/bus/usb/devices/$BUS-*"
            for usb_dev in $USB_SYS_PATH; do
                if [ -f "$usb_dev/idVendor" ]; then
                    VENDOR=$(cat "$usb_dev/idVendor" 2>/dev/null)
                    if [ "$VENDOR" = "a11c" ]; then
                        echo "找到设备: $usb_dev"
                        
                        # 解绑所有接口上的驱动
                        for interface in "$usb_dev"/*:1.*; do
                            if [ -d "$interface" ]; then
                                IFACE_NAME=$(basename "$interface")
                                if [ -e "$interface/driver" ]; then
                                    DRIVER=$(basename $(readlink "$interface/driver"))
                                    echo "  接口 $IFACE_NAME 被驱动 $DRIVER 占用，正在解绑..."
                                    echo "$IFACE_NAME" | sudo tee "$interface/driver/unbind" > /dev/null 2>&1
                                    if [ $? -eq 0 ]; then
                                        echo "  ✓ 已解绑 $DRIVER 驱动"
                                    else
                                        echo "  - 无需解绑或解绑失败"
                                    fi
                                else
                                    echo "  接口 $IFACE_NAME 没有绑定驱动"
                                fi
                            fi
                        done
                    fi
                fi
            done
            
            echo ""
            echo "✓ USB设备已准备就绪"
            echo "  注意：这是临时方案，重新插拔USB设备后需要重新运行此脚本"
        else
            echo "✗ 设备路径不存在: $DEVICE_PATH"
        fi
    fi
    
    echo ""
    echo "方案2: 在宿主机上永久配置（推荐）"
    echo "----------------------------------------"
    echo "请在宿主机（非Docker容器）上创建以下文件："
    echo ""
    echo "文件: /etc/udev/rules.d/99-rmcs.rules"
    echo "内容:"
    echo '  SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", MODE="0666"'
    echo '  SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", ATTR{idProduct}=="fdce", RUN+="/bin/sh -c '"'"'echo -n \$kernel > /sys/bus/usb/drivers/cdc_acm/unbind 2>/dev/null'"'"'"'
    echo ""
    echo "然后运行:"
    echo "  sudo udevadm control --reload-rules"
    echo "  sudo udevadm trigger"
    echo ""
    echo "最后重新插拔USB设备"
    
else
    echo "检测到在宿主机上运行，正在配置永久udev规则..."
    echo ""
    
    # 创建udev规则文件（包含权限和驱动解绑）
    cat << 'EOF' | sudo tee /etc/udev/rules.d/99-rmcs.rules > /dev/null
# RMCS USB设备权限和驱动配置
SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", MODE="0666"
# 自动解绑cdc_acm驱动（如果被占用）
SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", ATTR{idProduct}=="fdce", RUN+="/bin/sh -c 'for i in /sys/bus/usb/drivers/cdc_acm/*:1.1; do [ -e \$i ] && echo \$(basename \$i) > /sys/bus/usb/drivers/cdc_acm/unbind 2>/dev/null; done'"
EOF
    
    if [ $? -eq 0 ]; then
        echo "✓ udev规则已创建：/etc/udev/rules.d/99-rmcs.rules"
        
        # 重新加载udev规则
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        
        echo "✓ udev规则已重新加载"
        echo ""
        echo "配置完成！请重新插拔USB设备，然后重新运行程序。"
    else
        echo "✗ 错误：无法创建udev规则文件"
        exit 1
    fi
fi

echo ""
echo "===================================="
