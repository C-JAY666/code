#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::controller::chassis {

// 简单的底盘命令器，从 ROS2 话题接收速度命令
class SimpleChassisCommander
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleChassisCommander()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        // 订阅外部速度命令
        twist_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            rclcpp::QoS{10},
            [this](geometry_msgs::msg::Twist::UniquePtr msg) {
                current_twist_ = *msg;
                received_command_ = true;
            });
        
        // 输出到底盘控制系统
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::AUTO);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        
        RCLCPP_INFO(get_logger(), "SimpleChassisCommander: Listening on /cmd_vel");
        RCLCPP_INFO(get_logger(), "Usage: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...");
    }

    void update() override {
        if (!received_command_) {
            // 没有收到命令，输出零速度
            chassis_control_velocity_->vector << 0.0, 0.0, 0.0;
        } else {
            // 转换 Twist 消息到底盘速度
            chassis_control_velocity_->vector[0] = current_twist_.linear.x;   // 前进速度
            chassis_control_velocity_->vector[1] = current_twist_.linear.y;   // 侧移速度
            chassis_control_velocity_->vector[2] = current_twist_.angular.z;  // 旋转速度
        }
        
        // 设置为自动模式
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    geometry_msgs::msg::Twist current_twist_;
    bool received_command_ = false;
    
    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SimpleChassisCommander, rmcs_executor::Component)

