#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <eigen3/Eigen/Dense>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::broadcaster {

// 虚拟遥控器，提供默认输入（用于测试）
class DummyRemote
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DummyRemote()
        : Node(get_component_name()) {
        
        // 输出遥控器接口（全部设为0或默认值）
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::MIDDLE);
        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UP);
        register_output("/remote/keyboard", keyboard_, rmcs_msgs::Keyboard::zero());
        register_output("/remote/mouse", mouse_, rmcs_msgs::Mouse::zero());
        register_output("/remote/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());
        register_output("/remote/rotary_knob", rotary_knob_, 0.0);
        
        // IMU 输出（底盘航向角速度）
        register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0.0);
        
        RCLCPP_INFO(get_logger(), "DummyRemote: Providing default remote control and IMU inputs");
    }

    void update() override {
        // 什么都不做，只是保持输出默认值
    }

private:
    OutputInterface<Eigen::Vector2d> joystick_right_;
    OutputInterface<Eigen::Vector2d> joystick_left_;
    OutputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<rmcs_msgs::Switch> switch_left_;
    OutputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<rmcs_msgs::Mouse> mouse_;
    OutputInterface<Eigen::Vector2d> mouse_velocity_;
    OutputInterface<double> rotary_knob_;
    OutputInterface<double> chassis_yaw_velocity_imu_;
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::DummyRemote, rmcs_executor::Component)

