
#include <rmcs_executor/component.hpp>
#include "rclcpp/node.hpp"
#include <eigen3/Eigen/Dense>
#include <rmcs_description/tf_description.hpp>



namespace rmcs_core::controller::chassis {  

class VescSteeringWheelController
    :public rmcs_executor::Component
    ,public rclcpp::Node {

public:
    explicit VescSteeringWheelController()
    :Node(
        get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
              ,wheel_radius_(get_parameter("wheel_radius").as_double())
    {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);

        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_steering/velocity", left_front_steering_velocity_);
        register_input("/chassis/left_back_steering/velocity", left_back_steering_velocity_);
        register_input("/chassis/right_back_steering/velocity", right_back_steering_velocity_);
        register_input("/chassis/right_front_steering/velocity", right_front_steering_velocity_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/control_velocity", chassis_control_velocity_);


        register_output(
            "/chassis/left_front_steering/control_torque", left_front_steering_control_torque_);
        register_output(
            "/chassis/left_back_steering/control_torque", left_back_steering_control_torque_);
        register_output(
            "/chassis/right_back_steering/control_torque", right_back_steering_control_torque_);
        register_output(
            "/chassis/right_front_steering/control_torque", right_front_steering_control_torque_);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_wheel_control_velocity_);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_wheel_control_velocity_);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_wheel_control_velocity_);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_wheel_control_velocity_);
    }

    void update() override {
        if(std::isnan(chassis_control_velocity_->vector[0])) {
            Eigen::Vector4d streering_torques = Eigen::Vector4d::Zero();
            Eigen::Vector4d wheel_velocities = Eigen::Vector4d::Zero();
            update_control_torques(streering_torques, wheel_velocities);
            return;
        }
        
        double v_x = chassis_control_velocity_->vector[0];
        double v_y = chassis_control_velocity_->vector[1];

        double v = std::sqrt(v_x * v_x + v_y * v_y)/wheel_radius_;

        Eigen::Vector4d steering_torques = Eigen::Vector4d::Zero();
        
        Eigen::Vector4d wheel_velocities (v, v, v, v);

        update_control_torques(steering_torques, wheel_velocities);
    }



    void update_control_torques(
        const Eigen::Vector4d& steering_torques, const Eigen::Vector4d& wheel_velocities) {
        *left_front_steering_control_torque_ = steering_torques[0];
        *left_back_steering_control_torque_ = steering_torques[1];
        *right_back_steering_control_torque_ = steering_torques[2];
        *right_front_steering_control_torque_ = steering_torques[3];

        *left_front_wheel_control_velocity_ = wheel_velocities[0];
        *left_back_wheel_control_velocity_ = wheel_velocities[1];
        *right_back_wheel_control_velocity_ = wheel_velocities[2];
        *right_front_wheel_control_velocity_ = wheel_velocities[3];
    }

private:


const double wheel_radius_;


InputInterface<Eigen::Vector2d> joystick_right_;
InputInterface<Eigen::Vector2d> joystick_left_;

InputInterface<double> left_front_steering_angle_;
InputInterface<double> left_back_steering_angle_;
InputInterface<double> right_back_steering_angle_;
InputInterface<double> right_front_steering_angle_;

InputInterface<double> left_front_steering_velocity_;
InputInterface<double> left_back_steering_velocity_;
InputInterface<double> right_back_steering_velocity_;
InputInterface<double> right_front_steering_velocity_;

InputInterface<double> left_front_wheel_velocity_;
InputInterface<double> left_back_wheel_velocity_;
InputInterface<double> right_back_wheel_velocity_;
InputInterface<double> right_front_wheel_velocity_;

InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

OutputInterface<double> left_front_steering_control_torque_;
OutputInterface<double> left_back_steering_control_torque_;
OutputInterface<double> right_back_steering_control_torque_;
OutputInterface<double> right_front_steering_control_torque_;

OutputInterface<double> left_front_wheel_control_velocity_;
OutputInterface<double> left_back_wheel_control_velocity_;
OutputInterface<double> right_back_wheel_control_velocity_;
OutputInterface<double> right_front_wheel_control_velocity_;

};
}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::VescSteeringWheelController, rmcs_executor::Component)