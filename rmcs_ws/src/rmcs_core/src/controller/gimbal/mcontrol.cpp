
#include <limits> 
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class Motor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Motor()
        : rclcpp::Node(
            get_component_name(),
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        
        auto set_pid_parameter = [this](pid::PidCalculator& apid, pid::PidCalculator& vpid, const std::string& name) {
            // 角度环参数
            apid.kp = get_parameter(name + "_angle_kp").as_double();
            apid.ki = get_parameter(name + "_angle_ki").as_double();
            apid.kd = get_parameter(name + "_angle_kd").as_double();
            get_parameter(name + "_angle_output_min", apid.output_min);
            get_parameter(name + "_angle_output_max", apid.output_max);

            // 速度环参数
            vpid.kp = get_parameter(name + "_velocity_kp").as_double();
            vpid.ki = get_parameter(name + "_velocity_ki").as_double();
            vpid.kd = get_parameter(name + "_velocity_kd").as_double();
            get_parameter(name + "_velocity_output_min", vpid.output_min);
            get_parameter(name + "_velocity_output_max", vpid.output_max);
        };
        
        set_pid_parameter(clip_turner_angle_pid_, clip_turner_velocity_pid_, "clip_turner");
        set_pid_parameter(rack_suction_telescopic_angle_pid_, rack_suction_telescopic_velocity_pid_, "rack_suction_telescopic");
        set_pid_parameter(rack_suction_updown_angle_pid_, rack_suction_updown_velocity_pid_, "rack_suction_updown");
        set_pid_parameter(rack_clip_updown_angle_pid_, rack_clip_updown_velocity_pid_, "rack_clip_updown");
        
        // 注册输入接口
        register_input("/clip/turner/angle", current_angles_[0]);
        register_input("/clip/turner/velocity", current_velocities_[0]);
        register_input("/clip/turner/target_angle", target_angles_[0], false);
        
        register_input("/rack/suction_telescopic/angle", current_angles_[1]);
        register_input("/rack/suction_telescopic/velocity", current_velocities_[1]);
        register_input("/rack/suction_telescopic/target_angle", target_angles_[1], false);
        
        register_input("/rack/suction_updown/angle", current_angles_[2]);
        register_input("/rack/suction_updown/velocity", current_velocities_[2]);
        register_input("/rack/suction_updown/target_angle", target_angles_[2], false);
        
        register_input("/rack/clip_updown/angle", current_angles_[3]);
        register_input("/rack/clip_updown/velocity", current_velocities_[3]);
        register_input("/rack/clip_updown/target_angle", target_angles_[3], false);
        
        // 注册输出接口
        register_output("/clip/turner/control_torque", control_torques_[0], 0.0);
        register_output("/rack/suction_telescopic/control_torque", control_torques_[1], 0.0);
        register_output("/rack/suction_updown/control_torque", control_torques_[2], 0.0);
        register_output("/rack/clip_updown/control_torque", control_torques_[3], 0.0);
    }

    void update() override {


        // 电机0: clip_turner
        *control_torques_[0] = double_pid(
            *target_angles_[0], *current_angles_[0]
            ,*current_velocities_[0],clip_turner_angle_pid_
            , clip_turner_velocity_pid_, *control_torques_[0]);
        
        // 电机1: rack_suction_telescopic
        *control_torques_[1] = double_pid(
            *target_angles_[1], *current_angles_[1]
            ,*current_velocities_[1],rack_suction_telescopic_angle_pid_
            , rack_suction_telescopic_velocity_pid_, *control_torques_[1]);
        
        // 电机2: rack_suction_updown
        *control_torques_[2] = double_pid(
            *target_angles_[2], *current_angles_[2]
            ,*current_velocities_[2],rack_suction_updown_angle_pid_
            , rack_suction_updown_velocity_pid_, *control_torques_[2]);
  
        
        // 电机3: rack_clip_updown
        *control_torques_[3] = double_pid(
            *target_angles_[3], *current_angles_[3]
            ,*current_velocities_[3],rack_clip_updown_angle_pid_
            , rack_clip_updown_velocity_pid_, *control_torques_[3]);
    }

private:
    static double double_pid(const double& target_angles, const double& current_angle, const double& current_velocity,
                    pid::PidCalculator& angle_pid, pid::PidCalculator& velocity_pid, double& control_torque)
    {
        double cv;
        if (!std::isnan(target_angles)) {
              cv = velocity_pid.update(
                angle_pid.update(target_angles - current_angle)
                - current_velocity
            );
            return ramp(current_velocity, cv, 100);
        } else {
            return nan_;
        }

    }

    static double ramp(double current_vel, double target_vel, double max_accel) {
        double diff = target_vel - current_vel;
        double max_delta = max_accel * 0.001;  // 1ms 周期
        
        if (std::abs(diff) > max_delta) {
            return current_vel + (diff > 0 ? max_delta : -max_delta);
        }
        return target_vel;
    }


    
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr int MOTOR_COUNT = 4;

    InputInterface<double> current_angles_[MOTOR_COUNT];      // 当前角度
    InputInterface<double> current_velocities_[MOTOR_COUNT];  // 当前速度
    InputInterface<double> target_angles_[MOTOR_COUNT];       // 目标角度

    pid::PidCalculator clip_turner_angle_pid_, rack_suction_telescopic_angle_pid_, rack_suction_updown_angle_pid_, rack_clip_updown_angle_pid_;
    pid::PidCalculator clip_turner_velocity_pid_, rack_suction_telescopic_velocity_pid_, rack_suction_updown_velocity_pid_, rack_clip_updown_velocity_pid_;

    OutputInterface<double> control_torques_[MOTOR_COUNT];
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::Motor, rmcs_executor::Component)