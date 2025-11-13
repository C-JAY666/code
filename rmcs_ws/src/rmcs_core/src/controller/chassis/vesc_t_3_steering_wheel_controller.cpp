#include <cmath>



#include <numbers>
#include <rclcpp/node.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>


#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"



namespace rmcs_core::controller::chassis {

class VescT3SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

public :
    explicit VescT3SteeringWheelController()
        : Node ( get_component_name()
        , rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))         
        , mass_(get_parameter("mass").as_double())
        , moment_of_inertia_(get_parameter("moment_of_inertia").as_double()) 
        ,vehicle_radius_(get_parameter("vehicle_radius").as_double())
        ,wheel_radius_(get_parameter("wheel_radius").as_double())
        ,friction_coefficient_(get_parameter("friction_coefficient").as_double())
        ,control_acceleration_filter_(5.0, 1000.0)
        ,chassis_velocity_expected_(Eigen::Vector3d::Zero())
        , chassis_translational_velocity_pid_(0.3, 0.0, 0.5)
        , chassis_angular_velocity_pid_(0.3, 0.0, 0.5)
        , cos_varphi_(1, 0, 0) // 0, pi/2, pi, 3pi/2        /////////////////////////////////////////////////////////////////////////////////////
        , sin_varphi_(0, 1,-1)
        , steering_velocity_pid_(0.15, 0.0, 0.0)
        , steering_angle_pid_(30.0, 0.0, 0.0)
        , wheel_velocity_pid_(0.002, 0.00, 0.001) {
        
        register_input("/chassis/front_steering/angle", front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);

        register_input("/chassis/front_steering/velocity", front_steering_velocity_);    
        register_input("/chassis/left_back_steering/velocity", left_back_steering_velocity_);
        register_input("/chassis/right_back_steering/velocity", right_back_steering_velocity_);

        register_input("/chassis/front_wheel/velocity", front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);

        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);
        register_input("/chassis/control_velocity", chassis_control_velocity_);

        register_output(
            "/chassis/front_steering/control_torque", front_steering_control_torque_);
        register_output(
            "/chassis/left_back_steering/control_torque", left_back_steering_control_torque_);
        register_output(
            "/chassis/right_back_steering/control_torque", right_back_steering_control_torque_);

        register_output(
            "/chassis/front_wheel/control_torque", front_wheel_control_torque_);
        register_output(
            "/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_);

    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        integral_yaw_angle_imu();

        auto steering_status = calculate_chassis_status();
        auto wheel_velocities = calculate_wheel_velocities();
        auto chassis_velocity = calculate_chassis_velocity(steering_status, wheel_velocities);

        auto chassis_status_expected = calculate_chassis_status_expected(chassis_velocity);
        auto chassis_control_velocity = calculate_chassis_control_velocity();

        auto chassis_control_acceleraeion = calculate_chassis_control_acceleration(
            chassis_status_expected.velocity, chassis_control_velocity);

        auto wheel_pid_torques = 
            calculate_wheel_pid_torques(steering_status, wheel_velocities,  chassis_status_expected);

        auto constrained_chassis_acceleration = constrain_chassis_control_acceleration(
            chassis_control_acceleraeion);
        auto filtered_chassis_acceleration = 
            odom_to_base_link_vector(control_acceleration_filter_.update(
            base_link_to_odom_vector(constrained_chassis_acceleration)));

        auto steering_torques = calculate_chassis_control_acceleration(
            steering_status, chassis_status_expected, filtered_chassis_acceleration);
        auto wheel_torques = calculate_wheel_control_torques(
            steering_status , filtered_chassis_acceleration, wheel_pid_torques);
 
        update_control_torques(steering_torques, wheel_torques);
        update_chassis_velocity_expected(filtered_chassis_acceleration);
    }

private:

    struct SteeringStatus {
        Eigen::Vector3d angle, cos_angle, sin_angle;
        Eigen::Vector3d velocity;
    };

    struct ChassisStatus {
        Eigen::Vector3d velocity;
        Eigen::Vector3d wheel_velocity_x, wheel_velocity_y;
    };
    void reset_all_controls() {
        control_acceleration_filter_.reset();

        chassis_yaw_angle_imu_ = 0.0;
        chassis_velocity_expected_ = Eigen::Vector3d::Zero();

        *front_steering_control_torque_ = 0.0;
        *left_back_steering_control_torque_ = 0.0;
        *right_back_steering_control_torque_ = 0.0;

        *front_wheel_control_torque_ = 0.0;
        *left_back_wheel_control_torque_ = 0.0;
        *right_back_wheel_control_torque_ = 0.0;

        }

    void integral_yaw_angle_imu(){
        chassis_yaw_angle_imu_ += *chassis_yaw_velocity_imu_ * dt_;
        chassis_yaw_angle_imu_ = std::fmod(chassis_yaw_angle_imu_, 2 * std::numbers::pi);/* 取余 */
    }

    SteeringStatus  calculate_chassis_status() {
        SteeringStatus steering_status;

        steering_status.angle = {
            *front_steering_angle_,
            *left_back_steering_angle_,
            *right_back_steering_angle_
        };
//感覺不用旋轉，可以轉遙控器
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();

        steering_status.velocity = {
            *front_steering_velocity_,
            *left_back_steering_velocity_,
            *right_back_steering_velocity_
        };

        return steering_status;
    }

    Eigen::Vector3d calculate_wheel_velocities() {
        return {
             *front_wheel_velocity_,
             *left_back_wheel_velocity_,
             *right_back_wheel_velocity_
        };
    }

    Eigen::Vector3d calculate_chassis_velocity(                    //正解
        const SteeringStatus& steering_status , const Eigen::Vector3d& wheel_velocities) const {
            Eigen::Vector3d velocity;
            double one_quarter_r = wheel_radius_ / 4.0;
            velocity.x() = one_quarter_r * wheel_velocities.dot(steering_status.cos_angle);
            velocity.y() = one_quarter_r * wheel_velocities.dot(steering_status.sin_angle);
            velocity.z() = one_quarter_r / vehicle_radius_
                         * (wheel_velocities[0] * steering_status.sin_angle[0]
                            - wheel_velocities[1] * std::cos(steering_status.angle[1] + 33.67 * std::numbers::pi / 180)            ///////////////////////////////////////////////////
                            + wheel_velocities[2] * std::cos(steering_status.angle[2] - 33.67 * std::numbers::pi / 180));
            return velocity;
        }

    ChassisStatus calculate_chassis_status_expected (const Eigen::Vector3d& chassis_velocity) {

        auto calculate_energy = [this](const Eigen::Vector3d& velocity) {
            return mass_* velocity.head<2>().squaredNorm()
                   +  moment_of_inertia_* velocity.z() * velocity.z();
        };

        auto chassis_energy = calculate_energy(chassis_velocity);
        auto chassis_energy_expected = calculate_energy(chassis_velocity_expected_);
        if(chassis_energy_expected > chassis_energy) {
            double k = std::sqrt(chassis_energy / chassis_energy_expected);      //??
            chassis_velocity_expected_ *= k;
        }

        ChassisStatus chassis_status_expected;
        chassis_status_expected.velocity =  odom_to_base_link_vector(chassis_velocity_expected_);

        const auto& [x, y, z] = chassis_status_expected.velocity;
        chassis_status_expected.wheel_velocity_x = x - vehicle_radius_ * z * sin_varphi_.array();
        chassis_status_expected.wheel_velocity_y = y + vehicle_radius_ * z * cos_varphi_.array();

        return chassis_status_expected;
    }

    Eigen::Vector3d calculate_chassis_control_velocity(){                               /////没有变遥控器坐标系
        Eigen::Vector3d chassis_control_velocity = chassis_control_velocity_->vector;
        return chassis_control_velocity;
    }

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_status_expected,
        const Eigen::Vector3d& chassis_control_velocity) {
        
        Eigen::Vector2d translational_control_velocity = chassis_control_velocity.head<2>();
        Eigen::Vector2d translational_velocity = chassis_status_expected.head<2>();
        Eigen::Vector2d translational_control_acceleration = 
               chassis_translational_velocity_pid_.update(
                translational_control_velocity - translational_velocity);

        const double& angular_control_velocity = chassis_control_velocity[2];
        const double& angular_velocity = chassis_status_expected[2];
        double angular_control_acceleration = 
               chassis_angular_velocity_pid_.update(
                angular_control_velocity - angular_velocity);

        Eigen::Vector3d chassis_control_acceleration;
        chassis_control_acceleration << translational_control_acceleration,
            angular_control_acceleration;

        if(chassis_control_acceleration.lpNorm<1>() <1e-2)
           chassis_control_acceleration.setZero();

        return chassis_control_acceleration;
    }

    Eigen::Vector3d calculate_wheel_pid_torques(
        const SteeringStatus& steering_status, const Eigen::Vector3d& wheel_velocities,
        const ChassisStatus& chassis_status_expected) {

        Eigen::Vector3d wheel_control_velocity = 
        chassis_status_expected.wheel_velocity_x.array() * steering_status.cos_angle.array()
        + chassis_status_expected.wheel_velocity_y.array() * steering_status.sin_angle.array();

        return wheel_velocity_pid_.update(wheel_control_velocity / wheel_radius_ - wheel_velocities );
    }

    Eigen::Vector3d constrain_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_acceleration) {

        Eigen::Vector2d translational_acceleration_direction = chassis_acceleration.head<2>();
        double translational_acceleration_max = translational_acceleration_direction.norm();
        if(translational_acceleration_max > 0.0)
            translational_acceleration_direction /= translational_acceleration_max;

        double angular_acceleration_max = chassis_acceleration.z();
        double angular_acceleration_direction = angular_acceleration_max > 0 ? 1.0 : -1.0;
        angular_acceleration_max *= angular_acceleration_direction;

        const double rhombus_right = friction_coefficient_ * g_;
        const double rhombus_top = rhombus_right * mass_ * vehicle_radius_ / moment_of_inertia_;

        QcpSolver::QuadraticConstraint no_power_constraint{
            .a = 0.0,
            .b = 0.0,
            .c = 0.0,                                                                               //gift
            .d = 0.0,
            .e = 0.0,
            .f = -1e10,
        };

        Eigen::Vector2d best_point = qcp_solver_.solve(
            {1.0, 0.2}, {translational_acceleration_max, angular_acceleration_max}, 
            {rhombus_right, rhombus_top}, {no_power_constraint});
 
        Eigen::Vector3d best_acceleration;
        best_acceleration << best_point.x() * translational_acceleration_direction,
        best_point.y() * angular_acceleration_direction;

        return best_acceleration;
    }

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected,
        const Eigen::Vector3d& chassis_acceleration){

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;
        const auto& [ax, ay, az] = chassis_acceleration;

        Eigen::Vector3d dot_r_squared =  chassis_status_expected.wheel_velocity_x.array().square()
                                       + chassis_status_expected.wheel_velocity_y.array().square();

        Eigen::Vector3d streeing_control_velocities = 
            vx * ay - vy * ax - vz * (vx * vx + vy * vy)
            + vehicle_radius_ * (az * vx - vz * (ax + vz * vy)) * cos_varphi_.array()
            + vehicle_radius_ * (az * vy - vz * (ay - vz * vx)) * sin_varphi_.array();

        Eigen::Vector3d steering_control_angles;

        for (int i = 0 ; i < streeing_control_velocities.size() ; ++i) {
            if(dot_r_squared[i] > 1e-2) {
                streeing_control_velocities[i] /= dot_r_squared[i];
                steering_control_angles[i] = std::atan2(
                    chassis_status_expected.wheel_velocity_x[i], 
                    chassis_status_expected.wheel_velocity_y[i]);
            } else {
                auto x = ax - vehicle_radius_ * (az * sin_varphi_[i] + 0 * cos_varphi_[i]);
                auto y = ay + vehicle_radius_ * (az * cos_varphi_[i] - 0 * sin_varphi_[i]);
                if(x * x + y * y > 1e-6) {
                    streeing_control_velocities[i] = 0.0;
                    steering_control_angles[i] = std::atan2(y, x);
                }
                else {
                    streeing_control_velocities[i] = nan_;
                    steering_control_angles[i] = nan_;
                }
            }
        }
        
        Eigen::Vector3d streeing_torques = steering_velocity_pid_.update(
            streeing_control_velocities
            + steering_angle_pid_.update(  
                (steering_control_angles - steering_status.angle).unaryExpr([](double diff) {
                    diff = std::fmod(diff, std::numbers::pi);
                    if(diff < -std::numbers::pi / 2){
                        diff += std::numbers::pi;
                    } else if (diff > std::numbers::pi / 2) {
                        diff -= std::numbers::pi;
                    }
                    return diff;
                }))
            - steering_status.velocity);

        return streeing_torques.unaryExpr([](double v) { return std::isnan(v) ? 0.0 : v ;});
    }

    Eigen::Vector3d calculate_wheel_control_torques(
        const SteeringStatus& steering_status, const  Eigen::Vector3d& chassis_acceleration,
        const Eigen::Vector3d& wheel_pid_torques) {
            
        const auto& [ax, ay, az] =  chassis_acceleration;
        Eigen::Vector3d wheel_torque = 
            wheel_radius_ 
            * (ax * mass_ * steering_status.cos_angle.array()
               + ay * mass_ * steering_status.sin_angle.array()
               + az * moment_of_inertia_
                    * ( steering_status.sin_angle.array() * cos_varphi_.array()
                       - steering_status.cos_angle.array() * sin_varphi_.array())
                       / vehicle_radius_)
            / 4.0;
        
        wheel_torque += wheel_pid_torques;

        return wheel_torque;
    }


    void update_control_torques(
        const Eigen::Vector3d& streeing_torques, const Eigen::Vector3d& wheel_torques) {
            *front_steering_control_torque_ = streeing_torques[0];
            *left_back_steering_control_torque_ = streeing_torques[1];
            *right_back_steering_control_torque_ = streeing_torques[2];

            *front_wheel_control_torque_ = wheel_torques[0];
            *left_back_wheel_control_torque_ = wheel_torques[1];
            *right_back_wheel_control_torque_ = wheel_torques[2];
        }
    
    void update_chassis_velocity_expected(const Eigen::Vector3d& chassis_acceleration) {
        auto acceleration_odom = base_link_to_odom_vector(chassis_acceleration);
        chassis_velocity_expected_ += dt_ * acceleration_odom;
    }
    Eigen::Vector3d base_link_to_odom_vector(Eigen::Vector3d vector) const
    {
        vector.head<2>() = Eigen::Rotation2Dd( chassis_yaw_angle_imu_ ) * vector.head<2>();
        return vector;
    }
     

    Eigen::Vector3d odom_to_base_link_vector(Eigen::Vector3d vector) const
    {
        vector.head<2>() = Eigen::Rotation2Dd( -chassis_yaw_angle_imu_ ) * vector.head<2>();
        return vector;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    
    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.81;

    double mass_;
    double moment_of_inertia_;
    const double vehicle_radius_;   
    const double wheel_radius_;
    const double friction_coefficient_;

    InputInterface<double> front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;

    InputInterface<double> front_steering_velocity_;
    InputInterface<double> left_back_steering_velocity_;
    InputInterface<double> right_back_steering_velocity_;

    InputInterface<double> front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;

    OutputInterface<double> front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;

    QcpSolver qcp_solver_;
    filter::LowPassFilter<3> control_acceleration_filter_;

    double chassis_yaw_angle_imu_ = 0.0;
    Eigen::Vector3d chassis_velocity_expected_ = Eigen::Vector3d::Zero();

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;

    const Eigen::Vector3d cos_varphi_, sin_varphi_;

    pid::MatrixPidCalculator<3> steering_velocity_pid_, steering_angle_pid_, wheel_velocity_pid_;

};
}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::VescT3SteeringWheelController, rmcs_executor::Component)