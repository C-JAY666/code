#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_description/tf_description.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/cboard.hpp>

#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class Test : public rmcs_executor::Component
           , public rclcpp::Node {
public:
    Test()
        : Node{
        get_component_name()
        , rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
            create_partner_component<TestCommand>(get_component_name() + "_command", *this))
    {
        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_,
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));

    }

    ~Test() override = default;
    
    void update() override {
        bottom_board_->update();
    }
    
    void command_update() {
        bottom_board_->command_update();
    }

private:
    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left front offset: %d",
            bottom_board_->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left back offset: %d",
            bottom_board_->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right back offset: %d",
            bottom_board_->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right front offset: %d",
            bottom_board_->chassis_steer_motors_[3].calibrate_zero_point());
    }
  
    class TestCommand : public rmcs_executor::Component
   {
    public:
        explicit TestCommand(Test& test)
            : test(test) {}

        void update() override { test.command_update(); }

        Test& test;
   };

class BottomBoard final : private librmcs::client::CBoard 
{
    public:
        friend class Test;
        explicit BottomBoard(
            Test& test, TestCommand& test_command, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , gimbal_clip_turner_(test, test_command, "/gimbal/clip_turner")
            , rack_suction_telescopic_(test, test_command, "/rack/suction_telescopic")
            , chassis_wheel_motors_(
                {test, test_command, "/chassis/left_front_wheel"},
                {test, test_command, "/chassis/left_back_wheel"},
                {test, test_command, "/chassis/right_back_wheel"},
                {test, test_command, "/chassis/right_front_wheel"})
            , chassis_steer_motors_(
                {test, test_command, "/chassis/left_front_steering"},
                {test, test_command, "/chassis/left_back_steering"},
                {test, test_command, "/chassis/right_back_steering"},
                {test, test_command, "/chassis/right_front_steering"})
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); })
            {
                gimbal_clip_turner_.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                        .enable_multi_turn_angle()
                        .set_reversed()
                        .set_reduction_ratio(19 * 2));
                rack_suction_telescopic_.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                        .enable_multi_turn_angle()
                        .set_reversed()
                        .set_reduction_ratio(19 * 2));
            
                for (auto& motor : chassis_wheel_motors_)
                    motor.configure(
                        device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                            .set_reduction_ratio(11)
                            .enable_multi_turn_angle()
                            .set_reversed());

                chassis_steer_motors_[0].configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                        .set_reversed()
                        .set_encoder_zero_point(
                            static_cast<int>(test.get_parameter("left_front_zero_point").as_int()))
                        .enable_multi_turn_angle());

                chassis_steer_motors_[1].configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                        .set_reversed()
                        .set_encoder_zero_point(
                            static_cast<int>(test.get_parameter("left_back_zero_point").as_int()))
                        .enable_multi_turn_angle());

                chassis_steer_motors_[2].configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                        .set_reversed()
                        .set_encoder_zero_point(
                            static_cast<int>(test.get_parameter("right_back_zero_point").as_int()))
                        .enable_multi_turn_angle());

                chassis_steer_motors_[3].configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                        .set_reversed()
                        .set_encoder_zero_point(
                            static_cast<int>(test.get_parameter("right_front_zero_point").as_int()))
                        .enable_multi_turn_angle());
            }

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            gimbal_clip_turner_.update_status();
            rack_suction_telescopic_.update_status();
            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();
        }
        void command_update() {
            uint16_t can_commands[4];
            for (int i = 0; i < 4; i++)
                can_commands[i] = chassis_wheel_motors_[i].generate_command();
                transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));
                transmit_buffer_.add_can1_transmission(0x1FF, gimbal_clip_turner_.generate_command());

                if (can_transmission_mode) {
                    for (int i = 0; i < 4; i++) {
                        can_commands[i] = chassis_steer_motors_[i].generate_command();
                    }
                    transmit_buffer_.add_can2_transmission(
                        0x1FE, std::bit_cast<uint64_t>( can_commands));
                    transmit_buffer_.trigger_transmission();
                } else {
                    transmit_buffer_.add_can1_transmission(0x1FF, rack_suction_telescopic_.generate_command());
                    // transmit_buffer_.add_can3_transmission(0x666, )
                    transmit_buffer_.trigger_transmission();
                }
                can_transmission_mode = !can_transmission_mode;
            
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x201)
                chassis_wheel_motors_[0].store_status(can_data);
            else if (can_id == 0x202)
                chassis_wheel_motors_[1].store_status(can_data);
            else if (can_id == 0x203)
                chassis_wheel_motors_[2].store_status(can_data);
            else if (can_id == 0x204)
                chassis_wheel_motors_[3].store_status(can_data);
        }
        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            else if (can_id == 0x205)
                chassis_steer_motors_[0].store_status(can_data);
            else if (can_id == 0x206)
                chassis_steer_motors_[1].store_status(can_data);
            else if (can_id == 0x207)
                chassis_steer_motors_[2].store_status(can_data);
            else if (can_id == 0x208)
                chassis_steer_motors_[3].store_status(can_data);
        }
    private:
        bool can_transmission_mode = true;

        device::DjiMotor gimbal_clip_turner_;
        device::DjiMotor rack_suction_telescopic_;
        device::DjiMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];

        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;


};
    std::shared_ptr<TestCommand> command_component_;
    std::shared_ptr<BottomBoard> bottom_board_;


    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
};
}



#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)

// ros2 topic pub /rack/suction_telescopic/control_torque std_msgs/msg/Float64 "{data: 0.3}"
