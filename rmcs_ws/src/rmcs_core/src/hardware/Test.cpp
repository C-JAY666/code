#include <cstdint>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_description/tf_description.hpp>
#include <std_msgs/msg/int32.hpp>

#include <librmcs/client/dm_mc02.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/vesc_motor.hpp"

namespace rmcs_core::hardware {

class Test : public rmcs_executor::Component
           , public rclcpp::Node {
public:
    Test()
        : Node(
        get_component_name()
        , rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
            create_partner_component<TestCommand>(get_component_name() + "_command", *this))
    {
        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/steers/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                steers_calibrate_subscription_callback(std::move(msg));
            });

            register_output("/chassis/control_velocity", chassis_control_velocity_);

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

   class SBUS_Motor_Ctrl {
    public:
/*         using Gpio_Port = librmcs::client::GpioCtrl::Gpio_Port;
        using Gpio_Pin = librmcs::client::GpioCtrl::Gpio_Pin;
        using Gpio_State = librmcs::client::GpioCtrl::Gpio_State; */

        friend class DMH7_Board;
        friend class Myrobot;
        explicit SBUS_Motor_Ctrl(Test& test)
            : test_(test)
            ,remote_last_key_state(0)
            ,sa_last_key_state(0)
            ,sd_last_key_state(0)
            ,sbus_last_key_state(0){}
            
        void dbus_receive_callback(const std::byte* uart_data,
            uint8_t uart_data_length){

        // ========= sbus data printing ==========
        // ROS_INFO("SBUS data received: %d bytes, data hex: ", uart_data_length);
        // std::string hex_output = "Data (hex): ";
        // for (uint8_t j = 0; j < uart_data_length; ++j) {
        //     char hex_str[4];
        //     snprintf(hex_str, sizeof(hex_str), "%02X ", static_cast<uint8_t>(uart_data[j]));
        //     hex_output += hex_str;
        // }
        // ROS_INFO("%s", hex_output.c_str());
        // ========= sbus data printing ==========



        // SBUS 帧长度为 25 字节，以 0x0F 开头，以 0x00 结尾
        if (uart_data_length != 25 || uart_data[0] != std::byte{0x0F} || uart_data[24] != std::byte{0x00}) {
        // 如果不是有效的 SBUS 帧，则忽略
        return;
        }

        const uint8_t* payload = reinterpret_cast<const uint8_t*>(uart_data);

        auto sbus_map_channel = [](uint16_t val, uint16_t min_val, uint16_t mid_val, uint16_t max_val) -> int16_t {
        if (val < min_val || val > max_val) return 0; // 失效保护或超出范围

        // 在中点附近设置一个小的死区，以防止摇杆漂移
        if (std::abs(static_cast<int32_t>(val) - mid_val) < 20) {
        return 0;
        }

        double mapped_val;
        if (val > mid_val) {
        // 将 [mid_val, max_val] 映射到 [0, 360]
        mapped_val = (static_cast<double>(val) - mid_val) * 360.0 / (max_val - mid_val);
        } else { // val < mid_val
        // 将 [min_val, mid_val] 映射到 [-360, 0]
        mapped_val = (static_cast<double>(val) - mid_val) * 360.0 / (mid_val - min_val);
        }

        return static_cast<int16_t>(std::clamp(mapped_val, -360.0, 360.0));
        };

        // 解包通道并将其映射到我们的遥控器数据结构
        // 注意：通道映射（哪个摇杆/开关对应哪个通道）取决于发射机配置。
        // 假设 AETR 映射:
        // SBUS Ch1 (Aileron) -> 右摇杆水平 (ch0)
        // SBUS Ch2 (Elevator) -> 右摇杆垂直 (ch1)
        // SBUS Ch3 (Throttle) -> 左摇杆垂直 (ch2)
        // SBUS Ch4 (Rudder)   -> 左摇杆水平 (ch3)
        uint16_t ch1_raw = (payload[1]       | payload[2] << 8) & 0x07FF;
        uint16_t ch2_raw = (payload[2] >> 3  | payload[3] << 5) & 0x07FF;
        uint16_t ch3_raw = (payload[3] >> 6  | payload[4] << 2 | payload[5] << 10) & 0x07FF;
        uint16_t ch4_raw = (payload[5] >> 1  | payload[6] << 7) & 0x07FF;

        uint16_t ch5_raw  = (payload[6] >> 4  | payload[7] << 4) & 0x07FF;                   // SA toggle
        uint16_t ch6_raw  = (payload[7] >> 7  | payload[8] << 1 | payload[9] << 9) & 0x07FF; // SB 3pos
        uint16_t ch7_raw  = (payload[9] >> 2  | payload[10] << 6) & 0x07FF;                  // SC 3pos
        uint16_t ch8_raw  = (payload[10] >> 5 | payload[11] << 3) & 0x07FF;                  // SD toggle
        uint16_t ch9_raw  = (payload[12]      | payload[13] << 8) & 0x07FF;                  // SE Key
        uint16_t ch10_raw = (payload[13] >> 3 | payload[14] << 5) & 0x07FF;                  // SF wheel

        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch1_raw=%d, ch2_raw=%d, ch3_raw=%d, ch4_raw=%d", ch1_raw, ch2_raw, ch3_raw, ch4_raw); 
        SBUSData_.ch2 = -sbus_map_channel(ch1_raw, 174, 992, 1811); // 右摇杆水平
        SBUSData_.ch3 =  sbus_map_channel(ch2_raw, 174, 992, 1811); // 右摇杆垂直
        // SBUSData_.ch1 = sbus_map_channel(ch3_raw, 174, 992, 1811); // 左摇杆垂直
        SBUSData_.ch0 = sbus_map_channel(ch4_raw, 174, 992, 1811); // 左摇杆水平
        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch0=%d, ch1=%d, ch2=%d, ch3=%d", SBUSData_.ch0, SBUSData_.ch1, SBUSData_.ch2, SBUSData_.ch3);
        if (ch5_raw < 500) {
            SBUSData_.sa = 0; // SA toggle
            } else if (ch5_raw > 500) {
            SBUSData_.sa = 1;
            } else {
            SBUSData_.sa = 0; 
            }
    
            if (ch8_raw < 500) {
            SBUSData_.sd = 0; // SD toggle
            } else if (ch8_raw > 500) {
            SBUSData_.sd = 1;
            } else {
            SBUSData_.sd = 0; 
            }
    
            if (ch6_raw < 500) {
            SBUSData_.sb = 0; // SB 3pos
            } else if (ch6_raw > 1300) {
            SBUSData_.sb = 2;
            } else if (ch6_raw > 500) {
            SBUSData_.sb = 1; 
            }
    
            if (ch7_raw < 500) {
            SBUSData_.sc = 0; // SC 3pos
            } else if (ch7_raw > 1300) {
            SBUSData_.sc = 2;
            } else if (ch7_raw > 500) {
            SBUSData_.sc = 1; 
            }
    
            if (ch9_raw < 500) {
            SBUSData_.se = 0; // SE Key
            } else if (ch9_raw > 500) {
            SBUSData_.se = 1;
            } else {
            SBUSData_.se = 0; 
            }
    
            SBUSData_.sf = ch10_raw; // SF wheel, 174-1811
    
    
    
            // update RemoteData_ with SBUSData_
            SBUSData_.remote = 0; // 0 for sbus remote
            RemoteData_ = SBUSData_; 
            remoteHandler(RemoteData_); // 处理遥控器数据
    
    
            // 打印解析
            // ROS_INFO( "[SBUS] ch0: %d, ch1: %d, ch2: %d, ch3: %d, sa: %d, sb: %d, sc: %d, sd: %d, se: %d, sf: %d",
            //           SBUSData_.ch0, SBUSData_.ch1, SBUSData_.ch2, SBUSData_.ch3,
            //           SBUSData_.sa, SBUSData_.sb, SBUSData_.sc, SBUSData_.sd,
            //           SBUSData_.se, SBUSData_.sf);
    
            // 打印raw 摇杆数据
            // ROS_INFO("[SBUS] ch1_raw: %d, ch2_raw: %d, ch3_raw: %d, ch4_raw: %d",
            //          ch1_raw, ch2_raw, ch3_raw, ch4_raw);
    
            // ROS_INFO("[SBUS raw] ch1: %d, ch2: %d, ch3: %d, ch4: %d, "
            //          "ch5: %d, ch6: %d, ch7: %d, ch8: %d, "
            //          "ch9: %d, ch10: %d",
            //          ch1_raw, ch2_raw, ch3_raw, ch4_raw,
            //          ch5_raw, ch6_raw, ch7_raw, ch8_raw,
            //          ch9_raw, ch10_raw);
            }
        private:
        
            struct REMOTE_Data_t{
    
                uint8_t remote; // 0 = SBUS, 1 = NRF24L01
                
                int16_t ch0; //left joystick
                
                int16_t ch1;
                
                int16_t ch2; //right joystick
                
                int16_t ch3;
                
                uint16_t s0; // keys
                
                uint16_t s1; // toggle switch
                
                  
                
                uint16_t sa, sd; // toggle switches
                
                uint16_t sb, sc; // 3pos
                
                uint16_t se; // key
                
                uint16_t sf; // wheel
                
                REMOTE_Data_t() :remote(0), ch0(0), ch1(0) ,ch2(0), ch3(0), s0(0), s1(0), sa(0), sd(0), sb(0), sc(0), se(0), sf(0) {}
                
                };
                void remoteHandler(REMOTE_Data_t remoteData){
                    // std::cout << "ch2=" << remoteData.ch2 << " ch3=" << remoteData.ch3 << " s0=" << int(remoteData.s0) << " s1=" << int(remoteData.s1) << std::endl;
                    // ROS_INFO("handler ");
                
                    if (remoteData.remote == 1) {      // 1 for nrf remote
                        if (remoteData.s0 != 0 && remoteData.s0 != remote_last_key_state) {
                            switch (remoteData.s0) {
                                case 1:
                                    if (remoteData.s1 == 2){
                                        
                                    }
                                    else{
                                        
                                    }
                                    break;
                                case 2:
                                    // increaseGivenYaw(-0.1);
                                    if (remoteData.s1 == 2){
                                        
                                    }
                                    else {
                                        
                                    }
                                    break;
                                case 3:
                                    
                                    break;
                                case 4:
                                    
                                    
                                    break;
                                case 5:
                                  
                
                                    break;
                                case 6:
                                    
                                    break;
                                case 7:
                
                                    break;
                                case 8:
                                   
                                    break;
                                
                
                                default:
                                    break;
                            }
                        }
                
                        remote_last_key_state = remoteData.s0;
                    } 
                    
                    else if (remoteData.remote == 0) { // 2 for SBUS remote
                        
                        // ch3: 右摇杆垂直 -> vx（前后移动）
                        // ch2: 右摇杆水平 -> vy（左右移动）
                        double vx = remoteData.ch3 / 360.0 * 10.0;      // 归一化并限速到1m/s
                        double vy = remoteData.ch2 / 360.0 * 10.0;      // 归一化并限速到1m/s
                        double omega = remoteData.ch0 / 360.0 * 5.0;  
                        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch2=%d, ch3=%d", remoteData.ch2, remoteData.ch3); 
                        // // 控制底盘
                        test_.chassis_control_velocity_->vector[0] = vy;
                        test_.chassis_control_velocity_->vector[1] = -vx;
                        test_.chassis_control_velocity_->vector[2] = -omega;
                        // ================== SE 按键处理 =================
                            if (remoteData.se != remote_last_key_state) {
                            if (remoteData.se == 1) {
                                // SE 刚刚按下
                            } else {
                                // SE 刚刚松开
                            }
                        }
                
                        // SE 按下触发分支
                        if (remoteData.se != remote_last_key_state && remoteData.se == 1) { 
                            switch (remoteData.sc){
                                /* running */
                                case 0:
                                    if (remoteData.sb == 0) {                                                     
    
/*                                       myrobot_.board_->gpio_ctrl_->multiple_gpio_state()
                                                                    .set_gpio_config(Gpio_Port::GPIO_A, Gpio_Pin::Pin_3, Gpio_State::HIGH)
                                                                    .set_gpio_config(Gpio_Port::GPIO_E, Gpio_Pin::Pin_3, Gpio_State::LOW)
                                                                    .execute(); */
                                      
                                    } else if (remoteData.sb == 1) {
                                        
/*                                         myrobot_.board_->gpio_ctrl_->multiple_gpio_state()
                                                                   .set_gpio_config(Gpio_Port::GPIO_A, Gpio_Pin::Pin_3, Gpio_State::LOW)
                                                                   .set_gpio_config(Gpio_Port::GPIO_E, Gpio_Pin::Pin_3, Gpio_State::HIGH)
                                                                   .execute(); */
                                    } 
                                    else if (remoteData.sb == 2) {
                                        
                                    }
                                    break;
                
                                /* shooting */
                                case 1:
                                    if (remoteData.sb == 0) {
                                        
                                        
                                    } else if (remoteData.sb == 1) {
                                       
                                    } else if (remoteData.sb == 2) {
    
                                    }
                                       
                                    break;
                
                                /* dunking */
                                case 2:
                                    if (remoteData.sb == 0) {
                                        
                                    } else if (remoteData.sb == 1) {
                                        
                                    } else if (remoteData.sb == 2) {
                                       
                                    }
                                    break;
                
                            }
                        } else 
                        // SE held dow
                        if (remoteData.se == remote_last_key_state && remoteData.se == 1) {
                            switch (remoteData.sc){
                                case 0:
                                    if (remoteData.sb == 0) {
                                        // ch3: 右摇杆垂直 -> vx（前后移动）
                                        // ch2: 右摇杆水平 -> vy（左右移动）
                                        // double vx = remoteData.ch3 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                        // double vy = remoteData.ch2 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                        // double omega = 0.0;  
                                        // // 控制底盘
                                        // // myrobot_.chassis_ctrl_->set_raw_target_vel_(vx, vy, omega);
                                        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: vx=%f, vy=%f, omega=%f", vx, vy, omega);
                                    } else if (remoteData.sb == 1) {
                                        // ch3: 右摇杆垂直 -> vx（前后移动）
                                        // ch2: 右摇杆水平 -> omega（旋转速度）
                                        // double vx = remoteData.ch3 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                        // double omega = remoteData.ch2 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                        // double vy = 0.0;  
                                        // // 控制底盘
                                        // myrobot_.chassis_ctrl_->set_raw_target_vel_(vx, vy, omega);
                                        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: vx=%f, vy=%f, omega=%f", vx, vy, omega);
                                    } else if (remoteData.sb == 2) {
                                    }
                                    break;
                
                                case 1:
                                    if (remoteData.sb == 0) {
                                    } else if (remoteData.sb == 1) {
                                    } else if (remoteData.sb == 2) {
                                    }
                                    break;
                
                                case 2:
                                    if (remoteData.sb == 0) {
                
                                    } else  {
                
                                    } 
                                    break;
                            }
                        }
                        remote_last_key_state = remoteData.se;
                        // ================== SE 按键处理 END =================
                
                        // ================== SA 处理 =================
                        if (remoteData.sa != sa_last_key_state) {
                            if (remoteData.sa == 1) {
                                // SA 刚刚按下
                                // AIMLOCK_MODE = true; // 切换 AIMLOCK 模式
                            } else {
                                // SA 刚刚松开
                                // AIMLOCK_MODE = false; // 关闭 AIMLOCK 模式
                            }
                            sa_last_key_state = remoteData.sa; // 更新 SA 按键状态
                        }
                        // ================ SA 处理 END =================
                
                        // ================== SD 处理 =================
                        if (remoteData.sd != sd_last_key_state) {
                            if (remoteData.sd == 1) {
                                // SD 刚刚按下
                                // AIMLOCK_MODE = true; // 切换 AIMLOCK 模式
                            } else {
                                // SD 刚刚松开
                                // AIMLOCK_MODE = false; // 关闭 AIMLOCK 模式
                            }
                            sd_last_key_state = remoteData.sd; // 更新 SD 按键状态
                        }
                    } 
                    
                    else {
                    
                    }
                
                    /* manual arm angle */
                    // if (remoteData.s1 == 3){
                    //     setArmRealRadian(map<float>(remoteData.ch1,      -360, 360, ARM_INIT_THETA, ARM_MAX_THETA));
                    // }
                
                }
                
                Test& test_;
                REMOTE_Data_t SBUSData_;
                REMOTE_Data_t RemoteData_; // finally saved remote data unified
                uint8_t remote_last_key_state;
                uint8_t sa_last_key_state;
                uint8_t sd_last_key_state;
                uint8_t sbus_last_key_state;
    
                float pos = 0.0;
        };
class BottomBoard final : private librmcs::client::DMH7Board 
{
    public:
        friend class Test;
        explicit BottomBoard(
            Test& test, TestCommand& test_command, int usb_pid = -1)
            : librmcs::client::DMH7Board(usb_pid)
            , sbus_motor_ctrl_(std::make_shared<SBUS_Motor_Ctrl>(test))
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
            , event_thread_([this]()    { std::cout << "=== Event thread STARTED ===" << std::endl;
            handle_events(); 
            std::cout << "=== Event thread ENDED ===" << std::endl;})
            {
                gimbal_clip_turner_.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                    .set_reduction_ratio(11.)
                    .enable_multi_turn_angle()
                    .set_reversed());
                rack_suction_telescopic_.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::M2006}
                        .enable_multi_turn_angle()
                        .set_reversed()
                        .set_reduction_ratio(19 * 2));
            
/*                 for (auto& motor : chassis_wheel_motors_)
                    motor.configure(
                        device::VescMotor::Config()
                        .set_reversed()
                        .enable_multi_turn_angle() 
                    ); */
                    // 替换第113-118行
chassis_wheel_motors_[0].configure(
    device::VescMotor::Config()
     //.set_reversed()  // 左前：不反向
    .enable_multi_turn_angle());

chassis_wheel_motors_[1].configure(
    device::VescMotor::Config()
    //.set_reversed()  // 左后：反向
    .enable_multi_turn_angle());

chassis_wheel_motors_[2].configure(
    device::VescMotor::Config()
     //.set_reversed()  // 右后：不反向
    .enable_multi_turn_angle());

chassis_wheel_motors_[3].configure(
    device::VescMotor::Config()
    //.set_reversed()  // 右前：反向
    .enable_multi_turn_angle());
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


            //gimbal_clip_turner_.update_status();
            // rack_suction_telescopic_.update_status();
           // if (update_counter % 4 == 0) {
            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_steer_motors_)
                motor.update_status();


        //}
    }
        void command_update() {
            uint16_t can_commands[4];
            for (int i = 0; i < 4; i++) {
               double current_A = chassis_wheel_motors_[i].generate_command();
                int32_t current_mA = static_cast<int32_t>(current_A * 1000);
                uint32_t current_u32 = std::bit_cast<uint32_t>(current_mA);
               uint32_t current_be = __builtin_bswap32(current_u32);  
/*                double rpm_A = chassis_wheel_motors_[i].generate_command(); 
               int32_t erpm = static_cast<int32_t>(rpm_A);
               uint32_t rpm_be = __builtin_bswap32(std::bit_cast<uint32_t>(erpm)); */ 
               //RCLCPP_INFO(rclcpp::get_logger("BB"), " %d current: %d", i+1,rpm_be);
               
               
               uint8_t motor_id = i ;
               uint32_t can_id = motor_id | (3 << 8);

               transmit_buffer_.add_can1_transmission(can_id, static_cast<uint64_t>(current_be),
            true, false, 4);

            }
             //transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));
            // transmit_buffer_.add_can1_transmission(0x1FF, gimbal_clip_turner_.generate_command());

        
           if (can_transmission_mode) {
            transmit_buffer_.trigger_transmission();
                    for (int i = 0; i < 4; i++) {
                        can_commands[i] = chassis_steer_motors_[i].generate_command();
                        
                    }
                    transmit_buffer_.add_can2_transmission(
                        0x1FF, std::bit_cast<uint64_t>( can_commands));
                        transmit_buffer_.trigger_transmission();
                        //RCLCPP_INFO(rclcpp::get_logger("BB"), "Trigger Can2 transmission on port success");
                }
        else {
        transmit_buffer_.trigger_transmission();
        //RCLCPP_INFO(rclcpp::get_logger("BB"), "Trigger Can1 transmission on port success");

        }
                    // transmit_buffer_.add_can1_transmission(0x1FF, rack_suction_telescopic_.generate_command());
                    //  transmit_buffer_.add_can1_transmission(0x666, 0x777);/* 通信，没测 */

            //    can_transmission_cycle = (can_transmission_cycle +1 ) % 2 ;  
            can_transmission_mode = !can_transmission_mode;
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
                //std::cout << "[RX CAN1] ID=0x" << std::hex << can_id << std::dec << std::endl;  // 每次都输出
                
            if (is_remote_transmission ) [[unlikely]]
                return;
            
              if(is_extended_can_id){
                uint8_t motor_id = can_id & 0xFF;
                if (motor_id >= 0 && motor_id <= 3) {
                    chassis_wheel_motors_[motor_id].store_status(can_id, can_data);
                }
                return;
            }
             
            // DJI电机使用标准ID，需要8字节数据
/*              if (can_data_length < 8)
                return; 
             if(can_id >= 0x201 && can_id <= 0x204) {
                    chassis_wheel_motors_[can_id - 0x201].store_status(can_data);
                } */
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {

        //std::cout << "[RX CAN2] ID=0x" << std::hex << can_id << std::dec << std::endl;  // 每次都输出

            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
            if (can_id == 0x205)
                chassis_steer_motors_[0].store_status(can_data);
             
            else if (can_id == 0x206)
                chassis_steer_motors_[1].store_status(can_data);
   
            else if (can_id == 0x207)
                chassis_steer_motors_[2].store_status(can_data);

            else if (can_id == 0x208)
                chassis_steer_motors_[3].store_status(can_data);
    
        }

        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            // RCLCPP_INFO(myrobot.get_logger(), "UART: uart_data_length=%d", uart_data_length); 
            sbus_motor_ctrl_->dbus_receive_callback(uart_data, uart_data_length);
        }

    private:

    std::shared_ptr<SBUS_Motor_Ctrl> sbus_motor_ctrl_;

        bool can_transmission_mode = true;
        //uint8_t can_transmission_cycle = 0;

        device::DjiMotor gimbal_clip_turner_;
        device::DjiMotor rack_suction_telescopic_;
        device::VescMotor chassis_wheel_motors_[4];
        device::DjiMotor chassis_steer_motors_[4];

        librmcs::client::DMH7Board::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;


};
    std::shared_ptr<TestCommand> command_component_;
    std::shared_ptr<BottomBoard> bottom_board_;

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
};
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)

// ros2 topic pub /rack/suction_telescopic/control_torque std_msgs/msg/Float64 "{data: 0.3}"
