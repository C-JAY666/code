#pragma once
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ros/ros.h>
#include <librmcs/client/dm_mc02.hpp>
#include <librmcs/utility/endian_promise.hpp>

struct VESCStatus {
  float rpm;            // ERPM
  float current;        // A
  float duty;           // %
  float ah_used;
  float ah_charged;
  float wh_used;
  float wh_charged;
  float temp_fet;
  float temp_motor;
  float current_in;
  float pid_pos;        // 本次读到的 0–360° 角度

  // 新增两项：
  float last_pid_pos;   // 上一次读到的 pid_pos
  double total_angle;   // 累积角度（°），正转＋，反转－

  float tachometer;
  float voltage_in;
  float adc1, adc2, adc3, ppm;

  bool fault;
  uint32_t error_code;
};

class VESC {
public:
  VESC()
    : motor_status_{},
      Motor_ID(0)
  {
    motor_status_.last_pid_pos = std::numeric_limits<float>::quiet_NaN();
    motor_status_.total_angle  = 0.0;
  }

  explicit VESC(uint8_t id)
    : motor_status_{},
      Motor_ID(id)
  {
    motor_status_.last_pid_pos = std::numeric_limits<float>::quiet_NaN();
    motor_status_.total_angle  = 0.0;
  }

  enum class COMM_PACKET_ID : uint32_t {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_CURRENT_REL = 10,
    CAN_PACKET_SET_CURRENT_BRAKE_REL,
    CAN_PACKET_SET_CURRENT_HANDBRAKE,
    CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
    CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
  };

  enum class STATUS_PACKET_ID : uint32_t {
    CAN_PACKET_STATUS   = 9,   // ERPM, Current, Duty Cycle
    CAN_PACKET_STATUS_2 = 14,  // Ah Used, Ah Charged
    CAN_PACKET_STATUS_3 = 15,  // Wh Used, Wh Charged
    CAN_PACKET_STATUS_4 = 16,  // Temp Fet, Temp Motor, Current In, PID position
    CAN_PACKET_STATUS_5 = 27,  // Tachometer, Voltage In
    CAN_PACKET_STATUS_6 = 28,  // ADC1, ADC2, ADC3, PPM
  };

  void ProcessCANMessage(uint32_t can_id,
                             uint64_t can_data,
                             uint8_t len)
    {
    if ((can_id & 0xFF) != Motor_ID) return;
    uint8_t cmd = (can_id >> 8) & 0xFF;

    #define BE32(shift) __builtin_bswap32(static_cast<int32_t>(  \
                            can_data >> (shift) ))
    #define BE16(shift) __builtin_bswap16(static_cast<int16_t>(  \
                            can_data >> (shift) ))

    switch (static_cast<STATUS_PACKET_ID>(cmd)) {
        case STATUS_PACKET_ID::CAN_PACKET_STATUS:
        if (len >= 8) {
          // ERPM   = B0–B3 → [3:0] big‑endian, 带符号
          int32_t raw_rpm      = static_cast<int32_t>(BE32(0));
          motor_status_.rpm    = static_cast<float>(raw_rpm);
          // Current= B4–B5 → [5:4], scale=10
          motor_status_.current= BE16(32) / 10.0f;
          // Duty   = B6–B7 → [7:6], scale=1000
          motor_status_.duty   = BE16(48) / 1000.0f;
        }
        break;


        case STATUS_PACKET_ID::CAN_PACKET_STATUS_2:
        if (len >= 8) {
            // Ah Used   (B0–B3) scale=10000
            motor_status_.ah_used    = static_cast<float>(BE32(0))  / 10000.0f;
            // Ah Charged(B4–B7) scale=10000
            motor_status_.ah_charged = static_cast<float>(BE32(32)) / 10000.0f;
        }
        break;

        case STATUS_PACKET_ID::CAN_PACKET_STATUS_3:
        if (len >= 8) {
            // Wh Used   (B0–B3) scale=10000
            motor_status_.wh_used    = static_cast<float>(BE32(0))  / 10000.0f;
            // Wh Charged(B4–B7) scale=10000
            motor_status_.wh_charged = static_cast<float>(BE32(32)) / 10000.0f;
        }
        break;

        case STATUS_PACKET_ID::CAN_PACKET_STATUS_4:
        if (len >= 8) {
            // Temp FET   (B0–B1) scale=10
            motor_status_.temp_fet   = static_cast<float>(BE16(0))  / 10.0f;
            // Temp Motor (B2–B3) scale=10
            motor_status_.temp_motor = static_cast<float>(BE16(16)) / 10.0f;
            // Current In (B4–B5) scale=10
            motor_status_.current_in = static_cast<float>(BE16(32)) / 10.0f;
            // PID Pos    (B6–B7) scale=50
            motor_status_.pid_pos    = static_cast<float>(BE16(48)) / 50.0f;

            // ROS_INFO("VESC PID pos = %.2f° (raw=0x%04X)",
            //         motor_status_.pid_pos,
            //         BE16(48));
        }
        break;

        case STATUS_PACKET_ID::CAN_PACKET_STATUS_5:
        if (len >= 6) {
            // Tachometer (B0–B3) scale=6
            motor_status_.tachometer = static_cast<float>(BE32(0))  / 6.0f;
            // Voltage In (B4–B5) scale=10
            motor_status_.voltage_in = static_cast<float>(BE16(32)) / 10.0f;
        }
        break;

        case STATUS_PACKET_ID::CAN_PACKET_STATUS_6:
        if (len >= 8) {
            // ADC1 (B0–B1) scale=1000
            motor_status_.adc1 = static_cast<float>(BE16(0))  / 1000.0f;
            // ADC2 (B2–B3) scale=1000
            motor_status_.adc2 = static_cast<float>(BE16(16)) / 1000.0f;
            // ADC3 (B4–B5) scale=1000
            motor_status_.adc3 = static_cast<float>(BE16(32)) / 1000.0f;
            // PPM  (B6–B7) scale=1000
            motor_status_.ppm  = static_cast<float>(BE16(48)) / 1000.0f;
        }
        break;

        default:
        break;
    }

    #undef BE32
    #undef BE16
    }




  const VESCStatus& motorStatus() const {
    return motor_status_;
  }

  bool VESC_Command(uint8_t can_port,
                    COMM_PACKET_ID command,
                    librmcs::client::DMH7Board::TransmitBuffer& txbuf,
                    uint32_t send_value)
  {
    using namespace librmcs::utility;
    uint32_t converted_value = swap_endian(send_value);
    bool res = false;
    switch (can_port) {
      case 1:
        res = txbuf.add_can1_transmission(
          Motor_ID | (static_cast<uint32_t>(command) << 8),
          static_cast<uint64_t>(converted_value),
          true, false, 4);
        break;
      case 2:
        res = txbuf.add_can2_transmission(
          Motor_ID | (static_cast<uint32_t>(command) << 8),
          static_cast<uint64_t>(converted_value),
          true, false, 4);
        break;
      case 3:
        res = txbuf.add_can3_transmission(
          Motor_ID | (static_cast<uint32_t>(command) << 8),
          static_cast<uint64_t>(converted_value),
          true, false, 4);
        break;
      default:
        ROS_ERROR("Invalid CAN port: %d", can_port);
        return false;
    }
    if (!res || !txbuf.trigger_transmission()) {
      ROS_ERROR("Failed to trigger VESC transmission on port %d", can_port);
      return false;
    }
    return true;
  }

public:
  librmcs::utility::be_uint32_t Motor_ID;

private:
  VESCStatus motor_status_;
};

// /**
//   * @brief Calculate the floating-point ramp filter.
//   * @param input: the filter input variables
//   * @param target: the input variables target value
//   * @param ramp: the ramp slope
//   * @retval the filter output
//   */
// float f_Ramp_Calc(float input,float target,float ramp)
// {
//   float error = target - input;
//   float output = input;

// 	if (error > 0){
//         if (error > ramp){output += ramp;}   
//         else{output += error;}
//     }else{
//         if (error < -ramp){output += -ramp;}
//         else{output += error;}
//     }

//     return output;
// }