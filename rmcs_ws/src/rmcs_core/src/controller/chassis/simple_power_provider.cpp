#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

// 简单的固定功率提供者，用于测试（不限制功率）
class SimplePowerProvider
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimplePowerProvider()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        double power_limit = get_parameter("power_limit").as_double();
        
        register_output("/chassis/control_power_limit", control_power_limit_, power_limit);
        
        RCLCPP_INFO(get_logger(), "SimplePowerProvider initialized with power_limit: %.1f W", power_limit);
    }

    void update() override {
        // 什么都不做，只是持续输出固定功率值
    }

private:
    OutputInterface<double> control_power_limit_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SimplePowerProvider, rmcs_executor::Component)

