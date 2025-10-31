#pragma once

#include <iostream>
#include <librmcs/device/vesc_motor.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class VescMotor : public librmcs::device::VescMotor {
public:
    VescMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : librmcs::device::VescMotor(){
        std::cout << "VescMotor constructor: Registering " << name_prefix << std::endl;
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/current", current_, 0.0);
        status_component.register_output(name_prefix + "/temperature", temperature_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
        command_component.register_input(name_prefix + "/control_velocity", control_velocity_, false);
        std::cout << "VescMotor constructor: Registered all interfaces for " << name_prefix << std::endl;
        }

    VescMotor(
    rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
    const std::string& name_prefix, const Config& config)
    :VescMotor(status_component, command_component, name_prefix) {
     configure(config);
    }
        
    void configure(const Config& config) {
        librmcs::device::VescMotor::configure(config);
        *max_torque_ = max_torque();

    }

    void update_status() {
        librmcs::device::VescMotor::update_status();
        *angle_ = angle();
        *velocity_ = velocity();
        *current_ = current();
        *temperature_ = temperature();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    double control_velocity() const{
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();

    }    
    double generate_command()  {
    if (first_generate_auto_command_) [[unlikely]] {
         first_generate_auto_command_ = false;
         if(!control_velocity_.ready() && !control_torque_.ready())
         throw std::runtime_error{"[VescMotor] No manipulating available!"};

         else{
           if (!control_velocity_.ready())
               control_velocity_.bind_directly(nan_);
           if (!control_torque_.ready())
               control_torque_.bind_directly(nan_);
    }   
    }
 
        if (!std::isnan(control_velocity()))
            return librmcs::device::VescMotor::velocity_to_rpm(control_velocity());
        else if(!std::isnan(control_torque()))
            return librmcs::device::VescMotor::torque_to_current(control_torque());
        else
            return 0;
    }


private:
static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

rmcs_executor::Component::OutputInterface<double> angle_;
rmcs_executor::Component::OutputInterface<double> velocity_;//
rmcs_executor::Component::OutputInterface<double> current_;
rmcs_executor::Component::OutputInterface<double> temperature_;
rmcs_executor::Component::OutputInterface<double> max_torque_;

rmcs_executor::Component::InputInterface<double> control_torque_;
rmcs_executor::Component::InputInterface<double> control_velocity_;

bool first_generate_auto_command_ = true;
};
}