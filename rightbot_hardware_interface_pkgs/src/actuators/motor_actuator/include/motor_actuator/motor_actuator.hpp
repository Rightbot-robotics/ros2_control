#ifndef MOTOR_ACTUATOR_H_
#define MOTOR_ACTUATOR_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <json_reader/json_read.h>
#include <actuator/actuator.hpp>
#include <motor/sockets.hpp>
#include <motor/motor.hpp>
#include <motor/motor_controls.hpp>

#include <socketcan_pkg/printd.h>
#include <socketcan_pkg/socketcan.h>
#include <canopen_pkg/canopen.h>
#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

#include "hardware_interface/actuator_interface.hpp"
#include<encoder_sensor/encoder_sensor.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_list_macros.hpp"  // NOLINT


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MotorActuator : public hardware_interface::ActuatorInterface {

public:

    typedef std::shared_ptr<MotorActuator> MotorActuatorSPtr;
    typedef std::unique_ptr<MotorActuator> MotorActuatorUPtr;

    MotorActuator();

    ~MotorActuator();

    void writeData(Json::Value &actuator_data) ;

    void requestData() ;

    void changeActuatorControlMode(Json::Value &actuator_control_mode) ;

    // ros2 control hardware interface
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    std::string get_name() const override { return "HubMotorActuator"; }
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;


private:

    std::string motor_name_;

    Json::Value actuator_data_;
    Json::Value actuator_config_data_;
    Sockets::SocketsSPtr motor_sockets_;
    Motor::MotorSPtr motor_;
    MotorControls::MotorControlsSPtr motor_controls_;
    EncoderSensor::EncoderSensorSPtr encoder_sensor ;

    std::shared_ptr<spdlog::logger> logger_;
    // bool motorCommand(int motor_id, std::string command_type, MotorControls::position_cmd_t position_cmd_element, MotorControls::velocity_cmd_t velocity_cmd_element);
    int motor_id_;
    int axis_;

    MotorControls::position_cmd_t position_cmd_received_;
    MotorControls::velocity_cmd_t velocity_cmd_received_;

    MotorControls::position_cmd_t
    setPosition(double timeout, double relative_pos, double max_vel, double accel, double decel);

    MotorControls::velocity_cmd_t
    setVelocity(double timeout, double velocity, double max_vel, double accel, double decel);

    bool motorCommand(int motor_id, std::string command_type, MotorControls::position_cmd_t position_cmd_element,
                      MotorControls::velocity_cmd_t velocity_cmd_element);

    void sendNodeGuardingRequest();

    // ros2 control hardware interface
    double status_state_ = std::numeric_limits<double>::quiet_NaN();
    double battery_voltage_state_ = std::numeric_limits<double>::quiet_NaN();
    double input_states_state_ = std::numeric_limits<double>::quiet_NaN();
    double position_state_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_state_ = std::numeric_limits<double>::quiet_NaN();
    double manufacturer_register_state_ = std::numeric_limits<double>::quiet_NaN();
    double latched_fault_state_ = std::numeric_limits<double>::quiet_NaN();
    double node_guard_error_state_ = std::numeric_limits<double>::quiet_NaN();
    
    double position_command_ = 0.0;
    double max_velocity_command_ = 0.0;
    double acceleration_command_ = 0.0;

    double previous_position_command_ = 0.0;
    double previous_max_velocity_command_ = 0.0;
    double previous_acceleration_command_ = 0.0;


};

#endif // MOTOR_ACTUATOR_H_