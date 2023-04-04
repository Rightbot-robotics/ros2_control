//
// Created by amrapali on 1/19/23.
//

#ifndef HARMONIC_MOTOR_ACTUATOR_HPP
#define HARMONIC_MOTOR_ACTUATOR_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <json_reader/json_read.h>
#include <actuator/actuator.hpp>
#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include "harmonic_motor_actuator/harmonic_motor_actuator_sockets.hpp"
#include <harmonic_motor_actuator/harmonic_encoder_sensor.hpp>

#include "hardware_interface/actuator_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_list_macros.hpp"  // NOLINT


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HarmonicMotorActuator : public hardware_interface::ActuatorInterface {

public:

    typedef std::shared_ptr<HarmonicMotorActuator> HarmonicMotorActuatorSPtr;
    typedef std::unique_ptr<HarmonicMotorActuator> HarmonicMotorActuatorUPtr;

    HarmonicMotorActuator();

    ~HarmonicMotorActuator();

    void writeData(Json::Value &actuator_data);

    void sendNodeGuardingRequest();

    void requestData();

    void changeActuatorControlMode(Json::Value &actuator_control_mode);
    
    // ros2 control hardware interface
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    void init_json(std::string path);


private:

    std::string motor_name_;
    int motor_id_;
    int axis_;

    int initMotor();
    int motorConfigNode(int motor_id);
    int motorControlword(uint16_t motor_id, enum Epos_ctrl ctrl);
    int motor_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob);
    int motor_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping* objects);
    int set_tpdo1_cobid(uint16_t node);
    int setTPDO_cobid(uint16_t node_id, uint8_t n);
    int motorSetmode(enum Motor_mode mode);

    int enableMotor();
    int disableMotor(void);
    int haltMotor(void);
    int resetFault(void) ;
    int quickStopMotor(void); 

    int set_profile_velocity(float vel);
    int set_profile_acc(float acc);
    int set_profile_deacc(float deacc);
    int rpm_to_countspersec(float rpm);
    int motor_rps2_to_cps2(float rpss);

    int set_relative_position(int32_t pos);
    void goToInitPos();

    // std::shared_ptr<spdlog::logger> logger_;
    HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr harmonic_motor_actuator_sockets_;
    HarmonicEncoderSensor::HarmonicEncoderSensorSPtr encoder_sensor_;

    Json::Value actuator_data_;
    std::string previous_mode;

    double status_state_ = std::numeric_limits<double>::quiet_NaN();
    double error_code_state_ = std::numeric_limits<double>::quiet_NaN();
    double position_state_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_state_ = std::numeric_limits<double>::quiet_NaN();
    double node_guard_error_state_ = std::numeric_limits<double>::quiet_NaN();
    
    double position_command_ = 0.0;
    double max_velocity_command_ = 0.0;
    double acceleration_command_ = 0.0;

    double previous_position_command_ = -1.0;
    double previous_max_velocity_command_ = -1.0;
    double previous_acceleration_command_ = -1.0;

    double motor_ppr_ = 524288;

    void Homing();
    bool using_default_max_velocity_ = true;
    bool using_default_acceleration_ = true;
    double default_max_velocity_ = 2.0;
    double default_acceleration_ = 1.0;

    bool homing_active_ = false;
    double homing_position_ = 0;
    double homing_max_velocity_ = 2.0;
    double homing_acceleration_ = 1.0;

};


#endif //HARMONIC_MOTOR_ACTUATOR_HPP
