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

#include "hardware_interface/actuator_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HarmonicEncoderData {

public:

    HarmonicEncoderData();

    ~HarmonicEncoderData() = default;

    uint16_t status_m;
    uint16_t err_code_m;
    int32_t pos_m;
    double vel_m;
    int guard_err_m;
    uint64_t time_sys;
    bool read_status_err_code;
    bool read_status_encoder;
    bool read_status_velocity;

};


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
    CallbackReturn on_init(const hardware_interface::HardwareInfo & /*info*/) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    std::string get_name() const override { return "HarmonicMotorActuator"; }
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/) override;


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

    int set_relative_position(int32_t pos, uint16_t nodeid);
    void goToInitPos();

    std::shared_ptr<spdlog::logger> logger_;
    HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr harmonic_motor_actuator_sockets_;

    Json::Value actuator_data_;
    std::string previous_mode;

    // 
    int motor_status_n_voltage_read(int motor_id, uint16_t *status, uint16_t *err_code, int timeout);
    int motor_enc_read(int motor_id, int32_t *pos, int timeout);
    int motor_vel_read(int motor_id, double *vel, int timeout);
    double motor_cps_to_rpm(double counts_per_sec) ;
    int node_guarding_response_read(uint16_t *response, int timeout);
    int readData(HarmonicEncoderData *encoder_data) ;
    void readMotorData();
    void getData(Json::Value &sensor_data);

    std::thread read_motor_data_thread_;

    bool reading_loop_started = false;
    HarmonicEncoderData encoder_data_;
    std::deque<HarmonicEncoderData> q_encoder_data_;
    HarmonicEncoderData encoder_data_q_element_;
    std::mutex read_mutex_;

    double status_state_ = std::numeric_limits<double>::quiet_NaN();
    double error_code_state_ = std::numeric_limits<double>::quiet_NaN();
    double position_state_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_state_ = std::numeric_limits<double>::quiet_NaN();
    double node_guard_error_state_ = std::numeric_limits<double>::quiet_NaN();
    
    double position_command_ = 0.0;
    double max_velocity_command_ = 0.0;
    double acceleration_command_ = 0.0;
    double deceleration_command_ = 0.0;

    double previous_position_command_ = 0.0;
    double previous_max_velocity_command_ = 0.0;
    double previous_acceleration_command_ = 0.0;
    double previous_deceleration_command_ = 0.0;

};


#endif //HARMONIC_MOTOR_ACTUATOR_HPP
