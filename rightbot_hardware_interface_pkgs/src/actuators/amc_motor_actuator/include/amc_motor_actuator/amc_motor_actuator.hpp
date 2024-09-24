#ifndef AMC_MOTOR_ACTUATOR_HPP
#define AMC_MOTOR_ACTUATOR_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <json_reader/json_read.h>
#include <actuator/actuator.hpp>
#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <sstream>

#include "amc_motor_actuator/amc_motor_actuator_sockets.hpp"
#include <amc_motor_actuator/amc_motor_actuator_feedback.hpp>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include <hardware_interface/actuator_interface.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_list_macros.hpp"  // NOLINT


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AmcMotorActuator : public hardware_interface::ActuatorInterface {

public:

    typedef std::shared_ptr<AmcMotorActuator> AmcMotorActuatorSPtr;
    typedef std::unique_ptr<AmcMotorActuator> AmcMotorActuatorUPtr;

    AmcMotorActuator();

    ~AmcMotorActuator();

    // void writeData(Json::Value &actuator_data);

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
    void fault_reset() override;
    void reinitialize_actuator() override;
    // void clear_can_buffer() override;

    void homing_execution(double &homing_pos) override;

    void data_request() override;

    void node_guarding_request() override;

    int set_vel_speed(uint16_t nodeid, int axis, float vel);

    int enable_motion_profile();

    union 
    {
        int64_t int_val;
        uint8_t byte_val[8];
    } int64_to_bytes;

private:

    std::string motor_name_;
    int motor_id_;
    int axis_;
    int zero_point_count_;

    int initMotor();
    int motorConfigNode(int motor_id);
    int motorControlword(uint16_t motor_id, enum Epos_ctrl ctrl);
    int motor_Transmit_PDO_n_Parameter(uint16_t node_id, uint16_t index);
    int amc_motor_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t num_objects, Epos_pdo_mapping* objects);
    int amc_motor_Receive_PDO_n_Parameter(uint16_t node_id, uint16_t index);
    int setTPDO_cobid(uint16_t node_id, uint16_t index, uint8_t n);
    int set_vel_RPDO_cobid(uint16_t node_id, uint16_t index);
    int motorSetmode(enum Motor_mode mode);

    int enableMotor();
    int disableMotor(void);
    int haltMotor(void);
    int resetFault(void) ;
    int quickStopMotor(void); 
    int reinitializeMotor(void);
    int amc_drive_reset(void);

    int set_target_velocity(float vel);

    int set_profile_velocity(float vel);
    int set_profile_acc(float acc);
    int set_profile_deacc(float deacc);
    int rpm_to_countspersec(float rpm);
    int motor_rps2_to_cps2(float rpss);
    int set_PTPC(float acc);        
    int set_PTNC(float deacc);    
    int set_NTNC(float acc);    
    int set_NTPC(float deacc);    
    int enable_brake(bool is_enabled);
    int set_guard_time(uint16_t motor_id, uint16_t value);
    int set_life_time_factor(uint16_t motor_id, uint8_t value);

    int set_position_kp(float kp);
    int set_position_ki(float ki);
    int set_position_kd(float kd);

    int set_velocity_kp(float kp);
    int set_velocity_ki(float ki);
    int set_velocity_kd(float kd);

    int store_params_to_drive();
    int set_relative_position(int32_t pos);
    void goToInitPos();
    
    std::shared_ptr<spdlog::logger> logger_;
    AmcMotorActuatorSockets::AmcMotorActuatorSocketsSPtr amc_motor_actuator_sockets_;
    AmcEncoderSensor::AmcEncoderSensorSPtr encoder_sensor_;

    Json::Value actuator_data_;
    std::string previous_mode;

    Json::Value sensor_data;

    double status_state_ = std::numeric_limits<double>::quiet_NaN();
    double error_code_state_ = std::numeric_limits<double>::quiet_NaN();
    double actual_motor_current_state_ = std::numeric_limits<double>::quiet_NaN();
    double position_state_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_state_ = std::numeric_limits<double>::quiet_NaN();
    double node_guard_error_state_ = std::numeric_limits<double>::quiet_NaN();
    double amc_drive_system_status_1_ = std::numeric_limits<double>::quiet_NaN();
    double amc_drive_system_status_2_ = std::numeric_limits<double>::quiet_NaN();
    double amc_drive_protection_status_ = std::numeric_limits<double>::quiet_NaN();
    double amc_system_protection_status_ = std::numeric_limits<double>::quiet_NaN();
    double functional_mode_state_ = std::numeric_limits<double>::quiet_NaN();

    enum ActuatorFunctionalState {
        OPERATIONAL = 0,
        SOFT_STOP = 1,
        HARD_STOP = 2,
        FAULT = 3
    };
    ActuatorFunctionalState curr_state_ = ActuatorFunctionalState::OPERATIONAL;
    ActuatorFunctionalState target_state_ = ActuatorFunctionalState::OPERATIONAL;
    ActuatorFunctionalState commanded_state_;

    double position_kp_value_ = 0.0;
    double position_ki_value_ = 0.0;
    double position_kd_value_ = 0.0;
    double velocity_kp_value_ = 0.0;
    double velocity_ki_value_ = 0.0;
    double velocity_kd_value_ = 0.0;
    
    double position_command_ = 0.0;
    double max_velocity_command_ = 0.0;
    double acceleration_command_ = std::numeric_limits<double>::quiet_NaN();
    double control_state_command_ = 0.0;
    double position_kp_command_ = std::numeric_limits<double>::quiet_NaN();
    double position_ki_command_ = std::numeric_limits<double>::quiet_NaN();
    double position_kd_command_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_kp_command_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_ki_command_ = std::numeric_limits<double>::quiet_NaN();
    double velocity_kd_command_ = std::numeric_limits<double>::quiet_NaN();    
    double previous_position_command_ = 0.0;
    double previous_max_velocity_command_ = 0.0;
    double previous_acceleration_command_ = 0.0;
    double previous_control_state_command_ = 0.0;
    double functional_mode_command_ = std::numeric_limits<double>::quiet_NaN();

    bool using_default_acceleration_ = true;
    double default_max_velocity_ = 2.0;
    double default_acceleration_ = 1.0;
    double default_deceleration_ = 1.0;
    std::string mode_of_operation_;
    std::string previous_mode_of_operation_ = "velocity";

    int is_homing_ = 0;
    double homing_max_velocity_ = 2.0;
    double homing_acceleration_ = 1.0;
    double homing_deceleration_ = 1.0;
    double counts_offset_ = 0.0;
    double min_position_ = 0.0;
    double max_position_ = 0.0;
    int is_homing_at_min_ = 0; 
    double travel_per_revolution_ = 1.0;
    double homing_position_ = 1.0;
    double motor_gear_ratio_ = 1.0;
    double motor_ppr_ = 1.0;

    double acceleration_epsilon = 10e-4;
    double velocity_epsilon = 10e-5;

    enum Control_mode {
	ACTUATOR_ENABLE = 0, //Contour speed mode=3 - mode selection
	ACTUATOR_DISABLE = 1, //Contour position mode=1 - mode selection
	ACTUATOR_QUICK_STOP = 2
    };

    bool Homing();

    double radianToDegree(double rad);
    double degreeToRadian(double deg);

};


#endif //AMC_MOTOR_ACTUATOR_HPP