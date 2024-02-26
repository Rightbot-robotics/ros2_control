//
// Created by amrapali on 11/30/22.
//

#ifndef ABSOLUTE_ENCODER_HPP
#define ABSOLUTE_ENCODER_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <fstream>

#include <sensor/sensor.hpp>
#include <json_reader/json_read.h>

#include "absolute_encoder/absolute_encoder_sockets.hpp"

#include <hardware_interface/sensor_interface.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "pluginlib/class_list_macros.hpp"  // NOLINT

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AbsoluteEncoderSensor : public hardware_interface::SensorInterface {

public:

    typedef std::shared_ptr <AbsoluteEncoderSensor> AbsoluteEncoderSensorSPtr;
    typedef std::unique_ptr <AbsoluteEncoderSensor> AbsoluteEncoderSensorUPtr;

    AbsoluteEncoderSensor();

    ~AbsoluteEncoderSensor();

    void getData(Json::Value &sensor_data);

    int requestData();
    void data_request();

    // 
    int readEncCounts(float* angle);

    // ros2 control hardware interface
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    void clear_can_buffer() override;


private:

    int sensor_id_;
    int axis_;
    std::string sensor_name_;

    Json::Value sensor_data_;

    int initSensor();
    int enableSensor();
    float convertToAngle(int counts);
    void readData();
    void updateCountsFile();

    typedef struct {
        uint16_t index;
        uint8_t subindex;
        uint8_t length;
    } Epos_pdo_mapping;

    typedef struct feedback_ {
        float angle;
        uint64_t time_sys;
        bool read_status;

    } feedback_s;

    int encoder_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob);
    int encoder_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping *objects);


    std::shared_ptr<spdlog::logger> logger_;
    AbsoluteEncoderSockets::AbsoluteEncoderSocketsSPtr absolute_encoder_sockets_;

    std::deque<feedback_s> q_angle_data_;
    std::mutex read_mutex_;
    std::thread read_enc_data_thread_;

    std::thread count_file_update_thread_;
    std::mutex count_file_update_mutex_;
    std::condition_variable count_file_update_cv_;
    bool update_file_ = false;
    std::string counts_file_name_;

    
    int absolute_encoder_init_pos;
    int abs_motor_ppr;
    int prev_counts;
    int curr_counts;
    int counts_diff;
    int num_rotations;
    int multi_turn_counts;

    bool reading_loop_started = false;

    double position_state_ = std::numeric_limits<double>::quiet_NaN();
    

};

#endif //ABSOLUTE_ENCODER_HPP
