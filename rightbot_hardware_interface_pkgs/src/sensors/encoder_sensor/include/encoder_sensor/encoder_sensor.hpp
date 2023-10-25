#ifndef ENCODER_SENSOR_H_
#define ENCODER_SENSOR_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <json_reader/json_read.h>
#include <sensor/sensor.hpp>
#include <motor/sockets.hpp>
#include <motor/motor_feedback.hpp>
#include <deque>
#include <condition_variable>

class EncoderData {

public:

    EncoderData();

    ~EncoderData() = default;


    uint16_t status_m;
    float battery_vol_m;
    uint16_t input_states_m;
    float actual_motor_current_m;
    int32_t pos_m;
    double vel_m;
    uint32_t manufacturer_reg_m;
    uint32_t latched_fault_m;
    int guard_err_m;
    uint64_t time_sys;
    bool read_status_voltage;
    bool read_status_encoder;
    bool read_status_velocity;
    bool read_status_mf_register;

};

class EncoderSensor : public Sensor {

public:

    typedef std::shared_ptr<EncoderSensor> EncoderSensorSPtr;
    typedef std::unique_ptr<EncoderSensor> EncoderSensorUPtr;

    EncoderSensor();

    ~EncoderSensor();

    void initialize(Json::Value &config_data, Sockets::SocketsSPtr motor_sockets);

    void getData(Json::Value &sensor_data);

    std::shared_ptr<spdlog::logger> logger_;

    std::thread update_data_thread_;
    std::thread read_motor_data_thread_;

    void readMotorData();

    Json::Value sensor_data_;
    Json::Value sensor_config_data_;
    std::mutex read_mutex_;

    void init_json(Json::Value &config_data);

    int motor_id_;
    int axis_;
    std::string motor_name_;
    MotorFeedback::MotorFeedbackSPtr motor_feedback_;
    Sockets::SocketsSPtr motor_sockets_;
    bool message_received;

    void updateData();

    int read_err_;

    int readData(int motor_id, EncoderData *encoder_data);

    MotorFeedback::feedback_s feedback_s_m;
    std::condition_variable cv;

    EncoderData encoder_data_;
    std::deque<EncoderData> q_encoder_data_;
    EncoderData encoder_data_q_element_;

    bool reading_loop_started = false;

    void stop_read_thread();
    bool stop_read_thread_flag = false;

    void enc_clear_can_buffer();
    bool clear_can_buffer_flag = false;

    void readToClearBuffer();

    bool sending_motor_request_internally = false;

};

#endif // ENCODER_SENSOR_H_
