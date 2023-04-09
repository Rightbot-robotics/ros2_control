#ifndef HARMONIC_ENCODER_SENSOR_H_
#define HARMONIC_ENCODER_SENSOR_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <json_reader/json_read.h>
#include <sensor/sensor.hpp>
#include "harmonic_motor_actuator/harmonic_motor_actuator_sockets.hpp"
#include <deque>
#include <condition_variable>

using namespace std;

class HarmonicEncoderData {

public:

    HarmonicEncoderData();

    ~HarmonicEncoderData() = default;

    uint16_t status_m;
    uint16_t err_code_m;
    float actual_motor_current_m;
    int32_t pos_m;
    double vel_m;
    int guard_err_m;
    uint64_t time_sys;
    bool read_status_err_code;
    bool read_status_encoder;
    bool read_status_velocity;

};

class HarmonicEncoderSensor : public Sensor {

public:

    typedef std::shared_ptr<HarmonicEncoderSensor> HarmonicEncoderSensorSPtr;
    typedef std::unique_ptr<HarmonicEncoderSensor> HarmonicEncoderSensorUPtr;

    HarmonicEncoderSensor();

    ~HarmonicEncoderSensor();

    void getData(Json::Value &sensor_data);

    void initialize(HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr motor_sockets);

    int motor_request(void);

    int motor_status_n_voltage_read(int motor_id, uint16_t *status, uint16_t *err_code, float *actual_motor_current, int timeout);
    int motor_enc_read(int motor_id, int32_t *pos, int timeout);
    int motor_vel_read(int motor_id, double *vel, int timeout);
    int node_guarding_response_read(uint16_t *response, int timeout);

    double motor_cps_to_rpm(double counts_per_sec);

    // std::shared_ptr<spdlog::logger> logger_;
    HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr motor_sockets_;

    std::string motor_name_;
    int motor_id_;

    std::thread read_motor_data_thread_;

    void readMotorData();

    Json::Value sensor_data_;
    Json::Value sensor_config_data_;
    std::mutex read_mutex_;

    void init_json();
   
    void updateData();

    int read_err_;

    int readData(HarmonicEncoderData *encoder_data);

    std::condition_variable cv;

    HarmonicEncoderData encoder_data_;
    std::deque<HarmonicEncoderData> q_encoder_data_;
    HarmonicEncoderData encoder_data_q_element_;

    bool reading_loop_started;

    bool init_enc = false;
    int err_enc = 0;

    
};

#endif // HARMONIC_ENCODER_SENSOR_H_