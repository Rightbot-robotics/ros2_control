#ifndef AMC_MOTOR_ACTUATOR_FEEDBACK_H_
#define AMC_MOTOR_ACTUATOR_FEEDBACK_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <json_reader/json_read.h>
#include <sensor/sensor.hpp>
#include <amc_motor_actuator/amc_motor_actuator_sockets.hpp>
#include <deque>
#include <condition_variable>

using namespace std;

class AmcEncoderData {

public:

    AmcEncoderData();

    ~AmcEncoderData() = default;

    uint16_t status_m;
    uint16_t err_code_m;
    float actual_motor_current_m;
    int32_t pos_m;
    double vel_m;
    int drive_stat_m;
    int system_stat_m;
    int voltage_m;
    int io_stat_m;
    int guard_err_m;
};

class AmcEncoderSensor : public Sensor {

public:

    typedef std::shared_ptr<AmcEncoderSensor> AmcEncoderSensorSPtr;
    typedef std::unique_ptr<AmcEncoderSensor> AmcEncoderSensorUPtr;

    AmcEncoderSensor();

    ~AmcEncoderSensor();

    void getData(Json::Value &sensor_data);

    void initialize(AmcMotorActuatorSockets::AmcMotorActuatorSocketsSPtr motor_sockets);

    int motor_request(void);

    int motor_status_n_pos_read(int motor_id, uint16_t *status, float *actual_position, int timeout);
    int motor_vel_read(int motor_id, double *vel, int timeout);
    int motor_current_read(int motor_id, int16_t *actual_motor_current, int timeout);
    int motor_stat_voltage_n_io_read(int motor_id, int16_t *drive_stat, int16_t *system_stat, int16_t *voltage, int16_t *io_stat, int timeout);
    int node_guarding_response_read(uint16_t *response, int timeout);

    double motor_cps_to_rpm(double counts_per_sec);

    uint16_t read_ki_constant();
    uint32_t read_ks_constant();
    uint16_t read_kds_constant();
    uint16_t read_kp_constant();
    uint16_t read_kov_constant();
    
    std::shared_ptr<spdlog::logger> logger_;
    AmcMotorActuatorSockets::AmcMotorActuatorSocketsSPtr motor_sockets_;

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

    int readData(AmcEncoderData *encoder_data);

    std::condition_variable cv;

    AmcEncoderData encoder_data_;
    std::deque<AmcEncoderData> q_encoder_data_;
    AmcEncoderData encoder_data_q_element_;

    bool reading_loop_started = false;

    bool init_enc = false;
    int err_enc = 0;
    int position_demand_value = 0;
    int prev_position_demand_value = 0;

    void stop_read_thread();
    bool stop_read_thread_flag = false;

    void readToClearBuffer();
    bool sending_motor_request_internally = false;

    uint16_t status_register_fb_[1]= {0};
    uint16_t err_code_fb_[1] = {0};
    int16_t actual_motor_current_fb_[1] = {0};
    float actual_position_fb_[1] = {0};
    int32_t encoder_fb_[1]= {0};
    double vel_fb_[1]= {0};
    int16_t drive_stat_fb_[1] = {0};
    int16_t system_stat_fb_[1] = {0};
    int16_t voltage_fb_[1] = {0};
    int16_t io_stat_fb_[1] = {0};
    
    int guard_err_fb_ = -1;

    float motor_rated_current_;

    uint16_t ki = 0;
    uint32_t ks = 0;
    uint16_t kp = 0;
    uint16_t kov = 0;
};

#endif // AMC_MOTOR_ACTUATOR_FEEDBACK_H_