#ifndef MOTOR_FEEDBACK_HPP_
#define MOTOR_FEEDBACK_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>
#include <motor/sockets.hpp>
#include <motor/motor_params.hpp>
#include<map>

class MotorFeedback {

private:
    std::string motor_name_;
    int motor_id;
    std::shared_ptr<spdlog::logger> logger_;

    bool init_enc;
    uint32_t err_enc;

public:
    MotorFeedback(Sockets::SocketsSPtr motor_sockets_);

    ~MotorFeedback();

    typedef std::shared_ptr<MotorFeedback> MotorFeedbackSPtr;
    typedef std::unique_ptr<MotorFeedback> MotorFeedbackUPtr;

    Sockets::SocketsSPtr motor_sockets;

    typedef struct feedback_ {
        uint16_t status_m;
        float battery_vol_m;
        uint16_t input_states_m;
        float actual_motor_current_m;
        int32_t pos_m; // position loop params
        double vel_m;
        uint32_t manufacturer_reg_m;
        uint32_t latched_fault_m;
        int guard_err_m;
    } feedback_s;


    int err_pdo_1_;
    int err_pdo_2_;
    int err_pdo_3_;
    int err_pdo_4_;

    int motor_status_n_voltage_n_input_states_read(int motor_id, uint16_t *status, float *battery_vol, uint16_t *input_states, float *actual_motor_current, int timeout);

    int motor_enc_read(int motor_id, int32_t *pos, int timeout);

    double motor_cps_to_rpm(double counts_per_sec);

    int motor_vel_read(int motor_id, double *vel, int timeout);

    int motor_system_status_read(int motor_id, uint32_t *manufacturer_reg, uint32_t *latched_fault, int timeout);

    int node_guarding_response_read(uint16_t *response, int timeout);

    uint16_t status_register_fb_[1];
    float battery_vol_fb_[1];
    uint16_t input_states_fb_[1];
    float actual_motor_current_fb_[1];
    int32_t encoder_fb_[1];
    double vel_fb_[1];
    uint32_t manufacturer_reg_fb_[1];
    uint32_t latched_fault_fb_[1];
    uint16_t response_fb_[1];
    int guard_err_fb_;

    int motor_request(void);

    std::map<std::string, int> motorFeedback(int motor_id, MotorFeedback::feedback_s *feedback_s_m);

};

#endif