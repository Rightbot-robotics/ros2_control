#include <motor/motor_feedback.hpp>


MotorFeedback::MotorFeedback(Sockets::SocketsSPtr motor_sockets_) {

    logger_ = spdlog::get("hardware_interface")->clone("motor_feedback");

    motor_sockets = motor_sockets_;
    motor_name_ = motor_sockets->motor_name_;
    init_enc = false;
    err_enc = 0;
    err_pdo_1_ = 0;
    err_pdo_2_ = 0;
    err_pdo_3_ = 0;
    err_pdo_4_ = 0;
    guard_err_fb_ = 0;

    status_register_fb_[1] = {0};
    battery_vol_fb_[1] = {0};
    input_states_fb_[1] = {0};
    actual_motor_current_fb_[1] = {0};
    encoder_fb_[1] = {0};
    vel_fb_[1] = {0};
    manufacturer_reg_fb_[1] = {0};
    latched_fault_fb_[1] = {0};
    response_fb_[1] = {0};


}

MotorFeedback::~MotorFeedback() {

}

int MotorFeedback::motor_request(void) {

    Socketcan_t data[1];
    data[0].size = 1;
    data[0].data = 0x00;
    return socketcan_write(motor_sockets->motor_sync_fd, 128, 1, data);
}

double MotorFeedback::motor_cps_to_rpm(double counts_per_sec) {

    double m_per_sec = (counts_per_sec) * DRIVING_MOTOR_COUNTS_PER_SEC_TO_RPM;
    return m_per_sec;
}

int MotorFeedback::motor_status_n_voltage_n_input_states_read(int motor_id, uint16_t *status, float *battery_vol, uint16_t *input_states,  float *actual_motor_current, int timeout){
    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets->motor_status_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    int actual_motor_current_register_value = 0;

    if (f.id == (PDO_TX1_ID + motor_id)) {
        *status = (f.data[0] << 0) | (f.data[1] << 8);

        *battery_vol = (f.data[2] << 0) | (f.data[3] << 8);
        *battery_vol = 0.1 * (*battery_vol);

        *input_states = (f.data[4] << 0) | (f.data[5] << 8);

        actual_motor_current_register_value = (f.data[6] << 0) | (f.data[7] << 8);
        *actual_motor_current = 0.01 * static_cast<float>(actual_motor_current_register_value);
    }

    return err;

}

int MotorFeedback::motor_enc_read(int motor_id, int32_t *pos, int timeout) {
    int err;
    my_can_frame f;
    uint32_t enc;

    // auto start_time = std::chrono::system_clock::now();
    // err = PDO_read(motor_sockets->motor_enc_pdo_fd, &f, timeout);
    // auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::system_clock::now() - start_time);
    // // logger_->debug("Time in execution [ motor_enc_read() ]: [{}] us, err: [{}]", time_passed_in_read.count(), err);

    // // err = PDO_read(motor_sockets->motor_enc_pdo_fd, &f, timeout);

    // if (err != 0) {
    //     // Read error, or no data
    //     return err;
    // }

    // if (f.id == (PDO_TX3_ID + motor_id)) {
    //     //ENCODER COUNT
    //     enc = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
    //           ((uint32_t) f.data[3] << 24);
    //     //rpm = ((uint32_t)f.data[4]<<0) | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24);
    //     // if (init_enc) {
    //     //     *pos = -enc - err_enc;
    //     // } else {
    //     //     err_enc = -enc;
    //     //     init_enc = true;
    //     // }
    //     *pos = enc; 
    //     //*vel = rpm*0.1;//motor_rpm_to_mmsec(-rpm);
    // }

    
    int iteration = 0;

    while(true){

        err = PDO_read(motor_sockets->motor_enc_pdo_fd, &f, timeout);

        if (err != 0) {
            // Read error, or no data
            break;
        }

        if (f.id == (PDO_TX3_ID + motor_id)) {
            //ENCODER COUNT
            enc = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                ((uint32_t) f.data[3] << 24);
            
            *pos = enc;
        }
        iteration+1;

        logger_->debug("[{}] pos read iteration [{}]", motor_name_, iteration);

    }

    return err;
}

int MotorFeedback::motor_vel_read(int motor_id, double *vel, int timeout) {
    int err;
    my_can_frame f;
    int32_t rpm;
    int32_t register_cps;
    double cps;

    // err = PDO_read(motor_sockets->motor_vel_pdo_fd, &f, timeout);

    // if (err != 0) {
    //     // Read error, or no data
    //     return err;
    // }

    // if (f.id == (PDO_TX2_ID + motor_id)) {
    //     //RPM OF LEFT
    //     register_cps = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
    //                    ((uint32_t) f.data[3] << 24);
    //     cps = register_cps;
    //     *vel = (double) motor_cps_to_rpm(cps);
    // }

    int iteration = 0;

    while(true){

        err = PDO_read(motor_sockets->motor_vel_pdo_fd, &f, timeout);

        if (err != 0) {
            // Read error, or no data
            break;
        }

        if (f.id == (PDO_TX2_ID + motor_id)) {
            //RPM OF LEFT
            register_cps = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                        ((uint32_t) f.data[3] << 24);
            cps = register_cps;
            *vel = (double) motor_cps_to_rpm(cps);
        }
        iteration+1;

        logger_->debug("[{}] vel read iteration [{}]", motor_name_, iteration);

    }

    return err;
}

int MotorFeedback::motor_system_status_read(int motor_id, uint32_t *manufacturer_reg, uint32_t *latched_fault,
                                            int timeout) {

    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets->motor_system_status_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX4_ID + motor_id)) {
        //STATUS OF LEFT MOTOR
        *manufacturer_reg = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                            ((uint32_t) f.data[3] << 24);
        *latched_fault = ((uint32_t) f.data[4] << 0) | ((uint32_t) f.data[5] << 8) | ((uint32_t) f.data[6] << 16) |
                         ((uint32_t) f.data[7] << 24);
    }

    return err;
}

int MotorFeedback::node_guarding_response_read(uint16_t *response, int timeout){
    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets->nmt_motor_cfg_fd, &f, timeout);

    if(err != 0) {
        // loop executed when read error, or no data
        return err;
    }

    if(f.data[0] <= 0){
        return -1;
    }

    if (f.id == (NMT_TX + motor_sockets->motor_can_id)) {
        *response = ((uint16_t)f.data[0]);
    }

    return 0;
}



std::map<std::string, int> MotorFeedback::motorFeedback(int motor_id, MotorFeedback::feedback_s *feedback_s_m) {

    std::map<std::string, int> read_error_code;

    err_pdo_1_ = motor_status_n_voltage_n_input_states_read(motor_id, status_register_fb_, battery_vol_fb_, input_states_fb_, actual_motor_current_fb_, 1);
    read_error_code.insert_or_assign("voltage", err_pdo_1_);

    err_pdo_2_ = motor_enc_read(motor_id, encoder_fb_, 1);
    read_error_code.insert_or_assign("encoder", err_pdo_2_);

    err_pdo_3_ = motor_vel_read(motor_id, vel_fb_, 1);
    read_error_code.insert_or_assign("velocity", err_pdo_3_);

    err_pdo_4_ = motor_system_status_read(motor_id, manufacturer_reg_fb_, latched_fault_fb_, 1);
    read_error_code.insert_or_assign("mf_register", err_pdo_4_);

    // guard_err_fb_ = node_guarding_response_read(response_fb_, 1);
    read_error_code.insert_or_assign("guard_err", guard_err_fb_);

    if (err_pdo_1_ == 0) {
        feedback_s_m->status_m = status_register_fb_[0];
        feedback_s_m->battery_vol_m = battery_vol_fb_[0];
        feedback_s_m->input_states_m = input_states_fb_[0];
        feedback_s_m->actual_motor_current_m = actual_motor_current_fb_[0];

    } else {
//        feedback_s_m->status_m = -1;
//        feedback_s_m->battery_vol_m = -1;
    }

    if (err_pdo_2_ == 0) {
        feedback_s_m->pos_m = encoder_fb_[0];
    } else {
//        feedback_s_m->pos_m = -1;
    }

    if (err_pdo_3_ == 0) {
        feedback_s_m->vel_m = vel_fb_[0];
    } else {
//        feedback_s_m->vel_m = -1;
    }

    if (err_pdo_4_ == 0) {
        feedback_s_m->manufacturer_reg_m = manufacturer_reg_fb_[0];
        feedback_s_m->latched_fault_m = latched_fault_fb_[0];
    } else {
//        feedback_s_m->manufacturer_reg_m = -1;
//        feedback_s_m->latched_fault_m = -1;
    }

    // read status of node guard error
    feedback_s_m->guard_err_m = guard_err_fb_;

    return read_error_code;
}