#include <harmonic_motor_actuator/harmonic_encoder_sensor.hpp>

HarmonicEncoderData::HarmonicEncoderData() {
    status_m = 0;
    err_code_m =0;
    pos_m = 0;
    vel_m = 0.0;
    guard_err_m = 0;
    time_sys = 0;
    read_status_err_code = false;
    read_status_encoder = false;
    read_status_velocity = false;

}

HarmonicEncoderSensor::HarmonicEncoderSensor() {

    

}

HarmonicEncoderSensor::~HarmonicEncoderSensor() {

    read_motor_data_thread_.join();

}

void HarmonicEncoderSensor::initialize(HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr motor_sockets) {
    logger_ = spdlog::get("hardware_interface")->clone("harmonic_encoder_sensor");
    motor_sockets_ = motor_sockets;
    
    motor_id_ = motor_sockets_->motor_id_;
    motor_name_ = motor_sockets_->motor_name_;

    read_err_ = 0;
    reading_loop_started = false;

    

    read_motor_data_thread_ = std::thread(&HarmonicEncoderSensor::readMotorData, this);
}

void HarmonicEncoderSensor::init_json() {

}

int HarmonicEncoderSensor::motor_request(void)
{

    logger_->debug("motor request");
	Socketcan_t data[1];
	data[0].size = 1;
	data[0].data = 0x00;
	return socketcan_write(motor_sockets_->motor_sync_fd, 128, 1, data);
}

int HarmonicEncoderSensor::motor_status_n_voltage_read(int motor_id, uint16_t *status, uint16_t *err_code, float *actual_motor_current, int timeout) {
    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets_->motor_status_pdo_fd, &f, timeout);

    uint16_t actual_motor_current_register_value = 0;

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX1_ID + motor_id)) {
        *status = (f.data[0] << 0) | (f.data[1] << 8);
        *err_code = (f.data[2] << 0) | (f.data[3] << 8);
        // actual_motor_current_register_value = (f.data[4] << 0) | (f.data[5] << 8);
        *actual_motor_current = static_cast<float>(actual_motor_current_register_value)/1000;
        // *battery_vol = ((uint32_t)f.data[4]<<0) | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24);
        // logger_->debug("test battery vol: [{}]", *battery_vol);

    }

    return err;

}

int HarmonicEncoderSensor::motor_enc_read(int motor_id, int32_t *pos, int timeout) {
    int err;
    my_can_frame f;
    uint32_t enc;

    err = PDO_read(motor_sockets_->motor_enc_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX3_ID + motor_id)) {
        //ENCODER COUNT
        enc = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
              ((uint32_t) f.data[3] << 24);
        //rpm = ((uint32_t)f.data[4]<<0) | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24);
        // logger_->debug("Harmonic motor counts init: [{}]", enc);
        position_demand_value =  ((uint32_t) f.data[4] << 0) | ((uint32_t) f.data[5] << 8) | ((uint32_t) f.data[6] << 16) |
              ((uint32_t) f.data[7] << 24);
        
        if (init_enc) {
            *pos = enc - err_enc;
        } else {
            err_enc = enc;
            init_enc = true;
        }

        if(prev_position_demand_value != position_demand_value){
            // logger_->info("[{}] new position demand value {}, current_count {}",motor_name_ , position_demand_value, *pos);
            prev_position_demand_value = position_demand_value;
        }

        //*vel = rpm*0.1;//motor_rpm_to_mmsec(-rpm);
    }

    return err;
}

int HarmonicEncoderSensor::motor_vel_read(int motor_id, double *vel, int timeout) {
    int err;
    my_can_frame f;
    int32_t rpm;
    int32_t register_cps;
    double cps;

    err = PDO_read(motor_sockets_->motor_vel_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX2_ID + motor_id)) {
        //RPM OF LEFT
        register_cps = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                       ((uint32_t) f.data[3] << 24);
        cps = register_cps;
        *vel = (double) motor_cps_to_rpm(cps);
    }

    return err;
}

double HarmonicEncoderSensor::motor_cps_to_rpm(double counts_per_sec) {

    double m_per_sec = (counts_per_sec) / EROB_CPR;
    return m_per_sec * 60;
}

int HarmonicEncoderSensor::node_guarding_response_read(uint16_t *response, int timeout){
    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets_->nmt_motor_cfg_fd, &f, timeout);

    if(err != 0) {
        // loop executed when read error, or no data
        return err;
    }

    if(f.data[0] <= 0){
        return -1;
    }

    if (f.id == (NMT_TX + motor_id_)) {
        *response = ((uint16_t)f.data[0]);
    }

    return 0;
}


int HarmonicEncoderSensor::readData(HarmonicEncoderData *encoder_data) {

    uint16_t status_register_fb_[1]= {0};
    uint16_t err_code_fb_[1] = {0};
    float actual_motor_current_fb_[1] = {0};
    int32_t encoder_fb_[1]= {0};
    double vel_fb_[1]= {0};
    int guard_err_fb_= -1;


    auto err_pdo_1_ = motor_status_n_voltage_read(motor_id_, status_register_fb_, err_code_fb_, actual_motor_current_fb_, 1);
    auto err_pdo_2_ = motor_enc_read(motor_id_, encoder_fb_, 1);
    auto err_pdo_3_ = motor_vel_read(motor_id_, vel_fb_, 1);
    
    guard_err_fb_ = err_pdo_2_;

    if (0 == err_pdo_1_) {

        encoder_data->status_m = status_register_fb_[0];
        encoder_data->err_code_m = err_code_fb_[0];
        encoder_data->actual_motor_current_m = actual_motor_current_fb_[0];
        encoder_data->read_status_err_code = true;

    }
    else{
        encoder_data->read_status_err_code = false;
    }

    if (0 == err_pdo_2_) {
        encoder_data->pos_m = encoder_fb_[0];
        // logger_->debug("Encoder_position: [{}]", encoder_fb_[0]);
        encoder_data->read_status_encoder = true;
        // logger_->debug("[{}] - enc read success",motor_name_);
    }
    else{
        encoder_data->read_status_encoder = false;
    }

    if (0 == err_pdo_3_) {
        encoder_data->vel_m = vel_fb_[0];
        // logger_->debug("Encoder_Velocity: [{}]", vel_fb_[0]);
        encoder_data->read_status_velocity = true;
        // logger_->debug("[{}] - vel read success",motor_name_);
    }else {
        encoder_data->read_status_velocity = false;
        logger_->debug("[{}] - vel read false",motor_name_);
    }

    encoder_data->guard_err_m = guard_err_fb_;

    auto now = std::chrono::high_resolution_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    long duration = value.count();
    encoder_data->time_sys = duration;

    bool status = encoder_data->read_status_encoder;
    int rt_value = -1;
    if(true == status) {
        rt_value = 0;
	logger_->debug("{} pos read successful",motor_sockets_->motor_name_);
    }
    return 0;

}

void HarmonicEncoderSensor::stop_read_thread() {
    stop_read_thread_flag = true;
    
}


void HarmonicEncoderSensor::readMotorData() {


    while (!stop_read_thread_flag) {

        auto start_time = std::chrono::system_clock::now();

        {
            
            if(reading_loop_started) {

                if(motor_name_ == "base_rotation_joint"){
                    motor_request();
                    
                }
                if(motor_name_ == "elbow_rotation_joint"){
                    motor_request();
                    
                }
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
                
                int err = readData( &encoder_data_);

                if (err == 0) {

                    read_mutex_.lock();
                    q_encoder_data_.push_back(encoder_data_);
                    read_mutex_.unlock();
                }
                else {
                    logger_->warn("incomplete data received, not pushing to sensor data q");
                }
            }
            

        }

        auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now() - start_time);
        // logger_->debug("Time in execution [ readMotorData() ]: [{}] us", time_passed_in_read.count());
        logger_->debug("Actuator [{}] Time in execution [ readMotorData() ]: [{}] us",motor_name_ , time_passed_in_read.count());


        std::this_thread::sleep_for(std::chrono::microseconds(20000 - time_passed_in_read.count()));

    }

}

void HarmonicEncoderSensor::getData(Json::Value &sensor_data) {

    read_mutex_.lock();
    reading_loop_started = true;

    if (!q_encoder_data_.empty()) {


        auto encoder_data_q_element = q_encoder_data_.back();
        q_encoder_data_.pop_front();

        // logger_->debug("Read deque size after pop: {}", q_encoder_data_.size());
        if (q_encoder_data_.size() > 10) {
            logger_->debug("Actuator [{}] Read deque size : [{}]", motor_sockets_->motor_name_, q_encoder_data_.size());
            // std::cout << "Read deque size: "<< q_encoder_data_.size() << std::endl;
            q_encoder_data_.clear();
        }

        sensor_data["status"] = encoder_data_q_element.status_m;
        sensor_data["err_code"] = encoder_data_q_element.err_code_m;
        sensor_data["actual_motor_current"] = encoder_data_q_element.actual_motor_current_m;
        sensor_data["counts"] = encoder_data_q_element.pos_m;
        sensor_data["velocity"] = encoder_data_q_element.vel_m;
        sensor_data["timestamp"] = to_string(encoder_data_q_element.time_sys);
        sensor_data["guard_err"] = encoder_data_q_element.guard_err_m;
        sensor_data["read_status"] = true;
        sensor_data["read_status_err_code"] = encoder_data_q_element.read_status_err_code;
        sensor_data["read_status_encoder"] = encoder_data_q_element.read_status_encoder;
        sensor_data["read_status_velocity"] = encoder_data_q_element.read_status_velocity;
        
        // logger_->debug("[{}] Status: [{}], Error Code: [{}]", motor_sockets_->motor_name_, encoder_data_q_element.status_m, encoder_data_q_element.err_code_m);
        // logger_->debug("[{}] Position: {} counts, Velocity: {} rpm", motor_sockets_->motor_name_, encoder_data_q_element.pos_m, encoder_data_q_element.vel_m);
        // logger_->debug("[{}] Guard Err: {}", motor_sockets_->motor_name_, encoder_data_q_element.guard_err_m);

    } else {
        sensor_data["read_status"] = false;
        logger_->debug("[{}] Sensor Data Queue Empty. ", motor_sockets_->motor_name_);
    }


    read_mutex_.unlock();
}
