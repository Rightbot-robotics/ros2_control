#include <amc_motor_actuator/amc_motor_actuator_feedback.hpp>

AmcEncoderData::AmcEncoderData() {
    status_m = 0;
    err_code_m =0;
    pos_m = 0;
    vel_m = 0.0;
    guard_err_m = 0;
}

AmcEncoderSensor::AmcEncoderSensor() {
}

AmcEncoderSensor::~AmcEncoderSensor() {

    read_motor_data_thread_.join();

}

void AmcEncoderSensor::initialize(AmcMotorActuatorSockets::AmcMotorActuatorSocketsSPtr motor_sockets) {
    logger_ = spdlog::get("hardware_interface")->clone("amc_encoder_sensor");
    motor_sockets_ = motor_sockets;
    
    motor_id_ = motor_sockets_->motor_id_;
    motor_name_ = motor_sockets_->motor_name_;

    read_err_ = 0;
    reading_loop_started = false;

    ki = read_ki_constant();
	ks = read_ks_constant();
    kp = read_kp_constant();
    kov = read_kov_constant();
    kms = read_kms_constant();

    // int err;
    // SDO_data req, resp;
	// req.nodeid = motor_id_;
	// req.index = 0x6075;
	// req.subindex = 0x00;
	// req.data = {0, 0x00};
	
	// err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    // if(err != 0) {
    //     logger_->info("[{}] Motor rated current was not read...", motor_name_);
    // }

    // motor_rated_current_ = ((float)resp.data.data) / 1000;  // mA to A

	// logger_->info("[{}] Motor rated current: {} A", motor_name_, motor_rated_current_);

    read_motor_data_thread_ = std::thread(&AmcEncoderSensor::readMotorData, this);
}

int64_t AmcEncoderSensor::read_kms_constant() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x203C;
	req.subindex = 0x09;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] Kms constant was not read...", motor_name_);
		return 1;
	}

	int64_t kms = (static_cast<int64_t>(resp.data.data));
    
	logger_->info("[{}] Kms constant was read...", kms);

	return kms;
}

uint16_t AmcEncoderSensor::read_ki_constant() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x2032;
	req.subindex = 0x08;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] Ki constant was not read...", motor_name_);
		return 1;
	}

	uint16_t ki = (static_cast<double>(resp.data.data));

	if(ki == 0) {
		return 1;
	}

	logger_->info("[{}] Ki constant was read...", ki);

	return ki;
}

uint32_t AmcEncoderSensor::read_ks_constant() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x20D8;
	req.subindex = 0x24;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] Ks constant was not read...", motor_name_);
		return 1;
	}
	
	uint32_t ks = (static_cast<double>(resp.data.data))/65.536;
	
	return ks;
}

uint16_t AmcEncoderSensor::read_kp_constant() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x20D8;
	req.subindex = 0x0C;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] Kp constant was not read...", motor_name_);
		return 1;
	}
	
	uint16_t kp = (static_cast<double>(resp.data.data))/10;
	
	return kp;
}

uint16_t AmcEncoderSensor::read_kov_constant() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x20D8;
	req.subindex = 0x09;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] Kp constant was not read...", motor_name_);
		return 1;
	}
	
	uint16_t kov = (static_cast<double>(resp.data.data))/10;
	
	return kov;
}

uint16_t AmcEncoderSensor::read_drive_status_1() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x2002;
	req.subindex = 0x05;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] drive status 1 was not read...", motor_name_);
		return 1;
	}
	
	uint16_t drive_status_1 = (static_cast<double>(resp.data.data));
	
	return drive_status_1;
}

uint16_t AmcEncoderSensor::read_drive_status_2() {
	int err;
    SDO_data req, resp;
	req.nodeid = motor_id_;
	req.index = 0x2002;
	req.subindex = 0x04;
	req.data = {0, 0x00};
	
	err = SDO_read(motor_sockets_->motor_cfg_fd, &req, &resp);
    if(err != 0) {
        logger_->info("[{}] drive status 2 was not read...", motor_name_);
		return 1;
	}
	
	uint16_t drive_status_2 = (static_cast<double>(resp.data.data));
	
	return drive_status_2;
}

void AmcEncoderSensor::init_json() {

}

int AmcEncoderSensor::motor_request(void)
{

    // logger_->debug("motor request");
	Socketcan_t data[1];
	data[0].size = 1;
	data[0].data = 0x00;
	return socketcan_write(motor_sockets_->motor_sync_fd, 128, 1, data);
}

int AmcEncoderSensor::motor_status_n_pos_read(int motor_id, uint16_t *status, float *actual_position, int timeout) {
    int err;
    my_can_frame f;
    err = PDO_read(motor_sockets_->motor_status_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX1_ID + motor_id)) {
        *status = (f.data[0] << 0) | (f.data[1] << 8);
        int32_t actual_position_int = ((f.data[2] << 0) | (f.data[3] << 8) | (f.data[4] << 16) | (f.data[5] << 24));
        *actual_position = ((float)actual_position_int);
        logger_->debug("[{}] Motor position: [{}]", motor_name_, *actual_position);
    }
    return err;
}

int AmcEncoderSensor::motor_vel_read(int motor_id, double *vel, int timeout) {
    int err;
    my_can_frame f;
    int32_t rpm;
    int32_t register_cps;
    double cps;
    double voltage;

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

        *vel = cps;
    }

    return err;
}

int AmcEncoderSensor::motor_current_read(int motor_id, int16_t *actual_motor_current, int timeout) {
    int err;
    my_can_frame f;
    uint32_t enc;

    err = PDO_read(motor_sockets_->motor_enc_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX3_ID + motor_id)) {
        
        int16_t actual_motor_current_int =  ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8);
        *actual_motor_current = actual_motor_current_int;
        
    }

    return err;
}

int AmcEncoderSensor::motor_stat_voltage_n_io_read(int motor_id, int16_t *drive_stat, int16_t *system_stat, int16_t *voltage, int16_t *io_stat, int timeout) {
    int err;
    my_can_frame f;
    uint32_t enc;

    err = PDO_read(motor_sockets_->motor_system_status_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }
    // std::cerr << "ID " << PDO_TX4_ID + motor_id << "  f_id " << f.id << std::endl;
    if (f.id == (PDO_TX4_ID + motor_id)) {
        
        int16_t drive_stat_int =  ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8);
        int16_t system_stat_int =  ((uint32_t) f.data[2] << 0) | ((uint32_t) f.data[3] << 8);
        int16_t voltage_int =  ((uint32_t) f.data[4] << 0) | ((uint32_t) f.data[5] << 8);
        int16_t io_stat_int =  ((uint32_t) f.data[6] << 0) | ((uint32_t) f.data[7] << 8);
        
        *drive_stat = drive_stat_int;
        *system_stat = system_stat_int;
        *voltage = voltage_int;
        *io_stat = io_stat_int;
        
    }

    return err;
}

double AmcEncoderSensor::motor_cps_to_rpm(double counts_per_sec) {

    double m_per_sec = (counts_per_sec) / AMC_CPR;
    return m_per_sec * 60;
}

int AmcEncoderSensor::node_guarding_response_read(uint16_t *response, int timeout){
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


int AmcEncoderSensor::readData(AmcEncoderData *encoder_data) {

    auto err_pdo_1_ = motor_status_n_pos_read(motor_id_, status_register_fb_, actual_position_fb_, 1);
    auto err_pdo_2_ = motor_vel_read(motor_id_, vel_fb_, 1);
    auto err_pdo_3_ = motor_current_read(motor_id_, actual_motor_current_fb_, 1);
    auto err_pdo_4_ = motor_stat_voltage_n_io_read(motor_id_, drive_stat_fb_, system_stat_fb_, voltage_fb_, io_stat_fb_, 1);
    
    if (0 == err_pdo_1_) {
        // status register
        encoder_data->status_m = status_register_fb_[0];
        // position register
        encoder_data->pos_m = actual_position_fb_[0];

    }
    else{
        logger_->debug("[{}] status & pos read error", motor_name_);
    }

    if (0 == err_pdo_2_) {
        // velocity register
        double vel = vel_fb_[0];
        vel = vel/(pow(2, 17)/(ki * ks));
        encoder_data->vel_m = vel;
    }
    else{
        logger_->debug("[{}] vel read error", motor_name_);
    }

    if (0 == err_pdo_3_) {
        // motor current register
        int16_t actual_motor_current = actual_motor_current_fb_[0] / (pow(2, 13)/kp);
        encoder_data->actual_motor_current_m = actual_motor_current;
        
    }else {
        logger_->debug("[{}] motor current read error", motor_name_);
    }

    if (0 == err_pdo_4_) {
        // drive stat register
        encoder_data->drive_stat_m = drive_stat_fb_[0];
        // system stat register
        encoder_data->system_stat_m = system_stat_fb_[0];
        // voltage register
        int16_t voltage = voltage_fb_[0] / (pow(2, 14)/(1.05 * kov));
        encoder_data->voltage_m = voltage;
        // io stat register
        encoder_data->io_stat_m = io_stat_fb_[0];
    }else {
        logger_->debug("[{}] drive stat, system stat, voltage, io read error", motor_name_);
    }

    return 0;

}

void AmcEncoderSensor::stop_read_thread() {
    stop_read_thread_flag = true;
    
}

void AmcEncoderSensor::readMotorData() {


    while (!stop_read_thread_flag) {

        auto start_time = std::chrono::system_clock::now();

        {
            
            if(reading_loop_started) {
                
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


        std::this_thread::sleep_for(std::chrono::microseconds(18000 - time_passed_in_read.count()));

    }

}

void AmcEncoderSensor::getData(Json::Value &sensor_data) {

    int16_t status_1 = 0;
    int16_t status_2 = 0;
    
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
        sensor_data["guard_err"] = encoder_data_q_element.guard_err_m;
        sensor_data["read_status"] = true;
        sensor_data["amc_drive_stat"] = encoder_data_q_element.drive_stat_m;
        sensor_data["amc_system_stat"] = encoder_data_q_element.system_stat_m;
        sensor_data["voltage"] = encoder_data_q_element.voltage_m;
        sensor_data["io_stat"] = encoder_data_q_element.io_stat_m;
        sensor_data["amc_drive_stat_1"] = status_1;
        sensor_data["amc_drive_stat_2"] = status_2;

        logger_->debug("motor status [{}], motor count [{}]", encoder_data_q_element.status_m, encoder_data_q_element.pos_m);
        logger_->debug("motor velocity [{}] cps", encoder_data_q_element.vel_m);
        logger_->debug("motor current [{}]", encoder_data_q_element.actual_motor_current_m);
        logger_->debug("motor drive stat [{}]", encoder_data_q_element.drive_stat_m);
        logger_->debug("motor system stat [{}]", encoder_data_q_element.system_stat_m);
        logger_->debug("motor voltage [{}]", encoder_data_q_element.voltage_m);
        logger_->debug("motor io stat [{}]", encoder_data_q_element.io_stat_m);

		auto is_fault = (( sensor_data["status"].asInt() & (1 << 3)) >> 3);

        if(is_fault == 1) {
            logger_->debug("[{}] Fault detected. ", motor_sockets_->motor_name_);
            status_1 = read_drive_status_1();
            status_2 = read_drive_status_2();
            sensor_data["amc_drive_stat_1"] = status_1;
            sensor_data["amc_drive_stat_2"] = status_2;
            logger_->debug("[{}] status 1 [{}], status 2 [{}]", motor_sockets_->motor_name_, status_1, status_2);
        }

    } else {
        sensor_data["read_status"] = false;
        logger_->debug("[{}] Sensor Data Queue Empty. ", motor_sockets_->motor_name_);
    }


    read_mutex_.unlock();
}
