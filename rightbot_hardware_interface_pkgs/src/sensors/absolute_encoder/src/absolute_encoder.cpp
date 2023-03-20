//
// Created by amrapali on 11/30/22.
//

#include "absolute_encoder/absolute_encoder.hpp"

AbsoluteEncoder::AbsoluteEncoder(int sensor_id ) {
    logger_ = spdlog::get("hardware_interface")->clone("absolute_encoder");
    sensor_id_ = sensor_id;
    absolute_encoder_sockets_ = std::make_shared<AbsoluteEncoderSockets>(sensor_id_);

    absolute_encoder_init_pos = ABS_POSITION;
    abs_motor_ppr = ABS_MOTOR_PPR;
    reading_loop_started = false;

    initSensor();
    enableSensor();

    read_enc_data_thread_ = std::thread(&AbsoluteEncoder::readData, this);

}

AbsoluteEncoder::~AbsoluteEncoder() {

}

int AbsoluteEncoder::initSensor(){

    int err = 0;

    err |= NMT_change_state(absolute_encoder_sockets_->abs_cfg_fd, sensor_id_, NMT_Enter_PreOperational);
    if (err != 0) {
        logger_->debug("Err in NMT_change_state: NMT_Enter_PreOperational ");
        return err;
    }

    err |= encoder_Transmit_PDO_n_Parameter(sensor_id_, 1, PDO_TX1_ID + sensor_id_);
    if (err != 0) {
        logger_->debug("Err in encoder_Transmit_PDO_n_Parameter ");
        return err;
    }

}

int AbsoluteEncoder::enableSensor(){

    int err = 0;

    err |= NMT_change_state(absolute_encoder_sockets_->abs_cfg_fd, sensor_id_, NMT_Start_Node);

    if (err != 0) {
        logger_->debug("Err in NMT_change_state: NMT_Start_Node ");
        return err;
    }

}

int AbsoluteEncoder::requestData(){
    Socketcan_t data[1];
 	data[0].size = 1;
 	data[0].data = 0x00;
 	return socketcan_write(absolute_encoder_sockets_->abs_cfg_fd, 128, 1, data);
}

int AbsoluteEncoder::encoder_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob) {

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x1800 + n - 1;
    d.subindex = 0x02;
    d.data.size = 1;
    d.data.data = 0x01;

    return SDO_write(absolute_encoder_sockets_->abs_cfg_fd, &d);
}

int AbsoluteEncoder::encoder_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping *objects) {

    int err = 0;

    // Set number of objects to zero
    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x1A00 + n - 1;
    d.subindex = 0x00;
    d.data.size = 1;
    d.data.data = 0;

    err = SDO_write(absolute_encoder_sockets_->abs_cfg_fd, &d);
    if (err != 0) {
        return err;
    }

    // Write objects
    d.data.size = 4;
    for (int i = 0; i < num_objects; i++) {
        Epos_pdo_mapping obj = objects[i];

        d.subindex = i + 1;
        d.data.data = ((uint32_t) obj.index << 16) | (obj.subindex << 8) | (obj.length);
        err = SDO_write(absolute_encoder_sockets_->abs_cfg_fd, &d);
        if (err != 0) {
            return err;
        }
    }

    // Set Correct number of objects
    d.subindex = 0x00;
    d.data.size = 1;
    d.data.data = num_objects;

    return SDO_write(absolute_encoder_sockets_->abs_cfg_fd, &d);

}

int AbsoluteEncoder::readEncCounts(float* angle){

    int err;
    my_can_frame f;
    uint32_t enc;
    int position_count;
    err = PDO_read(absolute_encoder_sockets_->abs_pdo_fd, &f, 1);

    if (f.id == (PDO_TX1_ID + sensor_id_)) {

        enc = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                            ((uint32_t) f.data[3] << 24);
        
        position_count = static_cast<int>(enc) - absolute_encoder_init_pos;
         logger_->debug(" Absolute Encoder Counts Value: {}", position_count);

        *angle = convertToAngle(position_count);
        // logger_->debug(" Absolute Encoder Angle Value: {}", *angle);

    }

    return err;
}

float AbsoluteEncoder::convertToAngle(int counts){
    float angle;
    angle = (-1.0 * static_cast<float>(counts)*360)/static_cast<float>(abs_motor_ppr);
    return angle;

}

void AbsoluteEncoder::readData(){

    while (true) {

        auto start_time = std::chrono::system_clock::now();
        read_mutex_.lock();

        if(reading_loop_started) {
            float angle;
            int err;
            feedback_s feedback_s_b_;

            err = readEncCounts(&angle);

            //
            auto now = std::chrono::high_resolution_clock::now();
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto epoch = now_ms.time_since_epoch();
            auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
            long duration = value.count();
            //

            if (err == 0) {

                feedback_s_b_.angle = angle;
                feedback_s_b_.time_sys = duration;
                feedback_s_b_.read_status = true;

                q_angle_data_.push_back(feedback_s_b_);

            } else {

                // logger_->debug("Enc Data Read Unsuccesful");
            }
        }
        read_mutex_.unlock();


        auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now() - start_time);
        logger_->debug("Time in execution [ readEncData() ]: [{}] us", time_passed_in_read.count());

        std::this_thread::sleep_for(std::chrono::microseconds(10000 - time_passed_in_read.count()));

    }

}

void AbsoluteEncoder::getData(Json::Value &sensor_data){

    read_mutex_.lock();
    reading_loop_started = true;

    if (!q_angle_data_.empty()) {


        auto angle_data_q_element = q_angle_data_.back();
        q_angle_data_.pop_front();

        logger_->debug("Read deque size after pop: {}", q_angle_data_.size());
        if (q_angle_data_.size() > 10) {
            logger_->warn("Read deque size : [{}]", q_angle_data_.size());
            q_angle_data_.clear();
        }

        sensor_data["angle"] = angle_data_q_element.angle;
        sensor_data["timestamp"] = std::to_string(angle_data_q_element.time_sys);
        sensor_data["read_status"] = true;

        // logger_->debug("Absolute Encoder Angle: {}", angle_data_q_element.angle);

    } else {
        sensor_data["read_status"] = false;
        logger_->debug("Absolute Encoder Data Queue Empty");
    }


    read_mutex_.unlock();

    
    
}
