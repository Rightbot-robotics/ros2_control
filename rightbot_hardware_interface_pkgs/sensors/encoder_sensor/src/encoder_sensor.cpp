#include <encoder_sensor/encoder_sensor.hpp>

using namespace std;

EncoderData::EncoderData() {
    status_m = 0;
    battery_vol_m = 0.0;
    input_states_m = 0;
    pos_m = 0;
    vel_m = 0.0;
    manufacturer_reg_m = 0;
    latched_fault_m = 0;
    guard_err_m = 0;
    time_sys = 0;
    read_status_voltage = false;
    read_status_encoder = false;
    read_status_velocity = false;
    read_status_mf_register = false;

}

EncoderSensor::EncoderSensor() {

    

}

EncoderSensor::~EncoderSensor() {

    read_motor_data_thread_.join();
//    update_data_thread_.join();

}

void EncoderSensor::initialize(Json::Value &config_data, Sockets::SocketsSPtr motor_sockets){
    // logger_ = spdlog::get("hardware_interface")->clone("encoder_sensor");
    motor_sockets_ = motor_sockets;
    motor_feedback_ = std::make_shared<MotorFeedback>(motor_sockets_);
    motor_id_ = config_data["motor_id"].asInt();
    motor_name_= config_data["motor_name"].asString();
    axis_ = config_data["motor_axis"].asInt();
    
    read_err_ = 0;
    message_received = false;
    reading_loop_started = false;

    feedback_s_m.status_m = 0;
    feedback_s_m.battery_vol_m = 0.0;
    feedback_s_m.input_states_m = 0;
    feedback_s_m.pos_m = 0;
    feedback_s_m.vel_m = 0.0;
    feedback_s_m.manufacturer_reg_m =0;
    feedback_s_m.latched_fault_m =0;
    feedback_s_m.guard_err_m = 0;

//    update_data_thread_ = std::thread(&EncoderSensor::updateData, this);
    read_motor_data_thread_ = std::thread(&EncoderSensor::readMotorData, this);



}

void EncoderSensor::init_json(Json::Value &config_data) {

    // sensor_config_data_ = config_data;

    // for (int i = 0; i < sensor_config_data_["state_interface"]["sensor_list"].size(); i++) {

    //     if (motor_id_ == sensor_config_data_["state_interface"]["sensor_list"][i]["can_id"].asInt()){
    //         axis_ = sensor_config_data_["state_interface"]["sensor_list"][i]["motor_axis"].asInt();
    //         logger_->debug("Encoder config info found. Motor axis of Motor id: {} is {}", motor_id_, axis_);
    //         motor_name_ = sensor_config_data_["state_interface"]["sensor_list"][i]["sensor_name"].asString();
    //     }
    // } 

    // if(axis_ == 0){
    //     logger_->info("Encoder config axis info not found for Motor id: {}", motor_id_);
    // }
    
}

int EncoderSensor::readData(int motor_id, EncoderData *encoder_data) {

    auto start_time = std::chrono::system_clock::now();
    auto read_error_code_map = motor_feedback_->motorFeedback(motor_id, &feedback_s_m);
    auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - start_time);
    // logger_->info("Time in execution [ readMotorFeedback() ]: [{}] us", time_passed_in_read.count());
   
    if (0 == read_error_code_map["voltage"]) {

        encoder_data->status_m = feedback_s_m.status_m;
        encoder_data->battery_vol_m = feedback_s_m.battery_vol_m;
        encoder_data->input_states_m = feedback_s_m.input_states_m;
        encoder_data->read_status_voltage = true;

    }
    else{
        encoder_data->read_status_voltage = false;
    }

    if (0 == read_error_code_map["encoder"]) {
        encoder_data->pos_m = feedback_s_m.pos_m;
        // logger_->debug("Encoder_position: [{}]", feedback_s_m.pos_m);
        // logger_->debug("[{}] Encoder_position: [{}]", motor_sockets_->motor_name_, feedback_s_m.pos_m);
        encoder_data->read_status_encoder = true;
    }
    else{
        encoder_data->read_status_encoder = false;
    }

    if (0 == read_error_code_map["velocity"]) {
        encoder_data->vel_m = feedback_s_m.vel_m;
        // logger_->debug("Encoder_Velocity: [{}]", feedback_s_m.vel_m);
        // logger_->debug("[{}] Encoder_velocity: [{}]", motor_sockets_->motor_name_, feedback_s_m.vel_m);
        encoder_data->read_status_velocity = true;
    }
    {
        encoder_data->read_status_velocity = false;
    }

    if (0 == read_error_code_map["mf_register"]) {
        encoder_data->manufacturer_reg_m = feedback_s_m.manufacturer_reg_m;
        encoder_data->latched_fault_m = feedback_s_m.latched_fault_m;
        encoder_data->read_status_mf_register = true;
    }
    else{
        encoder_data->read_status_mf_register = false;
    }

    encoder_data->guard_err_m = feedback_s_m.guard_err_m;


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
	// logger_->debug("{} pos read successful",motor_sockets_->motor_name_);
    }
    return 0;

}


void EncoderSensor::updateData() {


    while (true) {
        std::unique_lock<std::mutex> lk(read_mutex_);
        //std::cout << "Waiting... \n";

        cv.wait(lk, [this] { return message_received; });
        //std::cout << "...finished waiting \n";

        message_received = false;

        lk.unlock();
    }

}

void EncoderSensor::readMotorData() {


    while (true) {

        auto start_time = std::chrono::system_clock::now();

        {
            
            if(reading_loop_started) {
                int err = readData(motor_id_, &encoder_data_);

                if (err == 0) {

                    message_received = true;
                    read_mutex_.lock();
                    q_encoder_data_.push_back(encoder_data_);
                    read_mutex_.unlock();
                }
                else {
                    // logger_->warn("incomplete data received, not pushing to sensor data q");
                }
            }
            

        }
//        cv.notify_one();

        auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now() - start_time);

        // if (time_passed_in_read.count() > 5000){
        //     logger_->debug("Time in execution [ readMotorData() ]: [{}] us", time_passed_in_read.count());
        // }
        // logger_->debug("Actuator [{}] Time in execution [ readMotorData() ]: [{}] us",motor_name_ , time_passed_in_read.count());
        

        std::this_thread::sleep_for(std::chrono::microseconds(10000 - time_passed_in_read.count()));

    }

}

void EncoderSensor::getData(Json::Value &sensor_data) {

    read_mutex_.lock();
    reading_loop_started = true;

    if (!q_encoder_data_.empty()) {


        auto encoder_data_q_element = q_encoder_data_.back();
        q_encoder_data_.pop_front();

        // logger_->debug("Read deque size after pop: {}", q_encoder_data_.size());
        if (q_encoder_data_.size() > 10) {
            //logger_->error("Read deque size : [{}]", q_encoder_data_.size());
            q_encoder_data_.clear();
        }

        sensor_data["status"] = encoder_data_q_element.status_m;
        sensor_data["battery_voltage"] = encoder_data_q_element.battery_vol_m;
        sensor_data["input_states"] = encoder_data_q_element.input_states_m;
        sensor_data["counts"] = encoder_data_q_element.pos_m;
        sensor_data["velocity"] = encoder_data_q_element.vel_m;
        sensor_data["manufacturer_register"] = encoder_data_q_element.manufacturer_reg_m;
        sensor_data["latched_fault"] = encoder_data_q_element.latched_fault_m;
        sensor_data["timestamp"] = to_string(encoder_data_q_element.time_sys);
        sensor_data["guard_err"] = encoder_data_q_element.guard_err_m;
        sensor_data["read_status"] = true;
        sensor_data["read_status_voltage"] = encoder_data_q_element.read_status_voltage;
        sensor_data["read_status_encoder"] = encoder_data_q_element.read_status_encoder;
        sensor_data["read_status_velocity"] = encoder_data_q_element.read_status_velocity;
        sensor_data["read_status_mf_register"] = encoder_data_q_element.read_status_mf_register;

        // logger_->debug("[{}] Status: [{}], Manufacturer Reg: [{}], Latched Fault: [{}]", motor_sockets_->motor_name_, encoder_data_q_element.status_m, encoder_data_q_element.manufacturer_reg_m, encoder_data_q_element.latched_fault_m);
        // logger_->debug("[{}] Battery Voltage: {} V", motor_sockets_->motor_name_, encoder_data_q_element.battery_vol_m);
        // logger_->debug("[{}] Position: {} counts, Velocity: {} rpm", motor_sockets_->motor_name_, encoder_data_q_element.pos_m, encoder_data_q_element.vel_m);
//        logger_->debug("[{}] Velocity: {}", motor_sockets_->motor_name_, encoder_data_q_element.vel_m);
//        logger_->debug("[{}] Manufacturer Reg: {}", motor_sockets_->motor_name_, encoder_data_q_element.manufacturer_reg_m);
//        logger_->debug("[{}] Latched Fault: {}", motor_sockets_->motor_name_, encoder_data_q_element.latched_fault_m);
        // logger_->debug("[{}] Guard Err: {}", motor_sockets_->motor_name_, encoder_data_q_element.guard_err_m);

    } else {
        sensor_data["read_status"] = false;
        // logger_->debug("Motor Data Queue Empty");
    }


    read_mutex_.unlock();
}
