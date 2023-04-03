#include <barcode_sensor/barcode_sensor.hpp>

using namespace std;


BarcodeSensor::BarcodeSensor(Json::Value &config_data, int barcode_id, int barcode_base_id) {

    logger_ = spdlog::get("hardware_interface")->clone("barcode_sensor");
    init_json(config_data);

    barcode_id_ = barcode_id;
    gls_barcode_ = std::make_shared<GlsBarcode>(barcode_id_, barcode_base_id);
    gls_barcode_->gls_init(barcode_id_);
    gls_barcode_->gls_enable(barcode_id_);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    read_err_ = 0;
    message_received = false;
    reading_loop_started = false;

    read_barcode_data_thread_ = std::thread(&BarcodeSensor::readBarcodeData, this);
//    update_data_thread_ = std::thread(&BarcodeSensor::updateData, this);
}

BarcodeSensor::~BarcodeSensor() {

//    update_data_thread_.join();
    read_barcode_data_thread_.join();
}

void BarcodeSensor::init_json(Json::Value &config_data) {

    Json::Value config_sensor_data_;

    config_sensor_data_ = config_data;

    sensor_data_ = config_sensor_data_["barcode_sensor"]; 

    // logger_->info("barcode sensor_data status [{}]",sensor_data_["status"].asString());
}

void BarcodeSensor::readData() {

    read_err_ = gls_barcode_->barcodeFeedback(barcode_id_, &feedback_s_b_);

}

void BarcodeSensor::updateData() {

    while (true) {
        std::unique_lock<std::mutex> lk(read_mutex_);
        //std::cout << "Waiting... \n";

        cv.wait(lk, [this] { return message_received; });
        //std::cout << "...finished waiting \n";

        message_received = false;

        lk.unlock();
    }
}

void BarcodeSensor::readBarcodeData() {


    while (true) {

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        {

            
            if(reading_loop_started) {

                readData();
                if (read_err_ == 0) {
//                std::lock_guard<std::mutex> lk(read_mutex_);
                    message_received = true;
                    read_mutex_.lock();
                    q_barcode_data_.push_back(feedback_s_b_);
                    read_mutex_.unlock();
                }

            }

            

        }
//        cv.notify_one();
    }

}


void BarcodeSensor::getData(Json::Value &sensor_data) {

    read_mutex_.lock();
    reading_loop_started = true;

    if (!q_barcode_data_.empty()) {

        auto encoder_barcode_q_element = q_barcode_data_.back();
        q_barcode_data_.pop_front();
        if (q_barcode_data_.size() > 1000) {
            logger_->warn("Read deque size : [{}]", q_barcode_data_.size());
            q_barcode_data_.clear();
        }
        sensor_data["x"] = encoder_barcode_q_element.x_b;
        sensor_data["y"] = encoder_barcode_q_element.y_b;
        sensor_data["angle"] = encoder_barcode_q_element.ang_b;
        sensor_data["sensor_time"] = static_cast<uint32_t>(encoder_barcode_q_element.sensor_time_b);
        sensor_data["decode_time"] = static_cast<uint32_t>(encoder_barcode_q_element.decode_time_b);
        sensor_data["tag_x"] = static_cast<uint32_t>(encoder_barcode_q_element.tag_x_b);
        sensor_data["tag_y"] = static_cast<uint32_t>(encoder_barcode_q_element.tag_y_b);
        sensor_data["timestamp"] = to_string(encoder_barcode_q_element.time_sys);
        sensor_data["read_status"] = encoder_barcode_q_element.read_status;

        logger_->info("Barcode x: [{}], y: [{}], angle: [{}]", sensor_data["x"], sensor_data["y"], sensor_data["angle"]);
        logger_->debug("Barcode tag_x: [{}], tag_y: [{}]", sensor_data["tag_x"], sensor_data["tag_y"]);

        if ((tag_x_u != sensor_data["tag_x"].asInt()) || (tag_y_u != sensor_data["tag_y"].asInt()) ){
            tag_x_u = sensor_data["tag_x"].asInt();
            tag_y_u = sensor_data["tag_y"].asInt();

            logger_->debug("Barcode unique tag_x: [{}], tag_y: [{}]", sensor_data["tag_x"], sensor_data["tag_y"]);
            
        }
        

    } else {
        sensor_data["read_status"] = false;
        logger_->debug("Barcode Data Queue Empty");
    }

    // logger_->debug("Data Read Sensor 2");
    read_mutex_.unlock();


}
