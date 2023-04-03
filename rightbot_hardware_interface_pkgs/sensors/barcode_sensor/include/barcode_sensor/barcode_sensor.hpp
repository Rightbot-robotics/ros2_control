#ifndef BARCODE_SENSOR_H_
#define BARCODE_SENSOR_H_

#include <iostream>
#include <thread>
#include <mutex>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#include <sensor/sensor.hpp>
#include <json_reader/json_read.h>
#include <gls_barcode/sockets.hpp>
#include <gls_barcode/gls_barcode.hpp>
#include <deque>

#include <socketcan_pkg/printd.h>
#include <socketcan_pkg/socketcan.h>
#include <canopen_pkg/canopen.h>
#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

class BarcodeSensor : public Sensor {

public:

    typedef std::shared_ptr<BarcodeSensor> BarcodeSensorSPtr;
    typedef std::unique_ptr<BarcodeSensor> BarcodeSensorUPtr;

    BarcodeSensor(Json::Value &config_data, int barcode_id, int barcode_base_id);

    ~BarcodeSensor();

    void getData(Json::Value &sensor_data);

private:
    int barcode_id_;
    int read_err_;
    std::thread update_data_thread_;
    std::thread read_barcode_data_thread_;

    void updateData();

    Json::Value sensor_data_;
    std::mutex read_mutex_;

    void init_json(Json::Value &config_data);

    GlsBarcode::GlsBarcodeSPtr gls_barcode_;

    GlsBarcode::feedback_s feedback_s_b_;

    void readData();

    std::condition_variable cv;
    bool message_received;

    void readBarcodeData();

    std::deque<GlsBarcode::feedback_s> q_barcode_data_;
    GlsBarcode::feedback_s encoder_barcode_q_element_;

    std::shared_ptr<spdlog::logger> logger_;

    bool reading_loop_started;

    uint64_t tag_x_u = 0;
    uint64_t tag_y_u = 0;


};

#endif // BARCODE_SENSOR_H_