#include <barcode_sensor/barcode_sensor.hpp>
#include <thread>
#include <gls_barcode/gls_barcode.hpp>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

using namespace std;

BarcodeSensor::BarcodeSensorSPtr barcode_sensor_1;

// void read_data() {

//     while (true) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//         Json::Value data;
//         barcode_sensor_1->getData(data);
//         std::cout << "x: " << data["x"].asInt() << std::endl;
//         std::cout << "y: " << data["y"].asInt() << std::endl;
//     }

// }
Json::Value barcode_data_1;

int main() {

    spdlog::init_thread_pool(8192, 1);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    std::string file_path = "/data/logs/robot_logs/hardware_interface_logs/hardware_interface.log";

    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file_path,
                                                                                1024 * 1024 * 100, 3);
    rotating_sink->set_level(spdlog::level::debug);
    std::vector<spdlog::sink_ptr> sinks{rotating_sink, console_sink};
    auto root_logger = std::make_shared<spdlog::async_logger>("controller_manager", sinks.begin(), sinks.end(),
                                                              spdlog::thread_pool(),
                                                              spdlog::async_overflow_policy::block);

    auto root_logger_1 = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(),
                                                                spdlog::thread_pool(),
                                                                spdlog::async_overflow_policy::block);

    root_logger->set_level(spdlog::level::debug);
    root_logger_1->set_level(spdlog::level::debug);
    spdlog::register_logger(root_logger);
    spdlog::register_logger(root_logger_1);

    spdlog::flush_every(std::chrono::seconds(1));


    Json::Value config_data;/// load config data from json file here

    barcode_sensor_1 = std::make_shared<BarcodeSensor>(config_data, 1, 1152);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        barcode_sensor_1->getData(barcode_data_1);
        std::cout << "test: x: " << barcode_data_1["x"].asDouble() << std::endl;
        std::cout << "test: y: " << barcode_data_1["y"].asDouble() << std::endl;
        std::cout << "test: ang: " << barcode_data_1["angle"].asDouble() << std::endl;
        std::cout << "test: sensor_time: " << barcode_data_1["sensor_time"].asInt64() << std::endl;
        std::cout << "test: decode_time: " << barcode_data_1["decode_time"].asInt64() << std::endl;
        std::cout << "test:  tag_x: " << barcode_data_1["tag_x"].asInt64() << std::endl;
        std::cout << "test:  tag_y: " << barcode_data_1["tag_y"].asInt64() << std::endl;


    }
    // std::thread read_thread(read_data);
    // read_thread.join();


    return 0;

}
