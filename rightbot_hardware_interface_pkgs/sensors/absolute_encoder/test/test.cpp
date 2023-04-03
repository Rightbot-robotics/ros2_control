
#include "absolute_encoder/absolute_encoder.hpp"


AbsoluteEncoder::AbsoluteEncoderSPtr absolute_encoder_;

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

std::shared_ptr<spdlog::logger> logger_;
uint32_t absolute_encoder_count;

Json::Value sensor_;
void data_request(){

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        absolute_encoder_->requestData();
    }
}

void read_data(){
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        absolute_encoder_->getData(sensor_);
        logger_->info("Angle Value: {}",sensor_["angle"]);
        logger_->info("Read Status: {}",sensor_["read_status"]);
        
    }
}

int main(){

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

    logger_ = spdlog::get("controller_manager")->clone("absolute_encoder");

    absolute_encoder_= std::make_shared<AbsoluteEncoder>(1);

    std::thread read_thread(read_data);
    std::thread data_request_thread(data_request);

    read_thread.join();
    data_request_thread.join();

    return 0;
    
}