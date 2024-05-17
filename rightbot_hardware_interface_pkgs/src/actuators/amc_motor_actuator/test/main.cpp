#include "amc_motor_actuator/amc_motor_actuator.hpp"
#include <thread>
#include <condition_variable>
#include <chrono>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

std::shared_ptr<spdlog::logger> logger_;
AmcMotorActuatorSockets::AmcMotorActuatorSocketsSPtr amc_motor_actuator_sockets_;
AmcMotorActuator::AmcMotorActuatorSPtr amc_motor_;

Json::Value actuator_data_;



int main() {

     spdlog::init_thread_pool(8192, 1);
     auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
     console_sink->set_level(spdlog::level::info);
     auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/home/rightbot/hardware_interface_data_logs.txt", 1024*1024*100, 3);
     rotating_sink->set_level(spdlog::level::debug);
     std::vector<spdlog::sink_ptr> sinks {console_sink,rotating_sink};
     auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
     root_logger->set_level(spdlog::level::debug);
     spdlog::register_logger(root_logger);

    logger_ = spdlog::get("hardware_interface")->clone("test_amc_motor_actuator");

    logger_->info("In Interface Initialization");
    logger_->flush();

    amc_motor_actuator_sockets_ = std::make_shared<AmcMotorActuatorSockets>(1, "motor_actuator");

    actuator_data_["timeout"] = 10;
    actuator_data_["mode"] = "position";
    actuator_data_["velocity"] = 0;
    actuator_data_["relative_pos"] = 145635;
    actuator_data_["max_vel"] = 15;
    actuator_data_["accel"] = 15;
    actuator_data_["decel"] = 15;

    amc_motor_->writeData(actuator_data_);
    return 0;

}
