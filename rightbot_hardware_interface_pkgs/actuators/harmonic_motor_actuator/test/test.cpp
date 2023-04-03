#include "harmonic_motor_actuator/harmonic_motor_actuator.hpp"
#include <thread>
#include <condition_variable>
#include <chrono>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

// std::shared_ptr<spdlog::logger> logger_;
HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr harmonic_motor_actuator_sockets_;
HarmonicMotorActuator::HarmonicMotorActuatorSPtr harmonic_motor_;

Json::Value actuator_data_;



int main() {

     spdlog::init_thread_pool(8192, 1);
     auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
     console_sink->set_level(spdlog::level::info);
     auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/data/logs/robot_logs/hardware_interface_logs/hardware_interface_data_logs.txt", 1024*1024*100, 3);
     rotating_sink->set_level(spdlog::level::debug);
     std::vector<spdlog::sink_ptr> sinks {console_sink,rotating_sink};
    //  auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
    //  root_logger->set_level(spdlog::level::debug);
    //  spdlog::register_logger(root_logger);

    // logger_ = spdlog::get("hardware_interface")->clone("test_harmonic_motor_actuator");

    // logger_->info("In Interface Initialization");
    // logger_->flush();

    harmonic_motor_actuator_sockets_ = std::make_shared<HarmonicMotorActuatorSockets>(1, "harmonic_motor");

    // harmonic_motor_ = std::make_shared<HarmonicMotorActuator>(harmonic_motor_actuator_sockets_);

    actuator_data_["timeout"] = 10;
    actuator_data_["mode"] = "positon";
    actuator_data_["velocity"] = 0;
    actuator_data_["relative_pos"] = 145635;
    actuator_data_["max_vel"] = 15;
    actuator_data_["accel"] = 15;
    actuator_data_["decel"] = 15;

    harmonic_motor_->writeData(actuator_data_);
    return 0;

}