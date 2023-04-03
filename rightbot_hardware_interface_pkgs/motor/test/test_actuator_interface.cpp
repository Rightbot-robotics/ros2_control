// #################### Header Files ##############################

#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <bits/stdc++.h>
#include <mutex>
#include <atomic>

#include <chrono>
#include <cmath>
#include <memory>
#include <functional>
#include <string>
#include <typeinfo>

#include <motor/sockets.hpp>
#include <motor/motor.hpp>
#include <motor/motor_controls.hpp>
#include <motor/motor_interface.hpp>

//###### Motor #######
#include <socketcan_pkg/printd.h>
#include <socketcan_pkg/socketcan.h>
#include <canopen_pkg/canopen.h>
#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

// ### Spd logger ###
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
// ##################

// ############################ Robot Hardware config ##########################

// ConfigParser::ConfigParserSPtr config_parser = std::make_shared<ConfigParser>();
// Sockets::SocketsSPtr left_motor_sockets = std::make_shared<Sockets>();
// Motor::MotorSPtr motor_sockets = std::make_shared<Motor>();
// MotorControls::MotorControlsSPtr motor_controls_ ;
// ############################ Namespace ##########################

std::shared_ptr<spdlog::logger> logger_;

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    spdlog::init_thread_pool(8192, 1);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            "/data/logs/robot_logs/hardware_interface_logs/hardware_interface_data_logs.txt", 1024 * 1024 * 100, 3);
    rotating_sink->set_level(spdlog::level::debug);
    std::vector<spdlog::sink_ptr> sinks{console_sink, rotating_sink};
    auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(),
                                                              spdlog::thread_pool(),
                                                              spdlog::async_overflow_policy::block);
    root_logger->set_level(spdlog::level::debug);
    spdlog::register_logger(root_logger);
// 
//     logger_ = spdlog::get("hardware_interface")->clone("test_hardware_interface");

//     logger_->debug("In Interface Initialization");
    std::shared_ptr<MotorInterface> node;
    node = std::make_shared<MotorInterface>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    MotorControls::position_cmd_t position_cmd_element_;
    MotorControls::velocity_cmd_t velocity_cmd_element_;
    position_cmd_element_ = {.timeout =10,
            .relative_pos=0,
            .max_vel=0,
            .accel=0,
            .decel=0
    };

    velocity_cmd_element_ = {.timeout =10,
            .velocity=1,
            .max_vel=20,
            .accel=1,
            .decel=1
    };
    //std::cout << "sending velocity" << std::endl;
    //motor_controls_->motor_command(0x0C, "velocity", position_cmd_element_, velocity_cmd_element_);

    while (rclcpp::ok()) {
        executor.spin_some();
        //logger->debug("spin some");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rclcpp::shutdown();
    return 0;
}
