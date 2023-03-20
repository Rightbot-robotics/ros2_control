#include <motor_actuator/motor_actuator.hpp>


#include <thread>
#include <condition_variable>
#include <chrono>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

using namespace std;

// #include "ros2_control_test_assets/descriptions.hpp"
// #include "ros2_control_test_assets/components_urdfs.hpp"
#include "hardware_interface/actuator.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "test/test_urdf.hpp"

std::condition_variable cv;

// This mutex is used for three purposes:
// 1) to synchronize accesses to i
// 2) to synchronize accesses to std::cerr
// 3) for the condition variable cv
std::mutex cv_m;

bool message_received = false;

Sockets::SocketsSPtr motor_sockets_1;
MotorActuator::MotorActuatorSPtr motor_actuator_1;
Json::Value actuator_data;
std::mutex sync_mutex; // for sync of message_received variable
std::shared_ptr<spdlog::logger> logger_;


void test_write() {

    while (true) {

        std::unique_lock<std::mutex> lk(cv_m);
        std::cerr << "Waiting... \n";

        cv.wait(lk, [] { return message_received; });
        std::cerr << "...finished waiting \n";

        // motor_actuator_1->writeData(actuator_data);

        sync_mutex.lock();
        message_received = false;
        sync_mutex.unlock();

    }
}

void signals() {

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(3));

        {
            std::lock_guard<std::mutex> lk(cv_m);

            actuator_data["counts"] = actuator_data["counts"].asInt() + 1;

            sync_mutex.lock();
            message_received = true;
            sync_mutex.unlock();

        }
        std::cerr << "Notifying...\n";
        cv.notify_one();
    }
}

int main() {

     spdlog::init_thread_pool(8192, 1);
     auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
     console_sink->set_level(spdlog::level::info);
     auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/data/logs/robot_logs/hardware_interface_logs/hardware_interface_data_logs.txt", 1024*1024*100, 3);
     rotating_sink->set_level(spdlog::level::debug);
     std::vector<spdlog::sink_ptr> sinks {console_sink,rotating_sink};
     auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
     root_logger->set_level(spdlog::level::debug);
     spdlog::register_logger(root_logger);

    logger_ = spdlog::get("hardware_interface")->clone("test_motor_actuator");

    logger_->info("In Interface Initialization");
    logger_->flush();

    Json::Value config_data;/// load config data from json file here

    // motor_sockets_1 = std::make_shared<Sockets>(12, "wheel_left");

    // hardware_interface::Actuator actuator_hw(std::make_unique<MotorActuator>());
    // hardware_interface::HardwareInfo mock_hw_info{};
    // auto state = actuator_hw.initialize(mock_hw_info);
    // motor_actuator_1->motorCommand(12, "velocity", motor_actuator_1->setPosition(0,0,0,0,0), motor_actuator_1->setVelocity(1,15,10,1,1) );

    // actuator_data["counts"] = 0;

    // std::thread write_thread(test_write);
    // std::thread subscribe_thread(signals);

    // write_thread.join();
    // subscribe_thread.join();

    std::string urdf_to_test =
    std::string(ros2_control_test_assets::urdf_head) +
    ros2_control_test_assets::hardware_resources +
    ros2_control_test_assets::urdf_tail;
    const auto control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_to_test);

    const auto hardware_info = control_hardware.front();

    logger_->info("test {}",hardware_info.name);
    logger_->info("test {}",hardware_info.type);

    hardware_interface::Actuator actuator_hw(std::make_unique<MotorActuator>());
    auto state = actuator_hw.initialize(hardware_info);
    state = actuator_hw.configure();

    return 0;

}
