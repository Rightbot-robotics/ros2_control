#include<harmonic_encoder_sensor/harmonic_encoder_sensor.hpp>
#include <thread>
#include <csignal>
using namespace std;

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

HarmonicMotorActuatorSockets::HarmonicMotorActuatorSocketsSPtr harmonic_motor_actuator_sockets_;
HarmonicMotorActuator::HarmonicMotorActuatorSPtr harmonic_motor_;
HarmonicEncoderSensor::HarmonicEncoderSensorSPtr encoder_sensor_;

Json::Value actuator_data_;
Json::Value sensor_data;
std::shared_ptr<spdlog::logger> logger_;

volatile sig_atomic_t sigint_flag = 0;
void signal_event(int sig) {
sigint_flag = 1;
}

void test_read(){

    
while(!sigint_flag){

    encoder_sensor_->motor_request();
    
    encoder_sensor_->getData(sensor_data);
    // std::cout << sensor_data["counts"].asInt() << std::endl;
    logger_->debug("Harmonic motor counts : {}",sensor_data["counts"].asInt());
    logger_->debug("Harmonic motor velocity : {}",sensor_data["velocity"].asDouble());

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}

int main(){

signal(SIGINT, signal_event);
signal(SIGTERM, signal_event);
spdlog::init_thread_pool(8192, 1);
auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
console_sink->set_level(spdlog::level::info);
auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/data/logs/robot_logs/hardware_interface_logs/hardware_interface_data_logs.txt", 1024*1024*100, 3);
rotating_sink->set_level(spdlog::level::debug);
std::vector<spdlog::sink_ptr> sinks {console_sink,rotating_sink};
auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
root_logger->set_level(spdlog::level::debug);
spdlog::register_logger(root_logger);

logger_ = spdlog::get("hardware_interface")->clone("test_harmonic_motor_actuator");

logger_->info("In Interface Initialization");
logger_->flush();

harmonic_motor_actuator_sockets_ = std::make_shared<HarmonicMotorActuatorSockets>(1, "harmonic_motor");

encoder_sensor_ = std::make_shared<HarmonicEncoderSensor>(harmonic_motor_actuator_sockets_);

harmonic_motor_ = std::make_shared<HarmonicMotorActuator>(harmonic_motor_actuator_sockets_);


std::thread read_thread (test_read);

actuator_data_["timeout"] = 10;
actuator_data_["mode"] = "position";
actuator_data_["velocity"] = 0;
actuator_data_["relative_pos"] = -65535;
actuator_data_["max_vel"] = 8;
actuator_data_["accel"] = 8;
actuator_data_["decel"] = 8;


std::this_thread::sleep_for(std::chrono::milliseconds(10000));

actuator_data_["relative_pos"] = 131072;
logger_->info("writing clockwise command");
harmonic_motor_->writeData(actuator_data_);
std::this_thread::sleep_for(std::chrono::milliseconds(5000));

actuator_data_["relative_pos"] = 65536;
logger_->info("writing clockwise command");
harmonic_motor_->writeData(actuator_data_);
std::this_thread::sleep_for(std::chrono::milliseconds(5000));

actuator_data_["relative_pos"] = -65536;
logger_->info("writing clockwise command");
harmonic_motor_->writeData(actuator_data_);
std::this_thread::sleep_for(std::chrono::milliseconds(5000));

actuator_data_["relative_pos"] = -131072;
logger_->info("writing clockwise command");
harmonic_motor_->writeData(actuator_data_);
std::this_thread::sleep_for(std::chrono::milliseconds(5000));

actuator_data_["relative_pos"] = 131072;
logger_->info("writing clockwise command");
harmonic_motor_->writeData(actuator_data_);
std::this_thread::sleep_for(std::chrono::milliseconds(5000));


while(!sigint_flag) {
    
    // actuator_data_["relative_pos"] = 655360;
    // logger_->info("writing clockwise command");
    // harmonic_motor_->writeData(actuator_data_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // actuator_data_["relative_pos"] = 393216;
    // logger_->info("writing clockwise command 1");
    // harmonic_motor_->writeData(actuator_data_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));


//   actuator_data_["relative_pos"] = -65535;
//   logger_->info("writing clockwise command");
//   harmonic_motor_->writeData(actuator_data_);
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
//   actuator_data_["relative_pos"] = 65535;
//   logger_->info("writing anti- clockwise command");
//   harmonic_motor_->writeData(actuator_data_);
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));


}

logger_->info("EXITING");

read_thread.join();

return 0;

}
