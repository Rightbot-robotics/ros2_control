#ifndef MOTOR_INTERFACE_H_
#define MOTOR_INTERFACE_H_

#include <motor/motor_params.hpp>
#include <motor/motor.hpp>
// #include <motor_controls.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <spdlog/spdlog.h>

using std::placeholders::_1;

class MotorInterface : public rclcpp::Node {
private:

public:

    MotorInterface();

    ~MotorInterface();

    std::shared_ptr<MotorInterface> MotorInterfaceSPtr;
    std::unique_ptr<MotorInterface> MotorInterfaceUPtr;

    Sockets::SocketsSPtr left_front_motor_sockets_;
    Sockets::SocketsSPtr right_front_motor_sockets_;
    // Sockets::SocketsSPtr left_rear_motor_sockets_ ;
    // Sockets::SocketsSPtr right_rear_motor_sockets_ ;

    Motor::MotorSPtr left_front_motor_;
    Motor::MotorSPtr right_front_motor_;
    // Motor::MotorSPtr left_rear_motor_ ;
    // Motor::MotorSPtr right_rear_motor_ ;

    MotorFeedback::feedback_s feedback_s_l_m_;
    std::shared_ptr<spdlog::logger> logger_;


    // Read loop
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr data_request_timer_;

    void requestMotorData();

    void readMotorData();

    bool initialization_done = false;

};

#endif 
