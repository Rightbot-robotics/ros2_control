//
// Created by amrapali on 1/19/23.
//

#ifndef HARMONIC_MOTOR_ACTUATOR_SOCKETS_HPP
#define HARMONIC_MOTOR_ACTUATOR_SOCKETS_HPP

#include <iostream>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <socketcan_pkg/printd.h>
#include <socketcan_pkg/socketcan.h>
#include <canopen_pkg/canopen.h>
#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

#include <harmonic_motor_actuator/harmonic_motor_actuator_params.hpp>

class HarmonicMotorActuatorSockets {

public:

    typedef std::shared_ptr<HarmonicMotorActuatorSockets> HarmonicMotorActuatorSocketsSPtr;
    typedef std::unique_ptr<HarmonicMotorActuatorSockets> HarmonicMotorActuatorSocketsUPtr;

    HarmonicMotorActuatorSockets(int motor_id, std::string motor_name);

    ~HarmonicMotorActuatorSockets();

    bool createSockets(int motor_id);

    int motor_pdo_fd;    //!< Process CAN-connection.
    int motor_cfg_fd;    //!< Configuration CAN-connection.
    int motor_sync_fd;   //!< Sync CAN-connection.

    int motor_status_pdo_fd;
    int motor_vel_pdo_fd;
    int motor_enc_pdo_fd;
    int motor_system_status_pdo_fd;

    int nmt_motor_cfg_fd;

    std::string motor_name_;
    int motor_id_;
    std::string can_network_;

private:
    std::shared_ptr<spdlog::logger> logger_;
};


#endif //HARMONIC_MOTOR_ACTUATOR_SOCKETS_HPP
