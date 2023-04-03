#ifndef SOCKETS_HPP_
#define SOCKETS_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#include<canopen_pkg/canopen.h>

#define SOCKETS_ERROR   (-1)

class Sockets {

private:

    std::shared_ptr<spdlog::logger> logger_;

public:
    Sockets(int motor_id, std::string motor_name);

    ~Sockets();

    bool createSockets(int motor_id);

    typedef std::shared_ptr<Sockets> SocketsSPtr;
    typedef std::unique_ptr<Sockets> SocketsUPtr;

    int motor_pdo_fd;    //!< Process CAN-connection.
    int motor_cfg_fd;    //!< Configuration CAN-connection.
    int motor_sync_fd;  //!< Sync CAN-connection.

    int nmt_motor_cfg_fd;

    int motor_status_pdo_fd;
    int motor_vel_pdo_fd;
    int motor_enc_pdo_fd;
    int motor_system_status_pdo_fd;

    int motor_can_id;
    std::string motor_name_;


};

#endif
