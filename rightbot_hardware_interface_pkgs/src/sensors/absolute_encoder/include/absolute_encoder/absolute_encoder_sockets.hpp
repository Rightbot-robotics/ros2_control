#ifndef ABSOLUTE_ENCODER_SOCKETS_HPP_
#define ABSOLUTE_ENCODER_SOCKETS_HPP_

#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include "absolute_encoder/absolute_encoder_params.hpp"

#include <socketcan_pkg/printd.h>
#include <socketcan_pkg/socketcan.h>
#include <canopen_pkg/canopen.h>
#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

class AbsoluteEncoderSockets {

private:

    std::shared_ptr<spdlog::logger> logger_;

public:
    AbsoluteEncoderSockets(int sensor_id);

    ~AbsoluteEncoderSockets();

    bool createSockets(int sensor_id);

    typedef std::shared_ptr<AbsoluteEncoderSockets> AbsoluteEncoderSocketsSPtr;
    typedef std::unique_ptr<AbsoluteEncoderSockets> AbsoluteEncoderSocketsUPtr;

    int abs_pdo_fd;    //!< Process CAN-connection.

    int abs_cfg_fd;    //!< Configuration CAN-connection.
    int abs_sync_fd;  //!< Sync CAN-connection.

};

#endif