#ifndef BARCODE_SOCKETS_HPP_
#define BARCODE_SOCKETS_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#include<canopen_pkg/canopen.h>

#define SOCKETS_ERROR   (-1)

class BarcodeSockets {

private:

    std::shared_ptr<spdlog::logger> logger_;

public:
    BarcodeSockets(int barcode_id, int barcode_base_id);

    ~BarcodeSockets();

    bool createBarcodeSockets(int barcode_id, int barcode_base_id);

    typedef std::shared_ptr<BarcodeSockets> BarcodeSocketsSPtr;
    typedef std::unique_ptr<BarcodeSockets> BarcodeSocketsUPtr;

    int gls_one_pdo_fd;    //!< Process CAN-connection.
    int gls_two_pdo_fd;
    int gls_three_pdo_fd;
    int gls_four_pdo_fd;
    int gls_five_pdo_fd;
    int gls_six_pdo_fd;
    int gls_seven_pdo_fd;

    int gls_cfg_fd;    //!< Configuration CAN-connection.
    int gls_sync_fd;  //!< Sync CAN-connection.

};

#endif