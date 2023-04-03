#ifndef SOCKETS_HPP_
#define SOCKETS_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#define SOCKETS_ERROR   (-1)

class SocketcanInterface{

    private:
    std::string motor_name;
    int motor_id;
    std::shared_ptr<spdlog::logger> logger_;

    public:
    SocketcanInterface();
    ~SocketcanInterface();

    typedef std::shared_ptr<SocketcanInterface> SocketcanInterfaceSPtr;
    typedef std::unique_ptr<SocketcanInterface> SocketcanInterfaceUPtr;
    
};

#endif