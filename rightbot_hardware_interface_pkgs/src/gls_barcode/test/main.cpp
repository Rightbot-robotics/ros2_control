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

#include <sockets.hpp>
#include <gls_barcode.hpp>

//###### Motor #######
#include<socketcan_pkg/printd.h>
#include<socketcan_pkg/socketcan.h>
#include<canopen_pkg/canopen.h>
#include<canopen_pkg/NMT.h>
#include<canopen_pkg/PDO.h>
#include<canopen_pkg/SDO.h>

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

// ############################ Namespace ##########################

std::shared_ptr<spdlog::logger> logger_;

int main() {


}
