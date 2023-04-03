cmake_minimum_required(VERSION 3.5)
project(motor)


if (UNIX)
    list(APPEND serial_SRCS RightbotDrivers/serial/impl/unix.cc)
    list(APPEND serial_SRCS RightbotDrivers/serial/impl/list_ports/list_ports_linux.cc)
endif ()

#if (NOT TARGET spdlog)
#    # Stand-alone build
#    find_package(spdlog REQUIRED)
#endif ()


## Add serial library
include_directories(
        include
        ../json_reader/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../spdlog/include
)

install(DIRECTORY include/
        DESTINATION include/
        )

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -fPIC -Wextra -Wpedantic)
endif ()


find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(canopen_pkg REQUIRED)
find_package(socketcan_pkg REQUIRED)


add_library(${PROJECT_NAME} SHARED
        src/sockets.cpp
        src/motor.cpp
        src/motor_controls.cpp
        src/motor_feedback.cpp
        )

target_link_libraries(${PROJECT_NAME} spdlog)
ament_target_dependencies(${PROJECT_NAME} rclcpp socketcan_pkg canopen_pkg)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include

)


ament_package()

