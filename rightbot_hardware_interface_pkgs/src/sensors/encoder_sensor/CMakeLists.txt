cmake_minimum_required(VERSION 3.8)
project(encoder_sensor)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../json_reader/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../spdlog/include
)

install(DIRECTORY include/
        DESTINATION include/
        )


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(sensor REQUIRED)
find_package(motor REQUIRED)
find_package(canopen_pkg REQUIRED)
find_package(socketcan_pkg REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/encoder_sensor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../json_reader/src/json_read.cpp
        )

target_link_libraries(${PROJECT_NAME}
        pthread
        jsoncpp
        )

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME}
        sensor
        motor
        canopen_pkg
        socketcan_pkg
        )

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include

)

add_executable(${PROJECT_NAME}_test
        test/main.cpp)

target_link_libraries(${PROJECT_NAME}_test

        ${PROJECT_NAME}
        pthread
        jsoncpp
        )
ament_target_dependencies(${PROJECT_NAME}_test
        sensor
        motor
        canopen_pkg
        socketcan_pkg
        )

install(
        TARGETS ${PROJECT_NAME}_test
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}

)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()