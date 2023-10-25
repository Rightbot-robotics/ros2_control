cmake_minimum_required(VERSION 3.5)
project(test)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

#if (NOT TARGET spdlog)
#    # Stand-alone build
#    find_package(spdlog REQUIRED)
#endif ()

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}/../json_reader/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../spdlog/include
)

install(DIRECTORY include/
        DESTINATION include/
        )


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)

find_package(rclcpp REQUIRED)
# find_package(pluginlib)
find_package(hardware_interface)


find_package(ros2_control_test_assets REQUIRED)



add_executable(${PROJECT_NAME}_test
        test/main.cpp)

target_link_libraries(${PROJECT_NAME}_test

        pthread
        jsoncpp

        spdlog

        )

ament_target_dependencies(${PROJECT_NAME}_test
        
        ros2_control_test_assets
        # pluginlib
        hardware_interface
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