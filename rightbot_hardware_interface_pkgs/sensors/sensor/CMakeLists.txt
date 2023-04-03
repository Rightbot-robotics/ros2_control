cmake_minimum_required(VERSION 3.8)
project(sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../json_reader/include
)

install(DIRECTORY include/
  DESTINATION include/
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
            src/sensor.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../../json_reader/src/json_read.cpp
)

target_link_libraries(${PROJECT_NAME} 
                      pthread
                      jsoncpp
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
   
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
