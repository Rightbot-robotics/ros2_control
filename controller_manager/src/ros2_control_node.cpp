// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

struct TimeData {
  rclcpp::Time time;
  std::chrono::microseconds read_period;
  std::chrono::microseconds update_period;
  std::chrono::microseconds write_period;
  std::chrono::microseconds total_operation_period;
};

using namespace std::chrono_literals;

const bool display_normal_time_data = false;
const std::chrono::microseconds normal_time_log_period = 5s;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());

  std::thread cm_thread(
    [cm]()
    {

      std::chrono::microseconds read_time;
      std::chrono::microseconds update_time;
      std::chrono::microseconds write_time;
      std::chrono::microseconds total_operation_time;
      std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
      std::chrono::time_point<std::chrono::high_resolution_clock> read_end_time;
      std::chrono::time_point<std::chrono::high_resolution_clock> update_end_time;
      std::chrono::time_point<std::chrono::high_resolution_clock> write_end_time;
      std::string time_log_string;

      size_t time_logging_window = 10;
      size_t data_length = 0;
      std::vector<TimeData> time_data;
      time_data.reserve(time_logging_window);
      for(size_t i=0; i<time_logging_window; i++) {
        time_data.push_back({rclcpp::Time(0), 0ms, 0ms, 0ms, 0ms});
      }


      if (realtime_tools::has_realtime_kernel())
      {
        if (!realtime_tools::configure_sched_fifo(kSchedPriority))
        {
          RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy");
        }
      }
      else
      {
        RCLCPP_INFO(cm->get_logger(), "RT kernel is recommended for better performance");
      }

      // for calculating sleep time
      auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
      auto const cm_now = std::chrono::nanoseconds(cm->now().nanoseconds());
      std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        next_iteration_time{cm_now};

      // for calculating the measured period of the loop
      rclcpp::Time previous_time = cm->now();
      rclcpp::Time last_normal_time_log = cm->now();

      while (rclcpp::ok())
      {
        // calculate measured period
        auto const current_time = cm->now();
        auto const measured_period = current_time - previous_time;
        previous_time = current_time;

        // execute update loop
        start_time = std::chrono::high_resolution_clock::now(); 
        cm->read(cm->now(), measured_period);
        read_end_time = std::chrono::high_resolution_clock::now();
        cm->update(cm->now(), measured_period);
        update_end_time = std::chrono::high_resolution_clock::now();
        cm->write(cm->now(), measured_period);
        write_end_time = std::chrono::high_resolution_clock::now();

        read_time = std::chrono::duration_cast<std::chrono::microseconds>(read_end_time - start_time);
        update_time = std::chrono::duration_cast<std::chrono::microseconds>(update_end_time - read_end_time);
        write_time = std::chrono::duration_cast<std::chrono::microseconds>(write_end_time - update_end_time);
        total_operation_time = std::chrono::duration_cast<std::chrono::microseconds>(write_end_time - start_time);

        if(total_operation_time > period || display_normal_time_data) {
          time_data[data_length].time = current_time;
          time_data[data_length].read_period = read_time;
          time_data[data_length].update_period = update_time;
          time_data[data_length].write_period = write_time;
          time_data[data_length].total_operation_period = total_operation_time;
          data_length++;
          if(data_length == time_logging_window) {
            data_length = 0;
            if(!display_normal_time_data ||
            (display_normal_time_data && (cm->now() - last_normal_time_log) > normal_time_log_period)) {
              last_normal_time_log = cm->now();
              time_log_string = "Time taken logs:\n";
              time_log_string += "Timestamp\t\t\tRead(us)\tUpdate(us)\tWrite(us)\tTotal(us)\n";
              for(size_t i=0; i<time_logging_window; i++) {
                time_log_string += (std::to_string(time_data[i].time.seconds()) + "\t\t");
                time_log_string += (std::to_string(time_data[i].read_period.count()) + "\t\t");
                time_log_string += (std::to_string(time_data[i].update_period.count()) + "\t\t");
                time_log_string += (std::to_string(time_data[i].write_period.count()) + "\t\t");
                time_log_string += (std::to_string(time_data[i].total_operation_period.count()) + "\n");
              }
              RCLCPP_INFO(cm->get_logger(), time_log_string.c_str());
            }
          }
        }

        // wait until we hit the end of the period
        next_iteration_time += period;
        std::this_thread::sleep_until(next_iteration_time);
      }
    });

  executor->add_node(cm);
  executor->spin();

  cm->exit();
  RCLCPP_INFO(cm->get_logger(), "Controller manager exited");
  
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}
