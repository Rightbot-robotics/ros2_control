// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_
#define HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_

namespace hardware_interface
{
/// Constant defining position interface
constexpr char HW_IF_POSITION[] = "position";
/// Constant defining velocity interface
constexpr char HW_IF_VELOCITY[] = "velocity";
/// Constant defining acceleration interface
constexpr char HW_IF_ACCELERATION[] = "acceleration";
/// Constant defining effort interface
constexpr char HW_IF_EFFORT[] = "effort";

constexpr char HW_IF_MAX_VELOCITY[] = "max_velocity";

constexpr char HW_IF_BATTERY_VOLTAGE[] = "battery_voltage";

constexpr char HW_IF_STATUS[] = "status";

constexpr char HW_IF_MANUFACTURER_REGISTER[] = "manufacturer_register";

constexpr char HW_IF_LATCHED_FAULT[] = "latched_fault";

constexpr char HW_IF_NODE_GUARD_ERROR[] = "node_guard_error";

constexpr char HW_IF_ERROR_CODE[] = "error_code";

constexpr char HW_IF_INPUT_STATES[] = "input_states";

constexpr char HW_IF_ACTUAL_MOTOR_CURRENT[] = "actual_motor_current";

constexpr char HW_IF_CONTROL_STATE[] = "control_state";



}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__HARDWARE_INTERFACE_TYPE_VALUES_HPP_
