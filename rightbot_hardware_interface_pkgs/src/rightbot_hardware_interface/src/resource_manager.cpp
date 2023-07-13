// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "hardware_interface/resource_manager.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcutils/logging_macros.h"

namespace hardware_interface
{
auto trigger_and_print_hardware_state_transition =
  [](
    auto transition, const std::string transition_name, const std::string & hardware_name,
    const lifecycle_msgs::msg::State::_id_type & target_state)
{
  RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "'%s' hardware '%s' ", transition_name.c_str(), hardware_name.c_str());

  const rclcpp_lifecycle::State new_state = transition();

  bool result = new_state.id() == target_state;

  if (result)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Successful '%s' of hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  else
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Failed to '%s' hardware '%s'", transition_name.c_str(),
      hardware_name.c_str());
  }
  return result;
};

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * actuator_interface_name = "hardware_interface::ActuatorInterface";
  static constexpr const char * sensor_interface_name = "hardware_interface::SensorInterface";
  static constexpr const char * system_interface_name = "hardware_interface::SystemInterface";

public:
  ResourceStorage()
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {
  }

  template <class HardwareT, class HardwareInterfaceT>
  void load_hardware(
    const HardwareInfo & hardware_info, pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Loading hardware '%s' ", hardware_info.name.c_str());
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT hardware(std::move(interface));
    container.emplace_back(std::move(hardware));
    // initialize static data about hardware component to reduce later calls
    HardwareComponentInfo component_info;
    component_info.name = hardware_info.name;
    component_info.type = hardware_info.type;
    component_info.class_type = hardware_info.hardware_class_type;

    hardware_info_map_.insert(std::make_pair(component_info.name, component_info));
  }

  template <class HardwareT>
  bool initialize_hardware(const HardwareInfo & hardware_info, HardwareT & hardware)
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Initialize hardware '%s' ", hardware_info.name.c_str());

    const rclcpp_lifecycle::State new_state = hardware.initialize(hardware_info);

    bool result = new_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

    if (result)
    {
      RCUTILS_LOG_INFO_NAMED(
        "resource_manager", "Successful initialization of hardware '%s'",
        hardware_info.name.c_str());
    }
    else
    {
      RCUTILS_LOG_INFO_NAMED(
        "resource_manager", "Failed to initialize hardware '%s'", hardware_info.name.c_str());
    }
    return result;
  }

  template <class HardwareT>
  bool configure_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::configure, &hardware), "configure", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    if (result)
    {
      // TODO(destogl): is it better to check here if previous state was unconfigured instead of
      // checking if each state already exists? Or we should somehow know that transition has
      // happened and only then trigger this part of the code?
      // On the other side this part of the code should never be executed in real-time critical
      // thread, so it could be also OK as it is...
      for (const auto & interface : hardware_info_map_[hardware.get_name()].state_interfaces)
      {
        // add all state interfaces to available list
        auto found_it = std::find(
          available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

        if (found_it == available_state_interfaces_.end())
        {
          available_state_interfaces_.emplace_back(interface);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager", "(hardware '%s'): '%s' state interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' state interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }

      // add command interfaces to available list
      for (const auto & interface : hardware_info_map_[hardware.get_name()].command_interfaces)
      {
        // TODO(destogl): check if interface should be available on configure
        auto found_it = std::find(
          available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

        if (found_it == available_command_interfaces_.end())
        {
          available_command_interfaces_.emplace_back(interface);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager", "(hardware '%s'): '%s' command interface added into available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' command interface already in available list."
            " This can happen due to multiple calls to 'configure'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }
    }
    return result;
  }

  template <class HardwareT>
  bool cleanup_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::cleanup, &hardware), "cleanup", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    if (result)
    {
      // remove all command interfaces from available list
      for (const auto & interface : hardware_info_map_[hardware.get_name()].command_interfaces)
      {
        auto found_it = std::find(
          available_command_interfaces_.begin(), available_command_interfaces_.end(), interface);

        if (found_it != available_command_interfaces_.end())
        {
          available_command_interfaces_.erase(found_it);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' command interface removed from available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' command interface not in available list."
            " This can happen due to multiple calls to 'cleanup'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }
      // remove all state interfaces from available list
      for (const auto & interface : hardware_info_map_[hardware.get_name()].state_interfaces)
      {
        auto found_it = std::find(
          available_state_interfaces_.begin(), available_state_interfaces_.end(), interface);

        if (found_it != available_state_interfaces_.end())
        {
          available_state_interfaces_.erase(found_it);
          RCUTILS_LOG_DEBUG_NAMED(
            "resource_manager", "(hardware '%s'): '%s' state interface removed from available list",
            hardware.get_name().c_str(), interface.c_str());
        }
        else
        {
          // TODO(destogl): do here error management if interfaces are only partially added into
          // "available" list - this should never be the case!
          RCUTILS_LOG_WARN_NAMED(
            "resource_manager",
            "(hardware '%s'): '%s' state interface not in available list. "
            "This can happen due to multiple calls to 'cleanup'",
            hardware.get_name().c_str(), interface.c_str());
        }
      }
    }
    return result;
  }

  template <class HardwareT>
  bool shutdown_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::shutdown, &hardware), "shutdown", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    if (result)
    {
      // TODO(destogl): change this - deimport all things if there is there are interfaces there
      // deimport_non_movement_command_interfaces(hardware);
      // deimport_state_interfaces(hardware);
      // use remove_command_interfaces(hardware);
    }
    return result;
  }

  template <class HardwareT>
  bool activate_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::activate, &hardware), "activate", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    if (result)
    {
      // TODO(destogl): make all command interfaces available (currently are all available)
    }

    return result;
  }

  template <class HardwareT>
  bool deactivate_hardware(HardwareT & hardware)
  {
    bool result = trigger_and_print_hardware_state_transition(
      std::bind(&HardwareT::deactivate, &hardware), "deactivate", hardware.get_name(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    if (result)
    {
      // TODO(destogl): make all command interfaces unavailable that should be present only
      // when active (currently are all available) also at inactive
    }
    return result;
  }

  template <class HardwareT>
  bool set_component_state(HardwareT & hardware, const rclcpp_lifecycle::State & target_state)
  {
    using lifecycle_msgs::msg::State;

    bool result = false;

    switch (target_state.id())
    {
      case State::PRIMARY_STATE_UNCONFIGURED:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = true;
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = cleanup_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            if (result)
            {
              result = cleanup_hardware(hardware);
            }
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_INACTIVE:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = deactivate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_ACTIVE:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = configure_hardware(hardware);
            if (result)
            {
              result = activate_hardware(hardware);
            }
            if(!result){
              remove_interfaces();

            }
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = activate_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = true;
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = false;
            RCUTILS_LOG_WARN_NAMED(
              "resource_manager", "hardware '%s' is in finalized state and can be only destroyed.",
              hardware.get_name().c_str());
            break;
        }
        break;
      case State::PRIMARY_STATE_FINALIZED:
        switch (hardware.get_state().id())
        {
          case State::PRIMARY_STATE_UNCONFIGURED:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_INACTIVE:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_ACTIVE:
            result = shutdown_hardware(hardware);
            break;
          case State::PRIMARY_STATE_FINALIZED:
            result = true;
            break;
        }
        break;
    }

    return result;
  }

  template <class HardwareT>
  void import_state_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_state_interfaces();
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto & interface : interfaces)
    {
      auto key = interface.get_name();
      state_interface_map_.emplace(std::make_pair(key, std::move(interface)));
      interface_names.push_back(key);
    }
    hardware_info_map_[hardware.get_name()].state_interfaces = interface_names;
    available_state_interfaces_.reserve(
      available_state_interfaces_.capacity() + interface_names.size());
  }

  template <class HardwareT>
  void import_command_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_command_interfaces();
    hardware_info_map_[hardware.get_name()].command_interfaces = add_command_interfaces(interfaces);
  }

  /// Adds exported command interfaces into internal storage.
  /**
   * Add command interfaces to the internal storage. Command interfaces exported from hardware or
   * chainable controllers are moved to the map with name-interface pairs, the interface names are
   * added to the claimed map and available list's size is increased to reserve storage when
   * interface change theirs status in real-time control loop.
   *
   * \param[interfaces] list of command interface to add into storage.
   * \returns list of interface names that are added into internal storage. The output is used to
   * avoid additional iterations to cache interface names, e.g., for initializing info structures.
   */
  std::vector<std::string> add_command_interfaces(std::vector<CommandInterface> & interfaces)
  {
    std::vector<std::string> interface_names;
    interface_names.reserve(interfaces.size());
    for (auto & interface : interfaces)
    {
      auto key = interface.get_name();
      command_interface_map_.emplace(std::make_pair(key, std::move(interface)));
      claimed_command_interface_map_.emplace(std::make_pair(key, false));
      interface_names.push_back(key);
    }
    available_command_interfaces_.reserve(
      available_command_interfaces_.capacity() + interface_names.size());

    return interface_names;
  }

  /// Removes command interfaces from internal storage.
  /**
   * Command interface are removed from the maps with theirs storage and their claimed status.
   *
   * \param[interface_names] list of command interface names to remove from storage.
   */
  void remove_command_interfaces(const std::vector<std::string> & interface_names)
  {
    for (const auto & interface : interface_names)
    {
      command_interface_map_.erase(interface);
      claimed_command_interface_map_.erase(interface);
    }
  }

  void remove_interfaces()
{
  for (const auto & interface : state_interface_map_)
  {
    state_interface_map_.erase(interface.first);
  }

  for (const auto & interface : command_interface_map_)
  {
    command_interface_map_.erase(interface.first);
    claimed_command_interface_map_.erase(interface.first);
  }
}

  void check_for_duplicates(const HardwareInfo & hardware_info)
  {
    // Check for identical names
    if (hardware_info_map_.find(hardware_info.name) != hardware_info_map_.end())
    {
      throw std::runtime_error(
        "Hardware name " + hardware_info.name +
        " is duplicated. Please provide a unique 'name' in the URDF.");
    }
  }

  // TODO(destogl): Propagate "false" up, if happens in initialize_hardware
  void load_and_initialize_actuator(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<Actuator, ActuatorInterface>(hardware_info, actuator_loader_, actuators_);
    initialize_hardware(hardware_info, actuators_.back());
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back());
  }

  void load_and_initialize_sensor(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<Sensor, SensorInterface>(hardware_info, sensor_loader_, sensors_);
    initialize_hardware(hardware_info, sensors_.back());
    import_state_interfaces(sensors_.back());
  }

  void load_and_initialize_system(const HardwareInfo & hardware_info)
  {
    check_for_duplicates(hardware_info);
    load_hardware<System, SystemInterface>(hardware_info, system_loader_, systems_);
    initialize_hardware(hardware_info, systems_.back());
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back());
  }

  void initialize_actuator(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
  {
    this->actuators_.emplace_back(Actuator(std::move(actuator)));
    initialize_hardware(hardware_info, actuators_.back());
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back());
  }

  void initialize_sensor(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
  {
    this->sensors_.emplace_back(Sensor(std::move(sensor)));
    initialize_hardware(hardware_info, sensors_.back());
    import_state_interfaces(sensors_.back());
  }

  void initialize_system(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
  {
    this->systems_.emplace_back(System(std::move(system)));
    initialize_hardware(hardware_info, systems_.back());
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back());
  }

  // hardware plugins
  pluginlib::ClassLoader<ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<SystemInterface> system_loader_;

  std::vector<Actuator> actuators_;
  std::vector<Sensor> sensors_;
  std::vector<System> systems_;

  std::unordered_map<std::string, HardwareComponentInfo> hardware_info_map_;

  std::unordered_map<std::string, std::vector<std::string>> controllers_reference_interfaces_map_;

  /// Storage of all available state interfaces
  std::map<std::string, StateInterface> state_interface_map_;
  /// Storage of all available command interfaces
  std::map<std::string, CommandInterface> command_interface_map_;

  /// Vectors with interfaces available to controllers (depending on hardware component state)
  std::vector<std::string> available_state_interfaces_;
  std::vector<std::string> available_command_interfaces_;

  /// List of all claimed command interfaces
  std::unordered_map<std::string, bool> claimed_command_interface_map_;
};

ResourceManager::ResourceManager() : resource_storage_(std::make_unique<ResourceStorage>()) {}

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(
  const std::string & urdf, bool validate_interfaces, bool activate_all)
: resource_storage_(std::make_unique<ResourceStorage>())
{
  load_urdf(urdf, validate_interfaces);

  if (activate_all)
  {
    for (auto const & hw_info : resource_storage_->hardware_info_map_)
    {
      using lifecycle_msgs::msg::State;
      rclcpp_lifecycle::State state(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
      set_component_state(hw_info.first, state);
    }
  }
}

void ResourceManager::load_urdf(const std::string & urdf, bool validate_interfaces)
{
  const std::string system_type = "system";
  const std::string sensor_type = "sensor";
  const std::string actuator_type = "actuator";

  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
  console_sink->set_level(spdlog::level::info);
  auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/data/logs/robot_logs/hardware_interface_logs/hardware_interface_data_logs.txt", 1024*1024*150, 5);
  rotating_sink->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks {console_sink,rotating_sink};
  auto root_logger = std::make_shared<spdlog::async_logger>("hardware_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
  root_logger->set_level(spdlog::level::debug);
  spdlog::register_logger(root_logger);

  logger_ = spdlog::get("hardware_interface")->clone("resource_manager");

  logger_->info("In Interface Initialization");
  logger_->flush();


  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  for (const auto & individual_hardware_info : hardware_info)
  {
    if (individual_hardware_info.type == actuator_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
      resource_storage_->load_and_initialize_actuator(individual_hardware_info);
    }
    if (individual_hardware_info.type == sensor_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      resource_storage_->load_and_initialize_sensor(individual_hardware_info);
    }
    if (individual_hardware_info.type == system_type)
    {
      std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
      std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
      resource_storage_->load_and_initialize_system(individual_hardware_info);
    }
  }

  // throw on missing state and command interfaces, not specified keys are being ignored
  if (validate_interfaces)
  {
    validate_storage(hardware_info);
  }
}

LoanedStateInterface ResourceManager::claim_state_interface(const std::string & key)
{
  if (!state_interface_is_available(key))
  {
    throw std::runtime_error(std::string("State interface with key '") + key + "' does not exist");
  }

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedStateInterface(resource_storage_->state_interface_map_.at(key));
}

std::vector<std::string> ResourceManager::state_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->state_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

std::vector<std::string> ResourceManager::available_state_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_state_interfaces_;
}

bool ResourceManager::state_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->state_interface_map_.find(key) !=
         resource_storage_->state_interface_map_.end();
}

bool ResourceManager::state_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_state_interfaces_.begin(),
           resource_storage_->available_state_interfaces_.end(),
           name) != resource_storage_->available_state_interfaces_.end();
}

void ResourceManager::import_controller_reference_interfaces(
  const std::string & controller_name, std::vector<CommandInterface> & interfaces)
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  auto interface_names = resource_storage_->add_command_interfaces(interfaces);
  resource_storage_->controllers_reference_interfaces_map_[controller_name] = interface_names;
}

std::vector<std::string> ResourceManager::get_controller_reference_interface_names(
  const std::string & controller_name)
{
  return resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
}

void ResourceManager::make_controller_reference_interfaces_available(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  resource_storage_->available_command_interfaces_.insert(
    resource_storage_->available_command_interfaces_.end(), interface_names.begin(),
    interface_names.end());
}

void ResourceManager::make_controller_reference_interfaces_unavailable(
  const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & interface : interface_names)
  {
    auto found_it = std::find(
      resource_storage_->available_command_interfaces_.begin(),
      resource_storage_->available_command_interfaces_.end(), interface);
    if (found_it != resource_storage_->available_command_interfaces_.end())
    {
      resource_storage_->available_command_interfaces_.erase(found_it);
      RCUTILS_LOG_DEBUG_NAMED(
        "resource_manager", "'%s' command interface removed from available list",
        interface.c_str());
    }
  }
}

void ResourceManager::remove_controller_reference_interfaces(const std::string & controller_name)
{
  auto interface_names =
    resource_storage_->controllers_reference_interfaces_map_.at(controller_name);
  resource_storage_->controllers_reference_interfaces_map_.erase(controller_name);

  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  resource_storage_->remove_command_interfaces(interface_names);
}

// CM API: Called in "update"-thread
bool ResourceManager::command_interface_is_claimed(const std::string & key) const
{
  if (!command_interface_is_available(key))
  {
    return false;
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  return resource_storage_->claimed_command_interface_map_.at(key);
}

// CM API: Called in "update"-thread
LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
{
  if (!command_interface_is_available(key))
  {
    throw std::runtime_error(std::string("Command interface with '") + key + "' does not exist");
  }

  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  if (command_interface_is_claimed(key))
  {
    throw std::runtime_error(
      std::string("Command interface with '") + key + "' is already claimed");
  }

  resource_storage_->claimed_command_interface_map_[key] = true;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return LoanedCommandInterface(
    resource_storage_->command_interface_map_.at(key),
    std::bind(&ResourceManager::release_command_interface, this, key));
}

// CM API: Called in "update"-thread
void ResourceManager::release_command_interface(const std::string & key)
{
  std::lock_guard<std::recursive_mutex> guard_claimed(claimed_command_interfaces_lock_);
  resource_storage_->claimed_command_interface_map_[key] = false;
}

std::vector<std::string> ResourceManager::command_interface_keys() const
{
  std::vector<std::string> keys;
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  for (const auto & item : resource_storage_->command_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

std::vector<std::string> ResourceManager::available_command_interfaces() const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->available_command_interfaces_;
}

bool ResourceManager::command_interface_exists(const std::string & key) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return resource_storage_->command_interface_map_.find(key) !=
         resource_storage_->command_interface_map_.end();
}

// CM API
bool ResourceManager::command_interface_is_available(const std::string & name) const
{
  std::lock_guard<std::recursive_mutex> guard(resource_interfaces_lock_);
  return std::find(
           resource_storage_->available_command_interfaces_.begin(),
           resource_storage_->available_command_interfaces_.end(),
           name) != resource_storage_->available_command_interfaces_.end();
}

size_t ResourceManager::actuator_components_size() const
{
  return resource_storage_->actuators_.size();
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensors_.size();
}

void ResourceManager::import_component(
  std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_actuator(std::move(actuator), hardware_info);
}

void ResourceManager::import_component(
  std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_sensor(std::move(sensor), hardware_info);
}

void ResourceManager::import_component(
  std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_system(std::move(system), hardware_info);
}

size_t ResourceManager::system_components_size() const
{
  return resource_storage_->systems_.size();
}
// End of "used only in tests"

std::unordered_map<std::string, HardwareComponentInfo> ResourceManager::get_components_status()
{
  for (auto & component : resource_storage_->actuators_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }
  for (auto & component : resource_storage_->systems_)
  {
    resource_storage_->hardware_info_map_[component.get_name()].state = component.get_state();
  }

  return resource_storage_->hardware_info_map_;
}

bool ResourceManager::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto interfaces_to_string = [&]()
  {
    std::stringstream ss;
    ss << "Start interfaces: " << std::endl << "[" << std::endl;
    for (const auto & start_if : start_interfaces)
    {
      ss << "  " << start_if << std::endl;
    }
    ss << "]" << std::endl;
    ss << "Stop interfaces: " << std::endl << "[" << std::endl;
    for (const auto & stop_if : stop_interfaces)
    {
      ss << "  " << stop_if << std::endl;
    }
    ss << "]" << std::endl;
    return ss.str();
  };

  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(), interfaces_to_string().c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(), interfaces_to_string().c_str());
      return false;
    }
  }
  return true;
}

bool ResourceManager::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  return true;
}

return_type ResourceManager::set_component_state(
  const std::string & component_name, rclcpp_lifecycle::State & target_state)
{
  using lifecycle_msgs::msg::State;
  using std::placeholders::_1;
  using std::placeholders::_2;

  auto found_it = resource_storage_->hardware_info_map_.find(component_name);

  if (found_it == resource_storage_->hardware_info_map_.end())
  {
    RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Hardware Component with name '%s' does not exists",
      component_name.c_str());
    return return_type::ERROR;
  }

  return_type result = return_type::OK;

  if (target_state.id() == 0)
  {
    if (target_state.label() == lifecycle_state_names::UNCONFIGURED)
    {
      target_state = rclcpp_lifecycle::State(
        State::PRIMARY_STATE_UNCONFIGURED, lifecycle_state_names::UNCONFIGURED);
    }
    if (target_state.label() == lifecycle_state_names::INACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::ACTIVE)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE);
    }
    if (target_state.label() == lifecycle_state_names::FINALIZED)
    {
      target_state =
        rclcpp_lifecycle::State(State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED);
    }
  }

  auto find_set_component_state = [&](auto action, auto & components)
  {
    auto found_component_it = std::find_if(
      components.begin(), components.end(),
      [&](const auto & component) { return component.get_name() == component_name; });

    if (found_component_it != components.end())
    {
      if (action(*found_component_it, target_state))
      {
        result = return_type::OK;
      }
      else
      {
        result = return_type::ERROR;
      }
      return true;
    }
    return false;
  };

  bool found = find_set_component_state(
    std::bind(&ResourceStorage::set_component_state<Actuator>, resource_storage_.get(), _1, _2),
    resource_storage_->actuators_);
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<Sensor>, resource_storage_.get(), _1, _2),
      resource_storage_->sensors_);
  }
  if (!found)
  {
    found = find_set_component_state(
      std::bind(&ResourceStorage::set_component_state<System>, resource_storage_.get(), _1, _2),
      resource_storage_->systems_);
  }

  return result;
}

void ResourceManager::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

  for (auto & component : resource_storage_->actuators_)
  {
    auto component_name = component.get_name();

    if(component_name == "Hardware_TruckUnloading_h_gantry_joint") {
      component.data_request();
      
    }

    if(component_name == "Hardware_TruckUnloading_base_rotation_joint") {
      component.data_request();
      
    }

    if(component_name == "Hardware_TruckUnloading_elbow_rotation_joint") {
      component.data_request();
      
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  
  for (auto & component : resource_storage_->actuators_)
  {
    component.read(time, period);
  }
  for (auto & component : resource_storage_->sensors_)
  {
    component.read(time, period);
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.read(time, period);
  }

  float camera_angle = 0.0f, base_rotation_angle = 0.0f;
  float angle_diff = 0.0f;
  static int count = 1;
  if((!camera_homing_status) && (!command_homing_sent)){
    double angle = 0.0;
    bool component_available = false;

    for (auto & component : resource_storage_->sensors_){

      auto component_name = component.get_name();

      // RCUTILS_LOG_INFO_NAMED(
      //         "resource_manager", "[camera_homing] components '%s' ", component_name.c_str());

      
      if(component_name == "TruckUnloading_absolute_encoder_sensor"){
        component_available = true;
        
        auto state_interfaces = component.export_state_interfaces();
        for (auto & current_interface : state_interfaces){

          if(current_interface.get_interface_name() == hardware_interface::HW_IF_POSITION){
            angle = current_interface.get_value();

            if(angle != 0.0){
              if(abs(angle) > 0.0087){ //angle > 0.5 degree
                RCUTILS_LOG_INFO_NAMED(
                "resource_manager", "[camera_homing] Camera angle '%f'. Sending homing command. ", angle);
                
                double angle_in_radian = (angle);
                camera_homing(angle_in_radian);
                homing_start_time = std::chrono::system_clock::now();
                command_homing_sent = true;
                // camera_homing_status = true;

              } else {
                RCUTILS_LOG_INFO_NAMED(
                "resource_manager", "[camera_homing] Camera angle '%f'. Already at home position", angle);
                camera_homing_status = true;
              }
            }
              
          }
        }
      }
    }

    if(!component_available){
      camera_homing_status = true;
      RCUTILS_LOG_ERROR_NAMED(
      "resource_manager", "[camera_homing] Component [Hardware_TruckUnloading_absolute_encoder_sensor] not available ");

    }

  }

  if(command_homing_sent){
    auto time_passed_since_homing_sent = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - homing_start_time);

    if(time_passed_since_homing_sent.count()>5000){
      camera_homing_status = true;
      command_homing_sent = false;
      RCUTILS_LOG_INFO_NAMED(
                "resource_manager", "[camera_homing] Camera homing success..");

    }

    

  }

  // if(camera_homing_status){

  //   if(count > 10) {
  //       count = 1;
  //       for (auto & component : resource_storage_->actuators_){ 

  //         bool component_available = false;
  //         auto component_name = component.get_name();

  //         if(component_name == "TruckUnloading_camera_rotation_joint") {
  //           // RCUTILS_LOG_INFO_NAMED(
  //           // "resource_manager", "[camera_align] Hardware_TruckUnloading_camera_rotation_joint ");
  //           auto state_interfaces = component.export_state_interfaces();

  //           for (auto & current_interface : state_interfaces){

  //             if(current_interface.get_interface_name() == hardware_interface::HW_IF_POSITION){
  //               camera_angle = current_interface.get_value();
  //               camera_angle = -camera_angle;
  //               // RCUTILS_LOG_INFO_NAMED(
  //               // "resource_manager", "[camera_align] Hardware_TruckUnloading_camera_rotation_joint camera angle '%f' ",camera_angle);
                
  //             }
  //           }
  //         }
  //       }
  //       for (auto & component : resource_storage_->actuators_){
  //         auto component_name = component.get_name();
  //         if(component_name == "Hardware_TruckUnloading_base_rotation_joint"){
  //           bool component_available = true;

  //           auto state_interfaces = component.export_state_interfaces();
  //           for (auto & current_interface : state_interfaces){

  //             if(current_interface.get_interface_name() == hardware_interface::HW_IF_POSITION){
  //               base_rotation_angle = current_interface.get_value();
  //               angle_diff = base_rotation_angle - camera_angle;
               
                
  //             }
  //           }
  //         }

  //      }

  //      // giving absolute angle to camera
  //      if(abs(previous_base_rotation_angle -base_rotation_angle) > 0.035){
  //                   //
  //         RCUTILS_LOG_INFO_NAMED(
  //           "resource_manager", "[camera_align] Command angle '%f', base_rotation_angle '%f', camera_rotation_angle '%f'",angle_diff,base_rotation_angle,camera_angle);

  //         // double angle_to_command_ = static_cast<double>(0.0);
  //          double angle_to_command_ = -1.0 * static_cast<double>(base_rotation_angle);
          
  //         camera_align(angle_to_command_);

  //         previous_base_rotation_angle = base_rotation_angle;

  //       }
  //   } 
  //   count++;
      
  // }


}

void ResourceManager::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.write(time, period);
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.write(time, period);
  }
}

void ResourceManager::validate_storage(
  const std::vector<hardware_interface::HardwareInfo> & hardware_info) const
{
  std::vector<std::string> missing_state_keys = {};
  std::vector<std::string> missing_command_keys = {};

  for (const auto & hardware : hardware_info)
  {
    for (const auto & joint : hardware.joints)
    {
      for (const auto & state_interface : joint.state_interfaces)
      {
        if (!state_interface_exists(joint.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(joint.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : joint.command_interfaces)
      {
        if (!command_interface_exists(joint.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(joint.name + "/" + command_interface.name);
        }
      }
    }
    for (const auto & sensor : hardware.sensors)
    {
      for (const auto & state_interface : sensor.state_interfaces)
      {
        if (!state_interface_exists(sensor.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(sensor.name + "/" + state_interface.name);
        }
      }
    }
  }

  if (!missing_state_keys.empty() || !missing_command_keys.empty())
  {
    std::string err_msg = "Wrong state or command interface configuration.\n";
    err_msg += "missing state interfaces:\n";
    for (const auto & missing_key : missing_state_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }
    err_msg += "\nmissing command interfaces:\n";
    for (const auto & missing_key : missing_command_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }

    throw std::runtime_error(err_msg);
  }
}

// Temporary method to keep old interface and reduce framework changes in PRs
void ResourceManager::activate_all_components()
{
  using lifecycle_msgs::msg::State;
  rclcpp_lifecycle::State active_state(
    State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);

  for (auto & component : resource_storage_->actuators_)
  {
    set_component_state(component.get_name(), active_state);
  }
  for (auto & component : resource_storage_->sensors_)
  {
    set_component_state(component.get_name(), active_state);
  }
  for (auto & component : resource_storage_->systems_)
  {
    set_component_state(component.get_name(), active_state);
  }
}

void ResourceManager::deactivate_all_components()
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.deactivate();
  }

}

void ResourceManager::reset_component(std::string component_name)
{
  bool component_available = false;
  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    if(current_component_ == component_name){

      RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Component '%s' reset.", current_component_.c_str());

      component.fault_reset();
      component_available = true;

      error_status = false; // reset error status
      error_monitoring_started[component_name] = false;
    }
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[reset component] Component '%s' not available ", component_name.c_str());

  }

  if(actuator_connection_break_status_[component_name] == true){
    logger_->info("[reset component] Actuator [{}] connection break status true. Resetting.", component_name);

    if(component_name == "Hardware_TruckUnloading_elbow_rotation_joint"){
      logger_->info("[reset component] Actuator [{}] Reset requries reinit. Sending reinint.", component_name);
      reinitialize_actuator(component_name);

      logger_->info("[reset component] Actuator [{}] Reset connection", component_name);

      error_monitoring_started[component_name] = false;
      actuator_connection_break_status_[component_name] = false;
      
    }else {
      logger_->info("[reset component] Actuator [{}] Reset connection", component_name);

      error_monitoring_started[component_name] = false;
      actuator_connection_break_status_[component_name] = false;

    }
  } else {
    logger_->info("[reset component] Actuator [{}] not in connection break", component_name);

  }
  
  error_status = false;

}

void ResourceManager::reinitialize_actuator(std::string component_name)
{
  bool component_available = false;
  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    if(current_component_ == component_name){

      RCUTILS_LOG_INFO_NAMED(
      "resource_manager", "Component '%s' reinitialize.", current_component_.c_str());

      component.reinitialize_actuator();
      component_available = true;
    }
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[reset component] Component '%s' not available ", component_name.c_str());

  }

}

void ResourceManager::driver_one_gpio_control(bool pump_one, bool gripper_one)
{
  bool component_available = false;

  double set_value = 0.0;
  if(pump_one && !gripper_one){
    set_value = 1.0;
  } else if (!pump_one && gripper_one){
    set_value = 2.0;
  } else if (pump_one && gripper_one) {
    set_value = 3.0;
  } else if (!pump_one && !gripper_one) {
    set_value = 0.0;
  } else {
    RCUTILS_LOG_ERROR_NAMED(
    "resource_manager", "[gripper/pump control] [Hardware_TruckUnloading_v_gantry_joint] Command not recognized");
  }

  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    //RCUTILS_LOG_INFO_NAMED(
    //"resource_manager", "[reset component] Current component '%s' ", current_component_.c_str());
    
    if(current_component_ == "Hardware_TruckUnloading_v_gantry_joint"){
      component_available = true;

      auto command_interfaces = component.export_command_interfaces();
      for (auto & current_interface : command_interfaces){

        if(current_interface.get_interface_name() == hardware_interface::HW_IF_GPIO){
          current_interface.set_value(set_value);
          RCUTILS_LOG_INFO_NAMED(
            "resource_manager", "[gripper/pump control] [Hardware_TruckUnloading_v_gantry_joint] Setting gpio interface value '%f' ", set_value);
        }
      }
    }
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[gripper/pump control] Component [Hardware_TruckUnloading_v_gantry_joint] not available ");

  }

}

void ResourceManager::driver_two_gpio_control(bool pump_two, bool gripper_two)
{
  bool component_available = false;

  double set_value = 0.0;
  if(pump_two && !gripper_two){
    set_value = 1.0;
  } else if (!pump_two && gripper_two){
    set_value = 2.0;
  } else if (pump_two && gripper_two) {
    set_value = 3.0;
  } else if (!pump_two && !gripper_two) {
    set_value = 0.0;
  } else {
    RCUTILS_LOG_ERROR_NAMED(
    "resource_manager", "[gripper/pump control] [TruckUnloading_h_gantry_joint] Command not recognized");
  }

  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    if(current_component_ == "TruckUnloading_h_gantry_joint"){
      component_available = true;

      auto command_interfaces = component.export_command_interfaces();
      for (auto & current_interface : command_interfaces){

        if(current_interface.get_interface_name() == hardware_interface::HW_IF_GPIO){
          current_interface.set_value(set_value);
          RCUTILS_LOG_INFO_NAMED(
            "resource_manager", "[gripper/pump control] [TruckUnloading_h_gantry_joint] Setting gpio interface value '%d' ", set_value);
        }
      }
    }
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[gripper/pump control] Component [TruckUnloading_h_gantry_joint] not available ");

  }

}


void ResourceManager::clear_can_buffer(){
  for (auto & component : resource_storage_->actuators_)
  {
    component.clear_can_buffer();

  }

  for (auto & component : resource_storage_->sensors_)
  {
    component.clear_can_buffer();

  }

}

std::string ResourceManager::get_zlac_driver_error(int error){

  std::string error_type;

  if(error < 0){
    error_type = "Negative error value";
    return error_type;
  }
  else if (error == 0){
    error_type = " Works properly ";
    return error_type;
  }

  if (((error & (1 << 0)) >> 0)){
    error_type = error_type + " Data flash CRC failure, Fatal FAULT, Cannot be cleared. ";
  }

  if (((error & (1 << 1)) >> 1)){
    error_type = error_type + " Amplifier internal error, Fatal FAULT, Cannot be cleared. ";
  }

  if (((error & (1 << 2)) >> 2)){
    error_type = error_type + " Short circuit. ";
  }

  if (((error & (1 << 3)) >> 3)){
    error_type = error_type + " Amplifier over temperature. ";
  }

  if (((error & (1 << 4)) >> 4)){
    error_type = error_type + " Motor over temperature. ";
  }

  if (((error & (1 << 5)) >> 5)){
    error_type = error_type + " Over voltage. ";
  }

  if (((error & (1 << 6)) >> 6)){
    error_type = error_type + " Under voltage. ";
  }

  if (((error & (1 << 7)) >> 7)){
    error_type = error_type + " Feedback fault. ";
  }

  if (((error & (1 << 8)) >> 8)){
    error_type = error_type + " Phasing error. ";
  }

  if (((error & (1 << 9)) >> 9)){
    error_type = error_type + " Tracking error. ";
  }

  if (((error & (1 << 10)) >> 10)){
    error_type = error_type + " Over Current, ";
  }

  if (((error & (1 << 11)) >> 11)){
    error_type = error_type + " FPGA failure. ";
  }

  if (((error & (1 << 12)) >> 12)){
    error_type = error_type + " Command input lost. ";
  }

  if (((error & (1 << 13)) >> 13)){
    error_type = error_type + " FPGA failure 2. ";
  }

  if (((error & (1 << 14)) >> 14)){
    error_type = error_type + " Safety circuit fault. ";
  }

  if (((error & (1 << 15)) >> 15)){
    error_type = error_type + " Unable to control current. ";
  }

  return error_type;

}

std::string ResourceManager::get_harmonic_driver_error(int error){

  std::string error_type;

  if(error < 0){
    error_type = "Negative error value";
    return error_type;
  }

  if (error == 0){
    error_type = " Works properly ";
  } else if (error == 8724){
    error_type = " Overcurrent ";
  } else if (error == 8784){
    error_type = " Electrical three-phase current and overlimit ";
  } else if (error == 9025){
    error_type = " U phase overflows ";
  } else if (error == 9026){
    error_type = " V phase overflows ";
  } else if (error == 9027){
    error_type = " W phase overflows ";
  } else if (error == 12816){
    error_type = " Over voltage ";
  } else if (error == 12832){
    error_type = " Under voltage ";
  } else if (error == 16656){
    error_type = " Power component temperature is too high ";
  } else if (error == 28961){
    error_type = " Motor blocking ";
  } else if (error == 29453){
    error_type = " Battery alarm error ";
  } else if (error == 29455){
    error_type = " Battery low voltage ";
  } else if (error == 29457){
    error_type = " Electromechanical end position error of sampling exceeds the limit";
  } else if (error == 29461){
    error_type = " Sampling error of the negative load terminal exceeds the limit";
  } else if (error == 29460){
    error_type = " Power - off state signal detected";
  } else if (error == 29520){
    error_type = " Motor end encoder type is not supported";
  } else if (error == 29556){
    error_type = " Multi-turn position error";
  } else if (error == 33792){
    error_type = " Offset error out of limit system value";
  } else if (error == 34048){
    error_type = " Velocity error exceeds limit value";
  } else if (error == 33793){
    error_type = " High motor speed";
  } else if (error == 40960){
    error_type = " Main station drops the line or exits the OP to a non-OP State";
  } else if (error == 61444){
    error_type = " EtherCAT initialization error";
  } else if (error == 61445){
    error_type = " STO function is activated";
  } else if (error == 61146){
    error_type = " Multiple turn count error";
  } else {
    error_type = " Error code unrecognized ";
  }  

  return error_type;  
  
}

void ResourceManager::get_error_data(ComponentErrorData *error_data_, bool *system_error_status){

  int components_number = resource_storage_->actuators_.size();

  error_data_->component_name.clear();
  error_data_->status.clear();
  error_data_->error_register.clear();
  error_data_->error_type.clear();

  for (auto & component : resource_storage_->actuators_)
  {
    std::string component_name;
    int status;
    int error_register;
    std::string error_type = "error";

    int node_guard_error;

    component_name = component.get_name();
    auto state_interfaces = component.export_state_interfaces();

    for (auto & state_interface : state_interfaces){

      // monitoring node guarding
      if(state_interface.get_interface_name() == hardware_interface::HW_IF_NODE_GUARD_ERROR){
        node_guard_error = static_cast<int>(state_interface.get_value());
        logger_->debug("[node_guarding][{}] response [{}]", component_name, node_guard_error);
        if((node_guard_error == 0) && (!actuator_connection_break_status_[component_name]) ){
          auto time_passed_response_received_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - actuator_response_time_[component_name]);
          // logger_->debug("[{}] - Received response ",x.first);
          
          if ((time_passed_response_received_.count() > logging_interval) && error_monitoring_started[component_name]){

              logger_->debug("[{}] - Interval between responses > {} us: {} milliseconds", component_name, logging_interval, time_passed_response_received_.count());
              
          }
          actuator_response_time_[component_name] = std::chrono::system_clock::now();
          error_monitoring_started[component_name] = true;
        }
        

        auto time_passed_response_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - actuator_response_time_[component_name]);
        
        if((time_passed_response_.count()>connection_break_interval) && error_monitoring_started[component_name] && (!actuator_connection_break_status_[component_name])){
          logger_->error("[{}] - Connection break ", component_name);
          logger_->info("[{}] - Interval between responses > {} us: {} milliseconds", component_name, connection_break_interval, time_passed_response_.count());
          actuator_connection_break_status_[component_name] = true;
          
          error_status = error_status && true;
        }
      }
      
      // monitoring fault
      if(state_interface.get_interface_name() == hardware_interface::HW_IF_STATUS){
        status = static_cast<int>(state_interface.get_value());
      }

      if(state_interface.get_interface_name() == hardware_interface::HW_IF_LATCHED_FAULT){
        error_register = static_cast<int>(state_interface.get_value());
        error_type = get_zlac_driver_error(error_register);
      }

      if(state_interface.get_interface_name() == hardware_interface::HW_IF_ERROR_CODE){
        error_register = static_cast<int>(state_interface.get_value());
        error_type = get_harmonic_driver_error(error_register);
      }

    }

    if((error_register != 0.0) && (actuator_connection_break_status_[component_name] == true)){
      error_type = error_type + " Connection break ";
      error_status = error_status && true;
      // logger_->error("[{}] - test log [{}] ", component_name, error_type);

    } else if((error_register == 0.0) && (actuator_connection_break_status_[component_name] == true)) {
      error_type = " Connection break ";
      error_register = 1;
      // logger_->error("[{}] - test log [{}] ", component_name, error_type);
      
    } else {
    }

    error_data_->component_name.push_back(component_name);
    error_data_->status.push_back(status);
    error_data_->error_register.push_back(error_register);
    error_data_->error_type.push_back(error_type);

    *system_error_status = error_status;

  }

}

void ResourceManager::camera_homing(double &homing_angle){

  // get absolute encoder data
  // say data is 10 degree

  // double homing_angle = 0.175;
  RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[camera_homing] angle %f ", homing_angle);

  bool component_available = false;

  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    if(current_component_ == "TruckUnloading_camera_rotation_joint"){
      component_available = true;

      component.homing_execution(homing_angle);
    }
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[camera_homing] Component [TruckUnloading_camera_rotation_joint] not available ");

  }

}

bool ResourceManager::camera_align_service_handle(double &angle){

  bool alignment_done = false;

  RCUTILS_LOG_INFO_NAMED("resource_manager", "[camera_align] Camera sending angle command [%f].", angle);
  camera_align(angle);

  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  std::chrono::system_clock::time_point camera_align_start_time = std::chrono::system_clock::now();      
  auto time_passed_camera_align_started = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - camera_align_start_time);
  hardware_interface::StateInterface *camera_joint_status_interface;
  bool component_available = false;

  for (auto & component : resource_storage_->actuators_)
  {
    auto component_name = component.get_name();

    if(component_name == "TruckUnloading_camera_rotation_joint") { // request for one interface

      auto state_interfaces = component.export_state_interfaces();
      component_available = true;

      for (auto & current_interface : state_interfaces){

        if(current_interface.get_interface_name() == hardware_interface::HW_IF_STATUS){
          camera_joint_status_interface = &current_interface;
        }
      }
    }
  }

  while((time_passed_camera_align_started.count()<5000) && (alignment_done == false)){

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double status = camera_joint_status_interface->get_value();

    RCUTILS_LOG_INFO_NAMED("resource_manager", "[camera_align] status value '%f'",status);

    int status_value = static_cast<int>(status);

    int in_motion_bit = ((status_value & (1 << 14)) >> 14);
    int target_reach_bit = ((status_value & (1 << 10)) >> 10);

    if(!in_motion_bit && target_reach_bit){
      alignment_done = true;
      RCUTILS_LOG_INFO_NAMED("resource_manager", "[camera_align] Camera align done.");
      
    }
    else{
      // RCUTILS_LOG_INFO_NAMED("resource_manager", "[camera_align] Camera alignment fail.");

    }

    time_passed_camera_align_started = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - camera_align_start_time);
  }

  if(!alignment_done){
    RCUTILS_LOG_INFO_NAMED("resource_manager", "[camera_align] Camera alignment fail.");
  }

  return alignment_done;

}

void ResourceManager::camera_align(double &align_angle){

  // RCUTILS_LOG_INFO_NAMED(
  //   "resource_manager", "[camera_align] angle %f ", align_angle);

  bool component_available = false;

  for (auto & component : resource_storage_->actuators_)
  {
    std::string current_component_ = component.get_name();

    if(current_component_ == "TruckUnloading_camera_rotation_joint"){
      component_available = true;

      auto command_interfaces = component.export_command_interfaces();
      for (auto & current_interface : command_interfaces){

        if(current_interface.get_interface_name() == hardware_interface::HW_IF_POSITION){
          double angle_to_command = static_cast<double>(align_angle);
          current_interface.set_value(angle_to_command);
          RCUTILS_LOG_INFO_NAMED(
            "resource_manager", "[camera_align] [TruckUnloading_camera_rotation_joint] align angle '%f' ", angle_to_command);
        }
      }
    }

    
  }

  if(!component_available){
    RCUTILS_LOG_INFO_NAMED(
    "resource_manager", "[camera_align] Component [TruckUnloading_camera_rotation_joint] not available ");

  }

}

}  // namespace hardware_interface
