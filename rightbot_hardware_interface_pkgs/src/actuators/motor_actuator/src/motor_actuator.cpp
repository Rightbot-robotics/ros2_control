#include <motor_actuator/motor_actuator.hpp>
#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(MotorActuator, hardware_interface::ActuatorInterface)

using namespace std;


MotorActuator::MotorActuator() {

}

MotorActuator::~MotorActuator() = default;

CallbackReturn MotorActuator::on_init(const hardware_interface::HardwareInfo & info){
    // We hardcode the info
    logger_ = spdlog::get("hardware_interface")->clone("motor_actuator");
   
    logger_->info("Motor Actuator Init");
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    // motor_id_ = stoi(info_.hardware_parameters["motor_id"]);
    // motor_name_ = stoi(info_.hardware_parameters["motor_name"]);
    // axis_ = stoi(info_.hardware_parameters["motor_axis"]);

    motor_id_ = 12;
    motor_name_ = "left_wheel";
    axis_ = 1;

    // can only control one joint
    // if (info_.joints.size() != 1)
    // {
    //   return CallbackReturn::ERROR;
    // }

    // can only control in position 
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    
    if (command_interfaces.size() != 5)
    {
        logger_->error("[{}] - Incorrect command interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    

    for (const auto & command_interface : command_interfaces)
    {
        if (
            (command_interface.name != hardware_interface::HW_IF_POSITION) &&
            (command_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (command_interface.name != hardware_interface::HW_IF_MAX_VELOCITY) &&
            (command_interface.name != hardware_interface::HW_IF_ACCELERATION) &&
            (command_interface.name != hardware_interface::HW_IF_DECELERATION)
        )
       {
            
            return CallbackReturn::ERROR;
       }

    }

    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() != 7)
    {
        logger_->error("[{}] - Incorrect state interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
        if (
            (state_interface.name != hardware_interface::HW_IF_STATUS) &&
            (state_interface.name != hardware_interface::HW_IF_BATTERY_VOLTAGE) &&
            (state_interface.name != hardware_interface::HW_IF_POSITION) &&
            (state_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (state_interface.name != hardware_interface::HW_IF_MANUFACTURER_REGISTER) &&
            (state_interface.name != hardware_interface::HW_IF_LATCHED_FAULT) &&
            (state_interface.name != hardware_interface::HW_IF_NODE_GUARD_ERROR)
            )
       {
            
            return CallbackReturn::ERROR;
       }

    }
    // fprintf(stderr, "TestSingleJointActuator configured successfully.\n");

    logger_->info("[{}] - Intialiazation successful", motor_name_);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn MotorActuator::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    motor_sockets_ = std::make_shared<Sockets>(motor_id_, motor_name_);
    motor_ = std::make_shared<Motor>(motor_sockets_);
    motor_controls_ = std::make_shared<MotorControls>(motor_sockets_);
    motor_->motor_init(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    

    Json::Value config_data;
    config_data["motor_id"] = motor_id_;
    config_data["motor_name"] = motor_name_;
    config_data["motor_axis"] = axis_;

    encoder_sensor = std::make_shared<EncoderSensor>();
    encoder_sensor->initialize(config_data, motor_sockets_);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn MotorActuator::on_activate(const rclcpp_lifecycle::State & previous_state){

    logger_->info("Motor Enable action for: [{}]",motor_name_);
    motor_->motor_enable(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

CallbackReturn MotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    logger_->info("Motor Disable action for: [{}]",motor_name_);
    motor_->motor_disable(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


}

std::vector<hardware_interface::StateInterface> MotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &status_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &battery_voltage_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &manufacturer_register_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &latched_fault_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &node_guard_err_state_));

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> MotorActuator::export_command_interfaces(){
    // We can command in velocity
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &max_velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &acceleration_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &deceleration_command_));

    return command_interfaces;
}

hardware_interface::return_type MotorActuator::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    Json::Value sensor_data;
    encoder_sensor->getData(sensor_data);

    if(sensor_data["read_status"].asBool() == false){
        return hardware_interface::return_type::ERROR;
    }
    
    status_state_ = sensor_data["status"].asInt();
    battery_voltage_state_ = sensor_data["battery_voltage"].asDouble();

    position_state_ = sensor_data["counts"].asInt();
    velocity_state_ = sensor_data["counts"].asDouble();

    manufacturer_register_state_ = sensor_data["manufacturer_register"].asInt();

    latched_fault_state_ = sensor_data["latched_fault"].asInt();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    // call write functions for velocity command, accel, decel, max vel TO DO

    motor_controls_->set_relative_position(motor_id_, axis_, static_cast<uint32_t>( position_command_));

    

    return hardware_interface::return_type::OK;
}

CallbackReturn MotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

}

CallbackReturn MotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

}


void MotorActuator::requestData(){
    motor_controls_->motor_request();

}


bool
MotorActuator::motorCommand(int motor_id, std::string command_type, MotorControls::position_cmd_t position_cmd_element,
                            MotorControls::velocity_cmd_t velocity_cmd_element) {
    motor_controls_->motor_command(motor_id, axis_, command_type, position_cmd_element, velocity_cmd_element);
}

MotorControls::position_cmd_t
MotorActuator::setPosition(double timeout, double relative_pos, double max_vel, double accel, double decel) {
    MotorControls::position_cmd_t position_cmd_element_;
    position_cmd_element_.timeout = timeout;
    position_cmd_element_.relative_pos = relative_pos;
    position_cmd_element_.max_vel = max_vel;
    position_cmd_element_.accel = accel;
    position_cmd_element_.decel = decel;

    return position_cmd_element_;
}

MotorControls::velocity_cmd_t
MotorActuator::setVelocity(double timeout, double velocity, double max_vel, double accel, double decel) {
    MotorControls::velocity_cmd_t velocity_cmd_element_;
    velocity_cmd_element_.timeout = timeout;
    velocity_cmd_element_.velocity = velocity;
    velocity_cmd_element_.max_vel = max_vel;
    velocity_cmd_element_.accel = accel;
    velocity_cmd_element_.decel = decel;

    return velocity_cmd_element_;
}

void MotorActuator::sendNodeGuardingRequest(){

    motor_controls_->nodeGuardingRequest();

}

void MotorActuator::changeActuatorControlMode(Json::Value &actuator_control_mode){

    if (actuator_control_mode["action"].asString() == "change_control_mode"){
        if (actuator_control_mode["control_mode"].asString() == "motor_reset"){
            logger_->info("Fault reset for : [{}]",motor_name_);
            motor_->motor_reset(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_quick_stop"){
            logger_->info("Quick stop for : [{}]",motor_name_);
            motor_->motor_quick_stop(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_disable"){
            logger_->info("Disable action for : [{}]",motor_name_);
            motor_->motor_disable(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_enable"){
            logger_->info("Enable action for : [{}]",motor_name_);
            motor_->motor_enable(motor_id_);
            
        }
        else{
            logger_->info("Control Mode not recognized for : [{}]",motor_name_);
        }

    }
    else{
        logger_->info("Control Mode Action not recognized for : [{}]",motor_name_);
    }

}

void MotorActuator::writeData(Json::Value &actuator_data) {

    actuator_data_["timeout"] = actuator_data["timeout"];
    actuator_data_["mode"] = actuator_data["mode"];
    actuator_data_["velocity"] = actuator_data["velocity"];
    actuator_data_["relative_pos"] = actuator_data["relative_pos"];
    actuator_data_["max_vel"] = actuator_data["max_vel"];
    actuator_data_["accel"] = actuator_data["accel"];
    actuator_data_["decel"] = actuator_data["decel"];


    if (actuator_data_["mode"].asString() == "velocity") {
        position_cmd_received_ = {0};
        velocity_cmd_received_.timeout = actuator_data_["timeout"].asDouble();
        velocity_cmd_received_.velocity = actuator_data_["velocity"].asDouble();
        velocity_cmd_received_.max_vel = actuator_data_["max_vel"].asDouble();
        velocity_cmd_received_.accel = actuator_data_["accel"].asDouble();
        velocity_cmd_received_.decel = actuator_data_["decel"].asDouble();
        motor_controls_->motor_command(motor_id_,axis_, "velocity", position_cmd_received_, velocity_cmd_received_);

    }
    else if (actuator_data_["mode"].asString() == "position") {
        velocity_cmd_received_ = {0};
        position_cmd_received_.timeout = actuator_data_["timeout"].asDouble();
        position_cmd_received_.relative_pos = actuator_data_["relative_pos"].asDouble();
        position_cmd_received_.max_vel = actuator_data_["max_vel"].asDouble();
        position_cmd_received_.accel = actuator_data_["accel"].asDouble();
        position_cmd_received_.decel = actuator_data_["decel"].asDouble();
        motor_controls_->motor_command(motor_id_, axis_, "position", position_cmd_received_, velocity_cmd_received_);

    }
    else if (actuator_data_["mode"].asString() == "position_immediate") {
        velocity_cmd_received_ = {0};
        position_cmd_received_.timeout = actuator_data_["timeout"].asDouble();
        position_cmd_received_.relative_pos = actuator_data_["relative_pos"].asDouble();
        position_cmd_received_.max_vel = actuator_data_["max_vel"].asDouble();
        position_cmd_received_.accel = actuator_data_["accel"].asDouble();
        position_cmd_received_.decel = actuator_data_["decel"].asDouble();
        motor_controls_->motor_command(motor_id_, axis_, "position_immediate", position_cmd_received_, velocity_cmd_received_);

    }
    else{
        logger_->info("'Write Data mode not recognized for motor [{}]",motor_sockets_->motor_name_);

    }

    // actuator_data_["counts"] = actuator_data["counts"];
    // actuator_data_["state"] = actuator_data["state"];

    // std::cerr << "Data Written actuator 1" << std::endl;
    // std::cerr << "counts: " << actuator_data_["counts"] << std::endl;


}


