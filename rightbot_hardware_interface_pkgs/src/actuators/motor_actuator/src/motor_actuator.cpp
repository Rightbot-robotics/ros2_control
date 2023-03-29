#include <motor_actuator/motor_actuator.hpp>


PLUGINLIB_EXPORT_CLASS(MotorActuator, hardware_interface::ActuatorInterface)

using namespace std;


MotorActuator::MotorActuator() {

}

MotorActuator::~MotorActuator() = default;

CallbackReturn MotorActuator::on_init(const hardware_interface::HardwareInfo & info){
    // We hardcode the info
    // logger_ = spdlog::get("hardware_interface")->clone("motor_actuator");
   
    // logger_->info("Motor Actuator Init");
    std::cout << "Motor Actuator Init" << std::endl;
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    motor_name_ = info_.joints[0].name;
    std::cout << "motor_name_: " << motor_name_ << std::endl;

    motor_id_ = stoi(info.joints[0].parameters.at("can_id"));
    std::cout << "motor_id_: " << motor_id_ << std::endl;
    
    axis_ = stoi(info.joints[0].parameters.at("axis"));
    std::cout << "axis_: " << axis_ << std::endl;
    
    // logger_->info("motor_name_ {}", motor_name_);
    // logger_->info("motor_id_ {}", motor_id_);
    // logger_->info("axis_ {}", axis_);

    // motor_id_ = 12;
    // motor_name_ = "left_wheel";
    // axis_ = 1;

    // can only control one joint
    // if (info_.joints.size() != 1)
    // {
    //   return CallbackReturn::ERROR;
    // }

    // can only control in position 
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    
    if (command_interfaces.size() != 3)
    {
        // logger_->error("[{}] - Incorrect number of command interfaces", motor_name_);
        std::cout << "Incorrect number of command interfaces. " << std::endl;
        return CallbackReturn::ERROR;
    }
    

    for (const auto & command_interface : command_interfaces)
    {
        if (
            (command_interface.name != hardware_interface::HW_IF_POSITION) &&
            (command_interface.name != hardware_interface::HW_IF_MAX_VELOCITY) &&
            (command_interface.name != hardware_interface::HW_IF_ACCELERATION) 
        )
       {
            // logger_->error("[{}] - Incorrect type of command interfaces", motor_name_);
            std::cout << "Incorrect type of command interfaces. " << std::endl;


            return CallbackReturn::ERROR;
       }

    }

    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() != 8)
    {
        // logger_->error("[{}] - Incorrect number of state interfaces", motor_name_);
        std::cout << "Incorrect number of state interfaces. " << std::endl;
        return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
        if (
            (state_interface.name != hardware_interface::HW_IF_STATUS) &&
            (state_interface.name != hardware_interface::HW_IF_BATTERY_VOLTAGE) &&
            (state_interface.name != hardware_interface::HW_IF_INPUT_STATES) &&
            (state_interface.name != hardware_interface::HW_IF_POSITION) &&
            (state_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (state_interface.name != hardware_interface::HW_IF_MANUFACTURER_REGISTER) &&
            (state_interface.name != hardware_interface::HW_IF_LATCHED_FAULT) &&
            (state_interface.name != hardware_interface::HW_IF_NODE_GUARD_ERROR)
            )
       {
            // logger_->error("[{}] - Incorrect type of state interfaces", motor_name_);
            std::cout << "Incorrect type of state interfaces. " << std::endl;
            return CallbackReturn::ERROR;
       }

    }
    // fprintf(stderr, "TestSingleJointActuator configured successfully.\n");

    // logger_->info("[{}] - Intialiazation successful", motor_name_);
    std::cout << "Intialiazation successful. " << std::endl;
    
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

    // logger_->info("Motor Enable action for: [{}]",motor_name_);
    motor_->motor_enable(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return CallbackReturn::SUCCESS;

}

CallbackReturn MotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    // logger_->info("Motor Disable action for: [{}]",motor_name_);
    motor_->motor_disable(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return CallbackReturn::SUCCESS;


}

std::vector<hardware_interface::StateInterface> MotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_STATUS, &status_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_BATTERY_VOLTAGE, &battery_voltage_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_name_, hardware_interface::HW_IF_INPUT_STATES, &input_states_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_MANUFACTURER_REGISTER, &manufacturer_register_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_LATCHED_FAULT, &latched_fault_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_NODE_GUARD_ERROR, &node_guard_error_state_));

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> MotorActuator::export_command_interfaces(){
    // We can command in velocity
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_MAX_VELOCITY, &max_velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_command_));

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
    input_states_state_ = sensor_data["input_states"].asInt();

    position_state_ = sensor_data["counts"].asInt();
    velocity_state_ = sensor_data["velocity"].asDouble();

    manufacturer_register_state_ = sensor_data["manufacturer_register"].asInt();

    latched_fault_state_ = sensor_data["latched_fault"].asInt();

    node_guard_error_state_ = sensor_data["guard_err"].asInt();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    if(previous_position_command_ != position_command_){
        motor_controls_->set_relative_position(motor_id_, axis_, static_cast<uint32_t>( position_command_));
    }

    if(previous_max_velocity_command_ != max_velocity_command_){
        motor_controls_->set_profile_velocity(motor_id_, motor_controls_->motor_rpm_to_cps(max_velocity_command_));
    }

    if(previous_acceleration_command_ != acceleration_command_){
        motor_controls_->set_profile_acc(motor_id_, motor_controls_->motor_rps2_to_cps2(acceleration_command_));
        motor_controls_->set_profile_deacc(motor_id_, motor_controls_->motor_rps2_to_cps2(acceleration_command_));
    }
    
    previous_position_command_ = position_command_;
    previous_max_velocity_command_ = max_velocity_command_;
    previous_acceleration_command_ = acceleration_command_;
    
    return hardware_interface::return_type::OK;
}

CallbackReturn MotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

CallbackReturn MotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

void MotorActuator::Homing(){

    // 100 rpm , 10 rps2 base

    motor_controls_->set_profile_velocity(motor_id_, 100);
    motor_controls_->set_profile_acc(motor_id_, 10);
    motor_controls_->set_profile_deacc(motor_id_, 10);
    motor_controls_->set_relative_position(motor_id_, axis_, 85000);

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
            // logger_->info("Fault reset for : [{}]",motor_name_);
            motor_->motor_reset(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_quick_stop"){
            // logger_->info("Quick stop for : [{}]",motor_name_);
            motor_->motor_quick_stop(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_disable"){
            // logger_->info("Disable action for : [{}]",motor_name_);
            motor_->motor_disable(motor_id_);
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_enable"){
            // logger_->info("Enable action for : [{}]",motor_name_);
            motor_->motor_enable(motor_id_);
            
        }
        else{
            // logger_->info("Control Mode not recognized for : [{}]",motor_name_);
        }

    }
    else{
        // logger_->info("Control Mode Action not recognized for : [{}]",motor_name_);
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
        // logger_->info("'Write Data mode not recognized for motor [{}]",motor_sockets_->motor_name_);

    }

    // actuator_data_["counts"] = actuator_data["counts"];
    // actuator_data_["state"] = actuator_data["state"];

    // std::cerr << "Data Written actuator 1" << std::endl;
    // std::cerr << "counts: " << actuator_data_["counts"] << std::endl;


}


