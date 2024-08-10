#include <motor_actuator/motor_actuator.hpp>


PLUGINLIB_EXPORT_CLASS(MotorActuator, hardware_interface::ActuatorInterface)

using namespace std;


MotorActuator::MotorActuator() {

}

MotorActuator::~MotorActuator() = default;

CallbackReturn MotorActuator::on_init(const hardware_interface::HardwareInfo & info){

    // We hardcode the info
    logger_ = spdlog::get("hardware_interface")->clone("motor_actuator");
   
    // logger_->info("Motor Actuator Init");
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    motor_name_ = info_.joints[0].name;
    // std::cout << "motor_name_: " << motor_name_ << std::endl;

    motor_id_ = stoi(info.joints[0].parameters.at("can_id"));
    // std::cout << "motor_id_: " << motor_id_ << std::endl;
    
    axis_ = stoi(info.joints[0].parameters.at("axis"));
    // std::cout << "axis_: " << axis_ << std::endl;

    logger_->info("Motor Actuator Init actuator: [{}], can_id: [{}], axis: [{}]", motor_name_, motor_id_, axis_);

    std::string config_path = info.joints[0].parameters.at("path");

    is_position_control_ = (
        std::string(info.joints[0].parameters.at("position_control")) == "true"
    );

    if(is_position_control_) {
        logger_->info("[{}] Motor is set to position control mode", motor_name_);
    } else {
        logger_->info("[{}] Motor is set to velocity control mode", motor_name_);
    }
    travel_per_revolution = stod(info.joints[0].parameters.at("travel_per_revolution"));

    // init_json(config_path);

    int homing_status = stoi(info.joints[0].parameters.at("homing"));
    if(homing_status == 1){
        homing_active = true;
        homing_velocity = stod(info.joints[0].parameters.at("homing_velocity"));
        // std::cout << "homing_velocity: " << homing_velocity << std::endl;
        homing_acceleration = stod(info.joints[0].parameters.at("homing_acceleration"));
        // std::cout << "homing_acceleration: " << homing_acceleration << std::endl;
        homing_position = stod(info.joints[0].parameters.at("homing_position"));
        // std::cout << "homing_position: " << homing_position << std::endl;

        logger_->info("Actuator: [{}]-> HOMING velocity:[{}], accleration:[{}], position: [{}]", motor_name_, homing_velocity, homing_acceleration, homing_position);
    
        total_travel_distance = stod(info.joints[0].parameters.at("total_travel_distance"));
        
        logger_->info("Actuator: [{}]-> Total travel distance:[{}], Travel per revolution: [{}]", motor_name_, total_travel_distance, travel_per_revolution);
    
        homing_at_zero = stoi(info.joints[0].parameters.at("homing_at_zero"));
        // std::cout << "homing_at_zero: " << homing_at_zero << std::endl;
        if(homing_at_zero){
            logger_->info("Actuator: [{}]-> Homing at zero", motor_name_);
        }
        else{
            logger_->info("Actuator: [{}]-> Homing not at zero", motor_name_);
        }

    }

    motor_gear_ratio = stod(info.joints[0].parameters.at("motor_gear_ratio"));
    logger_->info("Actuator: [{}]-> Motor gear ratio:[{}]", motor_name_, motor_gear_ratio);

    default_max_velocity_ = stod(info.joints[0].parameters.at("default_max_velocity"));
    default_acceleration_ = stod(info.joints[0].parameters.at("default_max_accleration"));
    // std::cout << "default_max_velocity_: " << default_max_velocity_ << std::endl;
    // std::cout << "default_acceleration_: " << default_acceleration_ << std::endl;

    logger_->info("Actuator: [{}]-> Default max velocity: [{}], Default accleration [{}]", motor_name_, default_max_velocity_, default_acceleration_);

    const auto & command_interfaces = info_.joints[0].command_interfaces;
    
    if (command_interfaces.size() != 5)
    {
        logger_->error("[{}] - Incorrect number of command interfaces", motor_name_);
        // std::cout << "Incorrect number of command interfaces. " << std::endl;
        return CallbackReturn::ERROR;
    }
    
    for (const auto & command_interface : command_interfaces)
    {
        if (
            (command_interface.name != hardware_interface::HW_IF_POSITION) &&
            (command_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (command_interface.name != hardware_interface::HW_IF_ACCELERATION) &&
            (command_interface.name != hardware_interface::HW_IF_CONTROL_STATE) &&
            (command_interface.name != hardware_interface::HW_IF_GPIO) 
        )
       {
            logger_->error("[{}] - Incorrect type of command interfaces", motor_name_);
            // std::cout << "Incorrect type of command interfaces. " << std::endl;


            return CallbackReturn::ERROR;
       }

    }

    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() != 9)
    {
        logger_->error("[{}] - Incorrect number of state interfaces", motor_name_);
        // std::cout << "Incorrect number of state interfaces. " << std::endl;
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
            (state_interface.name != hardware_interface::HW_IF_NODE_GUARD_ERROR) &&
            (state_interface.name != hardware_interface::HW_IF_EFFORT)
            )
       {
            logger_->error("[{}] - Incorrect type of state interfaces", motor_name_);
            // std::cout << "Incorrect type of state interfaces. " << std::endl;
            return CallbackReturn::ERROR;
       }

    }

    logger_->info("[{}] - Intialiazation successful", motor_name_);

    functional_mode_state_ = static_cast<double>(ActuatorFunctionalState::OPERATIONAL);
    
    return CallbackReturn::SUCCESS;
}

void MotorActuator::init_json(std::string path){

    Json::Value config_data;
    JsonRead config_parser("/home/rightbot/test_ws/src/ros2_control/rightbot_hardware_interface_pkgs/src/config/config.json");

    if (!config_parser.parse())
    throw std::invalid_argument("Parsing error in config of Controller Manager");

    config_parser.getValue(config_data);

    if(config_data["motor_actuator"]["using_default_max_velocity"].asString() == "yes"){
        using_default_max_velocity_ = true;
        default_max_velocity_ = config_data["motor_actuator"]["default_max_velocity"].asDouble();
        std::cout << "default_max_velocity_: " << default_max_velocity_ << std::endl;
    }
    else{
        using_default_max_velocity_ = false;
    }

    if(config_data["motor_actuator"]["using_default_acceleration"].asString() == "yes"){
        using_default_acceleration_ = true;
        default_acceleration_ = config_data["motor_actuator"]["default_acceleration"].asDouble();
        std::cout << "default_acceleration_: " << default_acceleration_ << std::endl;
    }
    else{
        using_default_acceleration_ = false;
    }


}

CallbackReturn MotorActuator::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    motor_sockets_ = std::make_shared<Sockets>(motor_id_, motor_name_);
    motor_ = std::make_shared<Motor>(motor_sockets_);
    motor_controls_ = std::make_shared<MotorControls>(motor_sockets_);
    motor_->motor_init(motor_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    motor_->set_guard_time(motor_id_,0);
    motor_->set_life_time_factor(motor_id_,0);

    if(homing_active){
        if(!Homing()){

            return CallbackReturn::ERROR;
        }
    }
 
    logger_->info("[{}] Setting default max_velocity: [{}]",motor_name_, default_max_velocity_);
    motor_controls_->set_profile_velocity(motor_id_, default_max_velocity_);
    
    logger_->info("[{}] Setting default acceleration: [{}]",motor_name_, default_acceleration_);
    motor_controls_->set_profile_acc(motor_id_, default_acceleration_);
    motor_controls_->set_profile_deacc(motor_id_, default_acceleration_);

    if(is_position_control_){
        velocity_mode = false;
        motor_controls_->motorSetmode("position");
    }

    if(velocity_mode){
        motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
        motor_controls_->motorSetmode("velocity");
        motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
		logger_->info("[{}] Motor mode [velocity]. Setting zero velocity",motor_name_);
    }

    return CallbackReturn::SUCCESS;

}

CallbackReturn MotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    logger_->info("Motor Disable action for: [{}]",motor_name_);

    encoder_sensor->stop_read_thread();
    std::this_thread::sleep_for(std::chrono::microseconds(50000));

    motor_->motor_disable(motor_id_);

    return CallbackReturn::SUCCESS;


}

std::vector<hardware_interface::StateInterface> MotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity

    // std::cout << "export_state_interfaces for: "<< motor_name_ << std::endl;

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
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_EFFORT, &actual_motor_current_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, "functional_state", &functional_mode_state_));

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> MotorActuator::export_command_interfaces(){
    // We can command in velocity

    // std::cout << "export_command_interfaces for: "<< motor_name_ << std::endl;

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &max_velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_CONTROL_STATE, &control_state_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_GPIO, &gpio_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, "function_halt", &functional_mode_command_));

    return command_interfaces;
}

hardware_interface::return_type MotorActuator::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    
    encoder_sensor->getData(sensor_data);

    if(!initialization_done){
        logger_->info("[{}] CAN buffer clear command", motor_name_);
        encoder_sensor->enc_clear_can_buffer();

        if(sensor_data["read_status_encoder"].asBool()){
            initial_counts_rotation = sensor_data["counts"].asInt();
            logger_->info("[{}] actuator, initial counts: [{}] ", motor_name_, initial_counts_rotation);
            initialization_done = true;
        }

    }

    if(sensor_data["read_status"].asBool() == false){
        // return hardware_interface::return_type::ERROR;
    } 

    status_state_ = sensor_data["status"].asInt();
    battery_voltage_state_ = sensor_data["battery_voltage"].asDouble();
    input_states_state_ = sensor_data["input_states"].asInt();
    actual_motor_current_state_ = sensor_data["actual_motor_current"].asDouble();

    if(homing_active){
        if(homing_at_zero){
            position_state_ = (sensor_data["counts"].asInt() - initial_counts)*(travel_per_revolution/(motor_ppr * motor_gear_ratio));
            position_state_ = total_travel_distance - (sensor_data["counts"].asInt() - initial_counts)*(travel_per_revolution/(motor_ppr * motor_gear_ratio));
        } else {
            position_state_ = total_travel_distance - (initial_counts - sensor_data["counts"].asInt())*(travel_per_revolution/(motor_ppr * motor_gear_ratio));
            position_state_ = (initial_counts - sensor_data["counts"].asInt() )*(travel_per_revolution/(motor_ppr * motor_gear_ratio));
        }
        velocity_state_ = axis_* ((sensor_data["velocity"].asDouble()*travel_per_revolution)/(motor_gear_ratio*60));
    } else {
        // position_state_ = axis_*((((sensor_data["counts"].asInt() - initial_counts_rotation) + initial_counts_offset)*3.14*2)/(motor_ppr_*motor_gear_ratio));
        velocity_state_ = axis_* ((sensor_data["velocity"].asDouble()*travel_per_revolution)/(motor_gear_ratio*60));
        position_state_ = axis_*((sensor_data["counts"].asInt() - initial_counts_rotation)*(travel_per_revolution)/(motor_ppr_*motor_gear_ratio));
        logger_->debug("[{}] Read pos debug: [{}], counts: [{}]", motor_name_, position_state_, sensor_data["counts"].asInt() );
    }

    manufacturer_register_state_ = sensor_data["manufacturer_register"].asInt();
    latched_fault_state_ = sensor_data["latched_fault"].asInt();
    node_guard_error_state_ = sensor_data["guard_err"].asInt();
    functional_mode_state_ = static_cast<double>(curr_state_);

    logger_->debug("[{}] Read status: [{}], battery_voltage: [{}], input_states: [{}], actual_motor_current: [{}]", motor_name_, status_state_, battery_voltage_state_, input_states_state_, actual_motor_current_state_);
    logger_->debug("[{}] Read position: [{}], velocity: [{}]", motor_name_, position_state_, velocity_state_);
    logger_->debug("[{}] Read manufacturer_register: [{}], latched_fault: [{}], node_guard_error: [{}]", motor_name_, manufacturer_register_state_, latched_fault_state_, node_guard_error_state_);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    if (!std::isnan(functional_mode_command_)) {
        logger_->info("[{}] Functional mode command: [{}]", motor_name_, functional_mode_command_);
        commanded_state_ = static_cast<ActuatorFunctionalState>(functional_mode_command_);
        switch (commanded_state_) {
            case ActuatorFunctionalState::OPERATIONAL:
            case ActuatorFunctionalState::SOFT_STOP:
            case ActuatorFunctionalState::HARD_STOP: {
                target_state_ = commanded_state_;
                break;
            }
            default: {
                logger_->warn("[{}] Invalid functional mode command: [{}]", motor_name_, functional_mode_command_);                break;
            }
        }
        functional_mode_command_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (curr_state_ != target_state_) {
        switch (target_state_) {
            case ActuatorFunctionalState::OPERATIONAL: {
                if (curr_state_ == ActuatorFunctionalState::HARD_STOP) {
                    logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
                    max_velocity_command_ = 0.0;
                    motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
                    logger_->info("[{}] Control state command: Actuator enable", motor_name_);
                    motor_->motor_enable(motor_id_);
                }
                curr_state_ = ActuatorFunctionalState::OPERATIONAL;
                break;
            }
            case ActuatorFunctionalState::SOFT_STOP: {
                motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
                max_velocity_command_ = 0.0;
                if (std::abs(velocity_state_ / travel_per_revolution) < 0.01) {
                    curr_state_ = ActuatorFunctionalState::SOFT_STOP;
                }
                break;
            }
            case ActuatorFunctionalState::HARD_STOP: {
                logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
                motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
                logger_->info("[{}] Control state command: Actuator quick stop", motor_name_);
                motor_->motor_quick_stop(motor_id_);
                max_velocity_command_ = 0.0;
                curr_state_ = ActuatorFunctionalState::HARD_STOP;
                break;
            }
        }
    }

    if (curr_state_ != target_state_ || target_state_ != ActuatorFunctionalState::OPERATIONAL) {
        return hardware_interface::return_type::OK;
    }

    if(previous_control_state_command_ != control_state_command_){
        logger_->info("[{}] Control state command: [{}]", motor_name_, control_state_command_);
        if(static_cast<int>(control_state_command_) == ACTUATOR_ENABLE){

            logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
            motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);

            logger_->info("[{}] Control state command: Actuator enable", motor_name_);
            motor_->motor_enable(motor_id_);

        } else if (static_cast<int>(control_state_command_) == ACTUATOR_DISABLE) {

            logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
            motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);
            
            logger_->info("[{}] Control state command: Actuator disable", motor_name_);
            motor_->motor_disable(motor_id_);

        } else if (static_cast<int>(control_state_command_) == ACTUATOR_QUICK_STOP) {
            
            logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
            motor_controls_->set_vel_speed(motor_id_, axis_, 0.0);

            logger_->info("[{}] Control state command: Actuator quick stop", motor_name_);
            motor_->motor_quick_stop(motor_id_);


        } else {

            logger_->info("[{}] Control state command not recognized", motor_name_);
            
        }

    }

    if((max_velocity_command_ > (previous_max_velocity_command_ + velocity_epsilon)) 
        || (max_velocity_command_ < (previous_max_velocity_command_ - velocity_epsilon)) ){
        
        if(abs(max_velocity_command_) < (10e-3)){
            max_velocity_command_ = 0.0;
        }
        
        if(!using_default_max_velocity_){

            if(homing_active){

                logger_->debug("[{}] Velocity command in m/s: [{}]", motor_name_, max_velocity_command_);
                double max_velocity_command_final_ = (max_velocity_command_/travel_per_revolution)*motor_gear_ratio*60;
                max_velocity_command_final_ = static_cast<float>(max_velocity_command_final_);
                float scaled_max_vel = 1.0f * max_velocity_command_final_;
                logger_->debug("[{}] Velocity command in rpm: [{}]", motor_name_, scaled_max_vel);
                motor_controls_->set_vel_speed(motor_id_, axis_, scaled_max_vel);


            } else {

                logger_->debug("[{}] Velocity command is: [{}]", motor_name_, max_velocity_command_);
                double revolution_per_min = (max_velocity_command_/travel_per_revolution)*motor_gear_ratio*60;
                float max_velocity_command_final_ = static_cast<float>(revolution_per_min);
                float scaled_max_vel = 1.0f * max_velocity_command_final_;
                logger_->debug("[{}] Velocity command in rpm: [{}]", motor_name_, scaled_max_vel);
                motor_controls_->set_vel_speed(motor_id_, axis_, scaled_max_vel);
            }

        }
    }

    if((acceleration_command_ > (previous_acceleration_command_ + acceleration_epsilon)) || (acceleration_command_ < (previous_acceleration_command_ - acceleration_epsilon))){
        if((acceleration_command_ > (0 + acceleration_epsilon)) || (acceleration_command_ < (0 - acceleration_epsilon))){

            if(!using_default_acceleration_){
                if(homing_active){

                    logger_->info("[{}] Acceleration command in m/s2: [{}]", motor_name_, acceleration_command_);
                    double acceleration_command_final_ = abs((acceleration_command_/travel_per_revolution)*motor_gear_ratio);
                    acceleration_command_final_ = static_cast<float>(acceleration_command_final_);
                    float scaled_acceleration = static_cast<float>(acceleration_command_final_* 1.0f);
                    logger_->info("[{}] Acceleration command in rps2: [{}]", motor_name_, scaled_acceleration);
                    motor_controls_->set_profile_acc(motor_id_, scaled_acceleration);
                    motor_controls_->set_profile_deacc(motor_id_, scaled_acceleration);
                }
                else{

                    logger_->info("[{}] Acceleration command in radian/sec2: [{}]", motor_name_, acceleration_command_);
                    double degree_per_sec = (acceleration_command_*(180/3.14));
                    double revolution_per_sec = abs(degree_per_sec/360.0);
                    float scaled_acceleration = revolution_per_sec * 1.0f;
                    logger_->info("[{}] Acceleration command in rps2: [{}]", motor_name_, scaled_acceleration);
                    motor_controls_->set_profile_acc(motor_id_, scaled_acceleration);
                    motor_controls_->set_profile_deacc(motor_id_, scaled_acceleration);
                }
            }
        }
    }

    if(previous_position_command_ != position_command_){

        if(!velocity_mode){

            if(homing_active){
                logger_->info("[{}] Position command: [{}] m", motor_name_, position_command_);

                int position_command_final_;
                if(homing_at_zero){
                    //h gantry
                    // auto position_command_final_tmp = initial_counts + (( position_command_)/travel_per_revolution)*motor_ppr*motor_gear_ratio;
                    auto position_command_final_tmp = initial_counts + ((total_travel_distance - position_command_)/travel_per_revolution)*motor_ppr*motor_gear_ratio;
                    position_command_final_ = static_cast<int32_t>(position_command_final_tmp);
                    
                } else {
                    // v gantry
                    // auto position_command_final_tmp = initial_counts - ((total_travel_distance - position_command_)/travel_per_revolution)*motor_ppr*motor_gear_ratio;
                    auto position_command_final_tmp = initial_counts - (( position_command_)/travel_per_revolution)*motor_ppr*motor_gear_ratio;
                    position_command_final_ = static_cast<int32_t>(position_command_final_tmp);
                    
                }

                logger_->info("[{}] Position command in counts: [{}]",motor_name_, position_command_final_);
                motor_controls_->set_relative_position_immediate(motor_id_, axis_, position_command_final_);

            } else {

                logger_->debug("[{}] Position command: [{}]", motor_name_, position_command_);
                double required_count_d = ((axis_ * position_command_ * motor_ppr_ * motor_gear_ratio)/travel_per_revolution) + initial_counts_rotation;
                int counts = (int)required_count_d;
                logger_->debug("[{}] Position command in counts: [{}]", motor_name_, counts);
                motor_controls_->set_absolute_position(motor_id_, axis_, counts); // send absolute position internally
            }
        }
        
    }
    
    if(previous_gpio_command_ != gpio_command_){
        if(gpio_command_ == 1.0){
            logger_->info("[{}] - gpio command ->  Pump Switch ON, Gripper Switch OFF", motor_name_);
            motor_controls_->set_gpio(motor_id_, 1); 

        } else if (gpio_command_ == 2.0){
            logger_->info("[{}] - gpio command ->  Gripper Switch ON, Pump Switch OFF", motor_name_);
            motor_controls_->set_gpio(motor_id_, 4);
           
        } else if (gpio_command_ == 3.0) {
            logger_->info("[{}] - gpio command ->  Pump and Gripper Switch ON", motor_name_);
            motor_controls_->set_gpio(motor_id_, 5);

        } else if (gpio_command_ == 0.0) {
            logger_->info("[{}] - gpio command ->  Pump and Gripper Switch OFF", motor_name_);
            motor_controls_->clear_gpio(motor_id_);
        } else {
            logger_->error("[{}] - gpio command not recognized", motor_name_);
        }

    }

    previous_position_command_ = position_command_;
    previous_max_velocity_command_ = max_velocity_command_;
    previous_acceleration_command_ = acceleration_command_;
    previous_control_state_command_ = control_state_command_;
    previous_gpio_command_ = gpio_command_;
    
    return hardware_interface::return_type::OK;
}

CallbackReturn MotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

CallbackReturn MotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

void MotorActuator::fault_reset(){
    logger_->debug("[{}] - RESET FAULT", motor_name_);
	motor_->motor_reset(motor_id_);
}

void MotorActuator::reinitialize_actuator(){
    logger_->warn("[{}] - Reinitialise motor is not implemented yet", motor_name_);
}

void MotorActuator::clear_can_buffer(){

    encoder_sensor->readToClearBuffer();

    //
}

void MotorActuator::data_request(){

    requestData();
}

void MotorActuator::homing_execution(double &homing_pos){

    // using this function for position reset of camera rotation joint using absolute encoder data

    if(!sync_with_absolute_encoder && homing_pos != 0.000){
        logger_->info("[{}] - Syncing with absolute encoder data angle: [{}] rad", motor_name_,homing_pos);
        int counts = ((homing_pos*180)/3.14)*((motor_ppr*motor_gear_ratio)/360);
        initial_counts_offset = counts;   
        motor_controls_->motorSetmode("position");
        sync_with_absolute_encoder = true;
    }

    logger_->debug("[{}] - Resetting pos data as: [{}] rad", motor_name_,homing_pos);
    position_state_ = -homing_pos;

}

void MotorActuator::node_guarding_request(){
    
    motor_controls_->nodeGuardingRequest();

}

bool MotorActuator::Homing(){

    // 100 rpm , 10 rps2 base

    Json::Value sensor_data_homing;

    // std::cout << "execute homing " << std::endl;

    auto homing_distance_to_travel = static_cast<int32_t>((homing_position/travel_per_revolution)*motor_ppr*motor_gear_ratio);
    // std::cout << "homing_distance_to_travel: " << homing_distance_to_travel << std::endl;

    std::chrono::system_clock::time_point recovery_lift_down_time = std::chrono::system_clock::now();
          
    auto time_passed_response_received_lift_down = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - recovery_lift_down_time);
    
    motor_controls_->set_profile_velocity(motor_id_, homing_velocity);
    motor_controls_->set_profile_acc(motor_id_, homing_acceleration);
    motor_controls_->set_profile_deacc(motor_id_, homing_acceleration);
    motor_controls_->set_relative_position(motor_id_, axis_, homing_distance_to_travel);

    while((time_passed_response_received_lift_down.count()<60000) && (homing_achieved == false)){

        requestData();

        std::this_thread::sleep_for(std::chrono::microseconds(2000));
        
        encoder_sensor->getData(sensor_data_homing);

        if(sensor_data_homing["read_status_encoder"].asBool() == true){
            initial_counts = sensor_data_homing["counts"].asInt();
        }

        if(sensor_data_homing["read_status_voltage"].asBool()){

            // std::cout << "encoder counts value: " << initial_counts << std::endl;
            // std::cout << "input states value: " << sensor_data_homing["input_states"].asInt() << std::endl;
            auto limit_switch_pos = !(( sensor_data_homing["input_states"].asInt() & (1 << 3)) >> 3);
            auto limit_switch_neg = !(( sensor_data_homing["input_states"].asInt() & (1 << 10)) >> 10);
            // std::cout << "input states pos value: " << limit_switch_pos << std::endl;
            // std::cout << "input states neg value: " << limit_switch_neg << std::endl;

            // if(limit_switch_pos != 0){ // when bit set
            //     homing_achieved = true;
            // }

            if((homing_distance_to_travel < 0) && (limit_switch_pos == 1)){
                homing_achieved = true;
            }

            if((homing_distance_to_travel > 0) && (limit_switch_neg == 1)){
                homing_achieved = true;
            }
        }

        time_passed_response_received_lift_down = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - recovery_lift_down_time);
        std::this_thread::sleep_for(std::chrono::microseconds(20000));

    }

    if(!homing_achieved){
        // std::cout << "homing timeout" << std::endl;
        logger_->error("[{}] Homing timeout", motor_name_);
        return false;
    }
    else{
        // std::cout << "homing achieved" << std::endl;
        logger_->info("[{}] Homing achieved", motor_name_);
        if(motor_name_ != "camera_rotation_joint"){
            motor_->set_guard_time(motor_id_,50);
            motor_->set_life_time_factor(motor_id_,6);
        }
        return true;
    }

    

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


