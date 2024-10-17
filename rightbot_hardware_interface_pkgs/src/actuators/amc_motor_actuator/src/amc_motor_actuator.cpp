#include "amc_motor_actuator/amc_motor_actuator.hpp"
PLUGINLIB_EXPORT_CLASS(AmcMotorActuator, hardware_interface::ActuatorInterface)


AmcMotorActuator::AmcMotorActuator() {

}

AmcMotorActuator::~AmcMotorActuator(){

	set_vel_speed(motor_id_, axis_, 0.0);
	motorControlword(motor_id_, Disable_Voltage);


}

CallbackReturn AmcMotorActuator::on_init(const hardware_interface::HardwareInfo & info){
    // We hardcode the info
    logger_ = spdlog::get("hardware_interface")->clone("amc_motor_actuator");
   
    logger_->info(" Amc Motor Actuator Initializing...");
  	logger_->flush();

    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    motor_id_ = stoi(info.joints[0].parameters.at("can_id"));
    motor_name_ = info_.joints[0].name;
    axis_ = stoi(info.joints[0].parameters.at("axis"));
    can_network_ = info.joints[0].parameters.at("can_network");

	logger_->info("Amc Motor Actuator Init actuator: [{}], can_id: [{}], axis: [{}]", motor_name_, motor_id_, axis_);
    
    default_max_velocity_ = stod(info.joints[0].parameters.at("default_max_velocity"));
    default_acceleration_ = stod(info.joints[0].parameters.at("default_max_accleration"));
    default_deceleration_ = stod(info.joints[0].parameters.at("default_max_deceleration"));
	
	homing_max_velocity_ = stod(info.joints[0].parameters.at("homing_max_velocity"));
    homing_acceleration_ = stod(info.joints[0].parameters.at("homing_max_accleration"));
    homing_deceleration_ = stod(info.joints[0].parameters.at("homing_max_deceleration"));

	is_homing_ = stoi(info.joints[0].parameters.at("is_homing"));
	travel_per_revolution_ = stod(info.joints[0].parameters.at("travel_per_revolution"));
	is_homing_at_min_ = stoi(info.joints[0].parameters.at("homing_at_min"));
	min_position_ = stod(info.joints[0].parameters.at("min_position"));
	max_position_ = stod(info.joints[0].parameters.at("max_position"));
	homing_position_ = max_position_ - min_position_;

	motor_gear_ratio_ = stod(info.joints[0].parameters.at("motor_gear_ratio"));
	motor_ppr_ = stod(info.joints[0].parameters.at("motor_ppr"));
    
	// mode_of_operation_ = std::string(info.joints[0].parameters.at("mode_of_operation"));
    // std::cout << "default_max_velocity_: " << default_max_velocity_ << std::endl;
    // std::cout << "default_acceleration_: " << default_acceleration_ << std::endl;
	logger_->info("Actuator: [{}]-> Default max velocity: [{}], Default accleration [{}]", motor_name_, default_max_velocity_, default_acceleration_);


    // can only control in position 
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    logger_->info("Number of command interfaces: {}", command_interfaces.size());
    if (command_interfaces.size() != 9)
    {
        logger_->error("[{}] - Incorrect number of command interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }

    for (const auto & command_interface : command_interfaces)
    {
        if (
            (command_interface.name != hardware_interface::HW_IF_POSITION) &&
            (command_interface.name != hardware_interface::HW_IF_VELOCITY) &&
			(command_interface.name != hardware_interface::HW_IF_CONTROL_STATE) &&
			(command_interface.name != hardware_interface::HW_IF_POSTITON_KP) &&
			(command_interface.name != hardware_interface::HW_IF_POSTITON_KI) &&
			(command_interface.name != hardware_interface::HW_IF_POSTITON_KD) &&
			(command_interface.name != hardware_interface::HW_IF_VELOCITY_KP) &&
			(command_interface.name != hardware_interface::HW_IF_VELOCITY_KI) &&
			(command_interface.name != hardware_interface::HW_IF_VELOCITY_KD)
        )
       {
            logger_->error("[{}] - Incorrect type of command interfaces", motor_name_);
            
            return CallbackReturn::ERROR;
       }

    }

    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
	logger_->info("Number of state interfaces: {}", state_interfaces.size());
    if (state_interfaces.size() != 14)
    {
        logger_->error("[{}] - Incorrect number of state interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
        if (
            (state_interface.name != hardware_interface::HW_IF_POSITION) &&
            (state_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (state_interface.name != hardware_interface::HW_IF_STATUS) &&
			(state_interface.name != hardware_interface::HW_IF_EFFORT) &&
			(state_interface.name != hardware_interface::HW_IF_AMC_DRIVE_SYSTEM_STATUS_1) &&
			(state_interface.name != hardware_interface::HW_IF_AMC_DRIVE_SYSTEM_STATUS_2) &&
			(state_interface.name != hardware_interface::HW_IF_AMC_DRIVE_PROTECTION_STATUS) &&
			(state_interface.name != hardware_interface::HW_IF_AMC_SYSTEM_PROTECTION_STATUS) &&
			(state_interface.name != hardware_interface::HW_IF_POSTITON_KP) &&
			(state_interface.name != hardware_interface::HW_IF_POSTITON_KI) &&
			(state_interface.name != hardware_interface::HW_IF_POSTITON_KD) &&
			(state_interface.name != hardware_interface::HW_IF_VELOCITY_KP) &&
			(state_interface.name != hardware_interface::HW_IF_VELOCITY_KI) &&
			(state_interface.name != hardware_interface::HW_IF_VELOCITY_KD)
			)
       {
            logger_->error("[{}] - Incorrect type of state interfaces", motor_name_);

            return CallbackReturn::ERROR;
       }

    }
    // fprintf(stderr, "TestSingleJointActuator configured successfully.\n");

    logger_->info("[{}] - Intialiazation successful", motor_name_);
    
	functional_mode_state_ = static_cast<double>(ActuatorFunctionalState::OPERATIONAL);
	
    return CallbackReturn::SUCCESS;
}

CallbackReturn AmcMotorActuator::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    previous_mode = "not_set";
    amc_motor_actuator_sockets_ = std::make_shared<AmcMotorActuatorSockets>(motor_id_, motor_name_, can_network_);
    
    initMotor();
    encoder_sensor_ = std::make_shared<AmcEncoderSensor>();
    encoder_sensor_->initialize(amc_motor_actuator_sockets_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

	
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn AmcMotorActuator::on_activate(const rclcpp_lifecycle::State & previous_state){
	
    logger_->info("Motor Enable action for: [{}]",motor_name_);
	
	enable_brake(true);	
    enableMotor();

	set_guard_time(motor_id_,50);
    set_life_time_factor(motor_id_,6);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	if(!Homing()){
        return CallbackReturn::ERROR;
    }		

	if (is_homing_){
		initMotor();
		amc_drive_reset();
		enableMotor();
	
		set_guard_time(motor_id_,50);
    	set_life_time_factor(motor_id_,6);
	}

	if(previous_mode_of_operation_ == "velocity")
	{
        motorSetmode(Motor_mode_Velocity);
		set_profile_velocity(default_max_velocity_);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		set_profile_acc(default_acceleration_);
		set_profile_deacc(default_deceleration_);
		set_PTPC(default_acceleration_);
		set_PTNC(default_deceleration_);
		set_NTNC(default_acceleration_);
		set_NTPC(default_deceleration_);
		set_vel_speed(motor_id_, axis_, 0.0);
		logger_->info("[{}] Motor mode [velocity]. Setting zero velocity done!",motor_name_);
    }

	if (previous_mode_of_operation_ == "position")
	{
		set_profile_velocity(default_max_velocity_);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		set_profile_acc(default_acceleration_);
		set_profile_deacc(default_deceleration_);
		motorSetmode(Motor_mode_Position);
		logger_->info("[{}] Motor mode [position]",motor_name_);
	}
	

    return CallbackReturn::SUCCESS;

}

CallbackReturn AmcMotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

	//
    logger_->info("Motor Disable action for: [{}]",motor_name_);
    disableMotor();
	enable_brake(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> AmcMotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_STATUS, &status_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_EFFORT, &actual_motor_current_state_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_AMC_DRIVE_SYSTEM_STATUS_1, &amc_drive_system_status_1_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_AMC_DRIVE_SYSTEM_STATUS_2, &amc_drive_system_status_2_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_AMC_DRIVE_PROTECTION_STATUS, &amc_drive_protection_status_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_AMC_SYSTEM_PROTECTION_STATUS, &amc_system_protection_status_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KP, &position_kp_value_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KI, &position_ki_value_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KD, &position_kd_value_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KP, &velocity_kp_value_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KI, &velocity_ki_value_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KD, &velocity_kd_value_));                    
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, "functional_state", &functional_mode_state_));
    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> AmcMotorActuator::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &max_velocity_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_CONTROL_STATE, &control_state_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KP, &position_kp_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KI, &position_ki_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSTITON_KD, &position_kd_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KP, &velocity_kp_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KI, &velocity_ki_command_));
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY_KD, &velocity_kd_command_));        
	command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, "function_halt", &functional_mode_command_));
    return command_interfaces;


}

hardware_interface::return_type AmcMotorActuator::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
	
    encoder_sensor_->getData(sensor_data);

    if(sensor_data["read_status"].asBool() == false){
        // return hardware_interface::return_type::ERROR;
    }
    
    status_state_ = sensor_data["status"].asInt();
    error_code_state_ = sensor_data["err_code"].asInt();
	actual_motor_current_state_ = sensor_data["actual_motor_current"].asDouble();
	
    position_state_ = (((sensor_data["counts"].asInt() - counts_offset_) * axis_) / motor_ppr_) * travel_per_revolution_;
    velocity_state_ = ((axis_ * (sensor_data["velocity"].asDouble())) / (motor_ppr_)) * travel_per_revolution_; 

    node_guard_error_state_ = sensor_data["guard_err"].asInt();

	amc_drive_system_status_1_ = sensor_data["amc_drive_stat"].asInt();

	amc_drive_system_status_2_ = sensor_data["amc_system_stat"].asInt();
	
	amc_drive_protection_status_ = sensor_data["amc_drive_stat_1"].asInt();
	
	amc_system_protection_status_ = sensor_data["amc_drive_stat_2"].asInt();

	position_kp_value_ = sensor_data["position_kp"].asInt();

	position_ki_value_ = sensor_data["position_ki"].asInt();

	position_kd_value_ = sensor_data["position_kd"].asInt();

	velocity_kp_value_ = sensor_data["velocity_kp"].asInt();

	velocity_ki_value_ = sensor_data["velocity_ki"].asInt();

	velocity_kd_value_ = sensor_data["velocity_kd"].asInt();

	functional_mode_state_ = static_cast<double>(curr_state_);

	if(sensor_data["read_status"].asBool() == true) {
		logger_->debug("[{}] Read status: [{}], actual_motor_current: [{}], error_code: [{}]", motor_name_, status_state_, actual_motor_current_state_, error_code_state_);
	    logger_->debug("[{}] Read position: [{}], velocity: [{}]", motor_name_, position_state_, velocity_state_);
    	logger_->debug("[{}] Read node_guard_error_state_: [{}]", motor_name_, node_guard_error_state_);

	}
	else {
		logger_->debug("[{}] read status false", motor_name_);
	}
	

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmcMotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

	// logger_->info("[{}] Max velocity command: [{}]", motor_name_, max_velocity_command_);
	// logger_->info("[{}] Functional Mode Command: [{}]", motor_name_, functional_mode_command_);

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
                    set_vel_speed(motor_id_, axis_, 0.0);
                    logger_->info("[{}] Control state command: Actuator enable", motor_name_);
                    enableMotor();
                }
                curr_state_ = ActuatorFunctionalState::OPERATIONAL;
                break;
            }
            case ActuatorFunctionalState::SOFT_STOP: {
                set_vel_speed(motor_id_, axis_, 0.0);
                max_velocity_command_ = 0.0;
                if (std::abs(velocity_state_ / travel_per_revolution_) < 0.01) {
                    curr_state_ = ActuatorFunctionalState::SOFT_STOP;
                }
                break;
            }
            case ActuatorFunctionalState::HARD_STOP: {
                logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
                set_vel_speed(motor_id_, axis_, 0.0);
                logger_->info("[{}] Control state command: Actuator quick stop", motor_name_);
                quickStopMotor();
                max_velocity_command_ = 0.0;
                curr_state_ = ActuatorFunctionalState::HARD_STOP;
                break;
            }
        }
    }

    if (curr_state_ != target_state_ || target_state_ != ActuatorFunctionalState::OPERATIONAL) {
        max_velocity_command_ = 0.0;
        previous_max_velocity_command_ = 0.0;
        return hardware_interface::return_type::OK;
    }

	if(previous_control_state_command_ != control_state_command_ && !std::isnan(control_state_command_)){
        logger_->info("[{}] Control state command: [{}]", motor_name_, control_state_command_);
		if(static_cast<int>(control_state_command_) == ACTUATOR_ENABLE){

			logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
			set_vel_speed(motor_id_, axis_, 0.0);

            logger_->info("[{}] Control state command: Actuator enable", motor_name_);
            enableMotor();

        } else if (static_cast<int>(control_state_command_) == ACTUATOR_DISABLE) {
            
			logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
			set_vel_speed(motor_id_, axis_, 0.0);

			logger_->info("[{}] Control state command: Actuator disable", motor_name_);
            disableMotor();

        } else if (static_cast<int>(control_state_command_) == ACTUATOR_QUICK_STOP) {
            
			logger_->info("[{}] Control mode change. Writing zero velocity command.", motor_name_);
			set_vel_speed(motor_id_, axis_, 0.0);


			logger_->info("[{}] Control state command: Actuator quick stop", motor_name_);
            quickStopMotor();


        } else {

            logger_->info("[{}] Control state command not recognized", motor_name_);

        }
	}


	// if (mode_of_operation_ == "velocity" && !std::isnan(max_velocity_command_)){
	if (!std::isnan(max_velocity_command_)){
		if (previous_mode_of_operation_ != "velocity"){
			logger_->info("[{}] Max velocity command: [{}]", motor_name_, max_velocity_command_);
			motorSetmode(Motor_mode_Velocity);
			set_profile_velocity(default_max_velocity_);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			set_profile_acc(default_acceleration_);
			set_profile_deacc(default_deceleration_);
			set_PTPC(default_acceleration_);
			set_PTNC(default_deceleration_);
			set_NTNC(default_acceleration_);
			set_NTPC(default_deceleration_);
			set_vel_speed(motor_id_, axis_, 0.0);
			logger_->info("[{}] Motor mode [velocity]. Setting zero velocity done!",motor_name_);
			previous_mode_of_operation_ = "velocity";
			auto vel = (((max_velocity_command_ / travel_per_revolution_) * motor_ppr_) * (axis_));
			set_vel_speed(motor_id_, axis_, vel);
		}
		else if((max_velocity_command_ > (previous_max_velocity_command_ + velocity_epsilon)) 
    	    || (max_velocity_command_ < (previous_max_velocity_command_ - velocity_epsilon)) ){
			
    	    if(abs(max_velocity_command_) < (10e-3)){
    	        max_velocity_command_ = 0.0;
    	    }
			if (default_max_velocity_ >= abs(max_velocity_command_)){	
    	        logger_->debug("[{}] Velocity command in radian per sec: [{}]", motor_name_, max_velocity_command_);
				auto velocity = (((max_velocity_command_ / travel_per_revolution_) * motor_ppr_) * (axis_));
				set_vel_speed(motor_id_, axis_, velocity);
				}
		}
	}

	// if (mode_of_operation_ == "position" && !std::isnan(position_command_)) {
	if (!std::isnan(position_command_)) {
		if (previous_mode_of_operation_ != "position"){
	    	max_velocity_command_ = std::nan("");
			set_profile_velocity(default_max_velocity_);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			set_profile_acc(default_acceleration_);
			set_profile_deacc(default_deceleration_);
			motorSetmode(Motor_mode_Position);
			logger_->info("[{}] Motor mode [position]",motor_name_);	
			previous_mode_of_operation_ = "position";
		}
		if(previous_position_command_ != position_command_){
			logger_->info("[{}] Position command: [{}]", motor_name_, position_command_);
			auto position = (((position_command_ / travel_per_revolution_) * motor_ppr_) * (axis_)) + (counts_offset_);
			set_relative_position(static_cast<int32_t>(position));
		}
    }
	
	if (!std::isnan(position_kp_command_) && !std::isnan(position_ki_command_) && !std::isnan(position_kd_command_)) {
			logger_->info("[{}] Position Kp command: [{}], Position Ki command: [{}], Position Kd command: [{}]", motor_name_, position_kp_command_, position_ki_command_, position_kd_command_);
			set_position_kp(position_kp_command_);
			set_position_ki(position_ki_command_);
			set_position_kd(position_kd_command_);
			store_params_to_drive();
			encoder_sensor_->read_position_pid_values = true;
	}

	if (!std::isnan(velocity_kp_command_) && !std::isnan(velocity_ki_command_) && !std::isnan(velocity_kd_command_)) {
			logger_->info("[{}] Velocity Kp command: [{}], Velocity Ki command: [{}], Velocity Kd command: [{}]", motor_name_, velocity_kp_command_, velocity_ki_command_, velocity_kd_command_);
			set_velocity_kp(velocity_kp_command_);
			set_velocity_ki(velocity_ki_command_);
			set_velocity_kd(velocity_kd_command_);
			store_params_to_drive();
			encoder_sensor_->read_velocity_pid_values = true;
	}

	previous_max_velocity_command_ = max_velocity_command_;
	previous_position_command_ = position_command_;
	previous_control_state_command_ = control_state_command_;

    position_command_ = std::nan("");
    max_velocity_command_ = std::nan("");
    acceleration_command_ = std::nan("");
	control_state_command_ = std::nan("");

	position_kp_command_ = std::nan("");
	position_ki_command_ = std::nan("");
	position_kd_command_ = std::nan("");
	velocity_kp_command_ = std::nan("");
	velocity_ki_command_ = std::nan("");
	velocity_kd_command_ = std::nan("");

    return hardware_interface::return_type::OK;
}

CallbackReturn AmcMotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

    disableMotor();
	return CallbackReturn::SUCCESS;

}

CallbackReturn AmcMotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

    disableMotor();
	return CallbackReturn::SUCCESS;

}

void AmcMotorActuator::fault_reset(){
    logger_->debug("[{}] - reset fault", motor_name_);
	resetFault();
	amc_drive_reset();
}

void AmcMotorActuator::reinitialize_actuator(){
    logger_->debug("[{}] - reinitialize actuator", motor_name_);
	reinitializeMotor();
}

// void AmcMotorActuator::clear_can_buffer(){

// 	encoder_sensor_->readToClearBuffer();

// }

void AmcMotorActuator::homing_execution(double &homing_pos){

}

void AmcMotorActuator::data_request(){
    requestData();
}

void AmcMotorActuator::node_guarding_request(){
    sendNodeGuardingRequest();
}

bool AmcMotorActuator::Homing(){

	Json::Value sensor_data_homing;
		
	int max_tries = 4;
	int current_tries = 0;
	
	while (!sensor_data_homing["read_status"].asBool() && current_tries < max_tries)
	{
		requestData();
		encoder_sensor_->getData(sensor_data_homing);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		current_tries++;
		
		logger_->info("[{}] - Homing loop count: [{}]", motor_name_, current_tries);
	}

	if (!sensor_data_homing["read_status"].asBool())
	{
		logger_->warn("[{}] - Unable to read initial counts offset", motor_name_);
		return false;
	}

	counts_offset_ = sensor_data_homing["counts"].asInt();
		
	if (is_homing_)
	{	
		set_profile_velocity(homing_max_velocity_);
		set_profile_acc(homing_acceleration_);
		set_profile_deacc(homing_deceleration_);
		motorSetmode(Motor_mode_Position);

		bool homing_achieved = false;
		
		auto homing_distance_counts = static_cast<int32_t>((homing_position_ / travel_per_revolution_) * motor_ppr_ * motor_gear_ratio_);
		auto commanded_count = 0;
		if (is_homing_at_min_)
		{
			commanded_count = homing_distance_counts * 1;				
		}
		else
		{
			commanded_count = homing_distance_counts * -1;
		}
		
		logger_->info("[{}] Homing distance: [{}]", motor_name_, commanded_count);
		logger_->info("[{}] Homing counts offset: [{}]", motor_name_, counts_offset_);

		set_relative_position(commanded_count + counts_offset_);
		
		std::chrono::system_clock::time_point recovery_lift_down_time = std::chrono::system_clock::now();
          
    	auto time_passed_response_received_lift_down = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - recovery_lift_down_time);

		while((time_passed_response_received_lift_down.count()<60000) && (homing_achieved == false)){

			node_guarding_request();
			
			requestData();
        
        	encoder_sensor_->getData(sensor_data_homing);
        	    
			auto limit_switch_pos = ((sensor_data_homing["io_stat"].asInt() & (1 << 0)) >> 0);
			auto limit_switch_neg = ((sensor_data_homing["io_stat"].asInt() & (1 << 1)) >> 1);
			
			if((limit_switch_pos == 1)){  
				counts_offset_ = sensor_data_homing["counts"].asInt();
				set_relative_position(counts_offset_);
				set_profile_velocity(0.0);
				homing_achieved = true;
				logger_->info("[{}] Positive limit switch detected. Homing done!",motor_name_);
				return true;
			}

			if((limit_switch_neg == 1) ){  
				counts_offset_ = sensor_data_homing["counts"].asInt();
				set_relative_position(counts_offset_);
				set_profile_velocity(0.0);
				homing_achieved = true;
				logger_->info("[{}] Negative limit switch detected. Homing done!",motor_name_);
				return true;
			}
        	

        	time_passed_response_received_lift_down = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - recovery_lift_down_time);
        	std::this_thread::sleep_for(std::chrono::microseconds(2000));

    	}

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		logger_->info(" current count [{}], count_offset [{}]", sensor_data_homing["counts"].asInt(), counts_offset_);
		// set_relative_position(counts_offset_);
		
    	if(!homing_achieved){
    	    logger_->error("[{}] Homing timeout", motor_name_);
    	    return false;
    	}

    	else{
    	    logger_->info("[{}] Homing achieved", motor_name_);
    	    return true;
		} 

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		return true;
	}
	return true;
}

int AmcMotorActuator::initMotor(){
    // 
    int err = 0;

	// err |= motorControlword(motor_id_, Disable_Voltage);

	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node); 
	err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Reset_Comunication); 
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Reset_Node); 

    err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational); 
	if (err != 0) {
        logger_->debug("Err in NMT_change_state : NMT_Enter_PreOperational for {}", motor_name_);
		return MOTOR_ERROR;
	}
             
	err |= motorConfigNode(motor_id_);
	if (err != 0) {
		logger_->debug("Err in motorConfigNode for {}", motor_name_);
		return MOTOR_ERROR;
	}

	err |= motorSetmode(Motor_mode_Velocity); 
	if (err != 0) {
		logger_->debug("Err in motorSetmode for {}", motor_name_);
		return MOTOR_ERROR;
	}

	return 0;

}

int AmcMotorActuator::motorConfigNode(int motor_id){
    int err = 0;
	int num_PDOs;

    //Clear error - required step for EROB motors
	// err |= motorControlword(motor_id, Reset_Fault);
	resetFault();
	amc_drive_reset();
    //set the communication parameter for TPDO - transmission on 1 SYNC
	err |= motor_Transmit_PDO_n_Parameter(motor_id, 0x1802);
	err |= motor_Transmit_PDO_n_Parameter(motor_id, 0x1815);
    err |= motor_Transmit_PDO_n_Parameter(motor_id, 0x1816);
    err |= motor_Transmit_PDO_n_Parameter(motor_id, 0x1819);
	err |= amc_motor_Receive_PDO_n_Parameter(motor_id, 0x1415);

	err |= setTPDO_cobid(motor_id, 0x1802, 1);
	err |= setTPDO_cobid(motor_id, 0x1815, 2);
	err |= setTPDO_cobid(motor_id, 0x1816, 3);
	err |= setTPDO_cobid(motor_id, 0x1819, 4);

	// set the communication parameter for RPDO - transmission on 1 SYNC
	err |= set_vel_RPDO_cobid(motor_id, 0x1415);

	num_PDOs = 4;
    Epos_pdo_mapping status_and_vol[] = {
            {0x2002, 0x02, 16},   // drive protection status
            {0x2002, 0x03, 16},   // system protection status
		    {0x200F, 0x01, 16},   // drive voltage
            {0x2023, 0x01, 16}    // i/o status
	};
    err |= amc_motor_Transmit_PDO_n_Mapping(motor_id, num_PDOs, status_and_vol);

	return err;

}

int AmcMotorActuator::motorControlword(uint16_t motor_id, enum Epos_ctrl ctrl) {
	
    SDO_data d;
	d.nodeid = motor_id;
	d.index = 0x6040;
	d.subindex = 0x00;
	d.data.size = 2;
	d.data.data = ctrl;

	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int AmcMotorActuator::motor_Transmit_PDO_n_Parameter(uint16_t node_id, uint16_t index) {

	SDO_data d;
	d.nodeid = node_id;
	d.index = index;
	d.subindex = 0x02;
	d.data.size = 1;
	d.data.data = 0x01;
	
	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int AmcMotorActuator::amc_motor_Receive_PDO_n_Parameter(uint16_t node_id, uint16_t index) {

	SDO_data d;
	d.nodeid = node_id;
	d.index = index;
	d.subindex = 0x02;
	d.data.size = 1;
	d.data.data = 0x01;
	
	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int AmcMotorActuator::amc_motor_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t num_objects, Epos_pdo_mapping* objects) {

	
	int err = 0;

	// Set number of mapped objects to zero
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x1A19;
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = 0;
	// err = SDO_write_no_wait(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	// if (err != 0) {
		// return err;
	// }

	// Write objects
	d.data.size = 4;
	for(int i=0; i<num_objects; i++) {
		Epos_pdo_mapping obj = objects[i];

		d.subindex = i + 1;
		d.data.data = ((uint32_t)obj.index << 16) | ((uint32_t)obj.subindex<<8) | ((uint32_t)obj.length);
		err = SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
		if (err != 0) {
			return err;
		}
	}

	// Set Correct number of objects
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = num_objects;
	
	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int AmcMotorActuator::setTPDO_cobid(uint16_t node_id, uint16_t index, uint8_t n){

	auto tpdo_id = 0x00000180;

	if(n==1){
		tpdo_id = 0x00000180;
	} else if (n==2) {
		tpdo_id = 0x00000280;
	} else if (n==3) {
		tpdo_id = 0x00000380;
	} else if (n==4) {
		tpdo_id = 0x00000480;
	} else {
		logger_->error("TPDO setting: PDO number not recognized for {}", motor_name_);
	}
	
	SDO_data d;
	d.nodeid = node_id;
	d.index = index;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = tpdo_id + node_id;
	
	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

}

int AmcMotorActuator::set_vel_RPDO_cobid(uint16_t node_id, uint16_t index){

	auto rpdo_id = 0x00000500;
	
	SDO_data d;
	d.nodeid = node_id;
	d.index = index;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = rpdo_id + node_id;
	
	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

}

int AmcMotorActuator::motorSetmode(enum Motor_mode mode){

	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6060;
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = mode;

	err |= SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;

}

int AmcMotorActuator::enableMotor(void) { 
	int err = 0;
	//Stop PDO-communication
	err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);

	err |= motorControlword(motor_id_, Shutdown);
	err |= motorControlword(motor_id_, Switch_On); 
   	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);

	//Open PDO-communication
	err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Start_Node);

	return err;
}

int AmcMotorActuator::disableMotor(void) {
	int err = 0;

	//Stop PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);

	encoder_sensor_->stop_read_thread();
	set_vel_speed(motor_id_, axis_, 0.0);
	std::this_thread::sleep_for(std::chrono::microseconds(50000));
	err |= motorControlword(motor_id_, Disable_Voltage);
	
	//Close PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}


int AmcMotorActuator::haltMotor(void) {
	int err = 0;

	//Stop PDO-communication
	err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Quickstop);
	
	//Close PDO-communication
	err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int AmcMotorActuator::resetFault(void) {
	int err = 0;

	//Stop PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Reset_Fault);
	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);
	
	//Close PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int AmcMotorActuator::amc_drive_reset(void) {
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2001;
	d.subindex = 0x01;
	d.data.size = 2;
	d.data.data = 4096;

	return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int AmcMotorActuator::reinitializeMotor(void) {
	int err = 0;

	enable_brake(true);	

	err |= motorControlword(motor_id_, Disable_Voltage);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	logger_->info("Reinitializing motor: {}", motor_name_);
    err |= initMotor();
   	err |= enableMotor();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	set_guard_time(motor_id_,50);
    set_life_time_factor(motor_id_,6);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// if(mode_of_operation_ == "velocity")
	// {
        motorSetmode(Motor_mode_Velocity);
		set_profile_velocity(default_max_velocity_);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		set_profile_acc(default_acceleration_);
		set_profile_deacc(default_deceleration_);
		set_PTPC(default_acceleration_);
		set_PTNC(default_deceleration_);
		set_NTNC(default_acceleration_);
		set_NTPC(default_deceleration_);
		set_vel_speed(motor_id_, axis_, 0.0);
		logger_->info("[{}] Motor mode [velocity]. Setting zero velocity done!",motor_name_);
    // }

	// if (mode_of_operation_ == "position")
	// {
		// set_profile_velocity(default_max_velocity_);
		// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		// set_profile_acc(default_acceleration_);
		// set_profile_deacc(default_deceleration_);
		// motorSetmode(Motor_mode_Position);
		// logger_->info("[{}] Motor mode [position]",motor_name_);
	// }

    return err;
}

int AmcMotorActuator::quickStopMotor(void) {
	int err = 0;

	//Stop PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Quickstop);
	
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	enable_brake(false);
	//Close PDO-communication
	// err |= NMT_change_state(amc_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int AmcMotorActuator::set_target_velocity(float vel) {
	int err = 0;
	// DS1
	int32_t drive_val =  ((int32_t)rpm_to_countspersec(vel*axis_)) * (pow(2, 17)/(encoder_sensor_->ki * encoder_sensor_->ks));
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x60FF;
	// d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = drive_val;
	err |= SDO_write_no_wait(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;
}

int AmcMotorActuator::set_profile_velocity(float vel) {
	int err = 0;

	int64_t drive_val = (int64_t)((rpm_to_countspersec(vel)) * (pow(2, 33)/(encoder_sensor_->ks))); //Convert to DS3
	int64_to_bytes.int_val = drive_val;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x09;
	d.data.size = 4;
	d.data.data = 8;
	// init data transfer
	err |= SDO_write_multi_byte(amc_motor_actuator_sockets_->motor_cfg_fd, &d, int64_to_bytes.byte_val);	
	return err;
}

int AmcMotorActuator::set_profile_acc(float acc) {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x0A;
	d.data.size = 4;
	int64_t max_speed = rpm_to_countspersec(default_max_velocity_);
	auto acc_scaled = (motor_rps2_to_cps2(acc) * (pow(2,28) / (max_speed * encoder_sensor_->ks)));
	auto acc_val = int32_t(acc_scaled);
	d.data.data = acc_val; //Convert to DA3 //encoder_sensor_->kms
	err |= SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);	
	return err;
}

int AmcMotorActuator::set_profile_deacc(float deacc) {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x0B;
	d.data.size = 4;
	int64_t max_speed = rpm_to_countspersec(default_max_velocity_);
	d.data.data = (int32_t)(motor_rps2_to_cps2(deacc) * (pow(2,28) / (max_speed * encoder_sensor_->ks))); //Convert to DA3 //encoder_sensor_->kms
	err |= SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_PTPC(float acc) {
	int err = 0;

	int64_to_bytes.int_val = (int32_t)motor_rps2_to_cps2(acc) * (pow(2,34)/ (encoder_sensor_->ki * (pow((encoder_sensor_->ks), 2)))); //Convert to DA2

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = 6;
	err |= SDO_write_multi_byte(amc_motor_actuator_sockets_->motor_cfg_fd, &d, int64_to_bytes.byte_val);

	return err;
}

int AmcMotorActuator::set_PTNC(float deacc) {
	int err = 0;
	int64_to_bytes.int_val = (int32_t)motor_rps2_to_cps2(deacc) * (pow(2,34)/ (encoder_sensor_->ki * (pow((encoder_sensor_->ks), 2)))); //Convert to DA2

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x02;
	d.data.size = 4;
	d.data.data = 6;
	err |= SDO_write_multi_byte(amc_motor_actuator_sockets_->motor_cfg_fd, &d, int64_to_bytes.byte_val);

	return err;
}
int AmcMotorActuator::set_NTNC(float acc) {
	int err = 0;
	int64_to_bytes.int_val = (int32_t)motor_rps2_to_cps2(acc) * (pow(2,34)/ (encoder_sensor_->ki * (pow((encoder_sensor_->ks), 2)))); //Convert to DA2

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x03;
	d.data.size = 4;
	d.data.data = 6;
	err |= SDO_write_multi_byte(amc_motor_actuator_sockets_->motor_cfg_fd, &d, int64_to_bytes.byte_val);

	return err;
}

int AmcMotorActuator::set_NTPC(float deacc) {
	int err = 0;
	int64_to_bytes.int_val = (int32_t)motor_rps2_to_cps2(deacc) * (pow(2,34)/ (encoder_sensor_->ki * (pow((encoder_sensor_->ks), 2)))); //Convert to DA2

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x203C;
	d.subindex = 0x04;
	d.data.size = 4;
	d.data.data = 6;
	err |= SDO_write_multi_byte(amc_motor_actuator_sockets_->motor_cfg_fd, &d, int64_to_bytes.byte_val);

	return err;
}
int AmcMotorActuator::rpm_to_countspersec(float rpm) {
	int counts_per_sec = static_cast<int>((rpm*AMC_CPR)/60.0);
	return counts_per_sec;
}

int AmcMotorActuator::motor_rps2_to_cps2(float rpss) {
    int m_cps2 = (int)(rpss * AMC_CPR);
    return m_cps2;
}

int AmcMotorActuator::enable_motion_profile() {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6086;
	d.subindex = 0x00;
	d.data.size = 2;
	d.data.data = 2;
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::enable_brake(bool is_enabled) {
	int err = 0;
	int trigger = 0;
	if (is_enabled) {
		trigger = 1;
	}
	else {
		trigger = 0;
	}

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x205A;
	d.subindex = 0x01;
	d.data.size = 2;
	d.data.data = trigger;
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_relative_position(int32_t pos) {
	// logger_->info("[{}] Motor mode [position]. Setting position",motor_name_);
	
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x607A;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)(pos);
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_vel_speed(uint16_t nodeid, int axis, float vel) {
	logger_->info("[{}] Motor mode [velocity]. Setting velocity",motor_name_);

    int err = 0;
    // const int32_t countspersec = axis * rpm_to_countspersec(vel);//motor_rpm_to_cps(axis * vel);
	int32_t drive_val =  vel * (pow(2, 17)/(encoder_sensor_->ki * encoder_sensor_->ks));
	Socketcan_t target_vel[1] = {
            {4, drive_val}};
    err = PDO_send(amc_motor_actuator_sockets_->motor_vel_read_pdo_fd, PDO_RX4_ID + nodeid, 1, target_vel);
	logger_->info("ki = [{}], ks = [{}], drive_val = [{}].", encoder_sensor_->ki, encoder_sensor_->ks, drive_val);		
    return err;
}

int AmcMotorActuator::set_guard_time(uint16_t motor_id, uint16_t value) {
    SDO_data d;
    d.nodeid = motor_id;
    d.index = 0x100C;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = value;

    return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

}

int AmcMotorActuator::set_life_time_factor(uint16_t motor_id, uint8_t value) {
    SDO_data d;
    d.nodeid = motor_id;
    d.index = 0x100D;
    d.subindex = 0x00;
    d.data.size = 1;
    d.data.data = value;

    return SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);

}

int AmcMotorActuator::set_position_kp(float kp) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2038;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = (float)(kp * pow(2, 32)); //(Position Loop Proportional Gain) x 2^32
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_position_ki(float ki) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2038;
	d.subindex = 0x02;
	d.data.size = 4;
	d.data.data = (float)(ki * (pow(2, 41) / (encoder_sensor_->ks / 2))); //(Position Loop Integral Gain) x (2^41 / Vpos), Vpos = (Switching Frequency / 2)
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_position_kd(float kd) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2038;
	d.subindex = 0x03;
	d.data.size = 4;
	d.data.data = (float)(kd * (pow(2, 28) * (encoder_sensor_->ks / 2))); //(Position Loop Derivative Gain) x (2^28 * Vpos), Vpos = (Switching Frequency / 2)
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_velocity_kp(float kp) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2036;
	d.subindex = 0x03;
	d.data.size = 4;
	d.data.data = (float)(kp * (((pow(2, 16) * (encoder_sensor_->ks / 2) * 1) / (2 * encoder_sensor_->kp)))); //(Velocity Loop Proportional Gain) x ((2^16 * Vvel * Rppv) / (2 * Cpk)), Vvel = (Switching Frequency / 2), Rppv = Interpolation Value, Cpk = Peak Current
	logger_->info("[{}] peak current# = [{}]", motor_name_, encoder_sensor_->kp);
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_velocity_ki(float ki) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2036;
	d.subindex = 0x04;
	d.data.size = 4;
	d.data.data = (float)(ki * ((pow(2, 32) * 1) / (2 * encoder_sensor_->kp))); //(Velocity Loop Integral Gain) x (2^32 * Rppv) / (2 * Cpk), Rppv = Interpolation Value, Cpk = Peak Current
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::set_velocity_kd(float kd) {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x2036;
	d.subindex = 0x05;
	d.data.size = 4;
	d.data.data = (float)(kd * ((pow(2, 16) * pow((encoder_sensor_->ks / 2),2) * 1) / (2 * encoder_sensor_->kp))); //(Velocity Loop Derivative Gain) x ((2^16 * (Vvel)^2 * Rppv) / (2 * Cpk)), Vvel = (Switching Frequency / 2), Rppv = Interpolation Value, Cpk = Peak Current
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

int AmcMotorActuator::store_params_to_drive() {
	int err = 0;
	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x1010;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = 1702257011;
	
	err |=  SDO_write(amc_motor_actuator_sockets_->motor_cfg_fd, &d);
	return err;
}

void AmcMotorActuator::goToInitPos(){

	// set_profile_velocity(100);
	// set_profile_acc(8);
	// set_profile_deacc(8);

	int err = 0;
	err |= set_relative_position(0);
	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);
	// err |= motorControlword(motor_id_, Start_Excercise);

}

void AmcMotorActuator::sendNodeGuardingRequest(){

	Socketcan_t data[1];
    uint32_t cob_id;
    uint32_t node_id;

    data[0].size = 0;
    data[0].data = 0x00;

    node_id = motor_id_;
    node_id = ( node_id | (1 << 30) );
    cob_id = NMT_TX + node_id;

	// logger_->info("[{}] Guard sending request", motor_name_);

    socketcan_write(amc_motor_actuator_sockets_->nmt_motor_cfg_fd, cob_id, 1, data);

}

void AmcMotorActuator::requestData(){
	
	encoder_sensor_->motor_request();

}

void AmcMotorActuator::changeActuatorControlMode(Json::Value &actuator_control_mode){

	if (actuator_control_mode["action"].asString() == "change_control_mode"){
        if (actuator_control_mode["control_mode"].asString() == "motor_reset"){
            logger_->info("Fault reset for : [{}]",motor_name_);
            resetFault();
			amc_drive_reset();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_quick_stop"){
            logger_->info("Quick stop for : [{}]",motor_name_);
            quickStopMotor();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_disable"){
            logger_->info("Disable action for : [{}]",motor_name_);
            disableMotor();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_enable"){
            logger_->info("Enable action for : [{}]",motor_name_);
            enableMotor();
            
        }
        else{
            logger_->info("Control Mode not recognized for : [{}]",motor_name_);
        }

    }
    else{
        logger_->info("Control Mode Action not recognized for : [{}]",motor_name_);
    }


}

double AmcMotorActuator::radianToDegree(double rad){
	double degrees = rad * (180/3.14);
	return degrees;
}

double AmcMotorActuator::degreeToRadian(double deg){
	double radian = deg * (3.14/180);
	return radian;
}

