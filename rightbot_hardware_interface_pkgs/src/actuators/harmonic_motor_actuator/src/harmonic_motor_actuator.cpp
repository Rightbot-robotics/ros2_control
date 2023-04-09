//
// Created by amrapali on 1/19/23.
//

#include "harmonic_motor_actuator/harmonic_motor_actuator.hpp"
PLUGINLIB_EXPORT_CLASS(HarmonicMotorActuator, hardware_interface::ActuatorInterface)


HarmonicMotorActuator::HarmonicMotorActuator() {

    // motor initializations
    // logger_ = spdlog::get("hardware_interface")->clone("harmonic_motor_actuator");

    // harmonic_motor_actuator_sockets_ = motor_sockets;

	// motor_id_ = harmonic_motor_actuator_sockets_->motor_id_;
    // motor_name_ = harmonic_motor_actuator_sockets_->motor_name_;

	// previous_mode = "not_set";

	// initMotor();
	// logger_->info("{}, initialization done", motor_name_);
	// enableMotor();
	// logger_->info("{}, enabled", motor_name_);

	// goToInitPos();

}

HarmonicMotorActuator::~HarmonicMotorActuator(){

}

CallbackReturn HarmonicMotorActuator::on_init(const hardware_interface::HardwareInfo & info){
    // We hardcode the info
    // logger_ = spdlog::get("hardware_interface")->clone("harmonic_motor_actuator");
   
    // logger_->info(" Harmonic Motor Actuator Init");
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    motor_id_ = stoi(info.joints[0].parameters.at("can_id"));
    motor_name_ = info_.joints[0].name;
    axis_ = stoi(info.joints[0].parameters.at("axis"));

	std::string config_path;
	// init_json(config_path);
    
    default_max_velocity_ = stod(info.joints[0].parameters.at("default_max_velocity"));
    default_acceleration_ = stod(info.joints[0].parameters.at("default_max_accleration"));
    std::cout << "default_max_velocity_: " << default_max_velocity_ << std::endl;
    std::cout << "default_acceleration_: " << default_acceleration_ << std::endl;

    // can only control in position 
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    
    if (command_interfaces.size() != 3)
    {
        // logger_->error("[{}] - Incorrect number of command interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }

    for (const auto & command_interface : command_interfaces)
    {
        if (
            (command_interface.name != hardware_interface::HW_IF_POSITION) &&
            (command_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (command_interface.name != hardware_interface::HW_IF_ACCELERATION)
        )
       {
            // logger_->error("[{}] - Incorrect type of command interfaces", motor_name_);
            
            return CallbackReturn::ERROR;
       }

    }

    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() != 6)
    {
        // logger_->error("[{}] - Incorrect number of state interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
        if (
            (state_interface.name != hardware_interface::HW_IF_POSITION) &&
            (state_interface.name != hardware_interface::HW_IF_VELOCITY) &&
            (state_interface.name != hardware_interface::HW_IF_STATUS) && 
            (state_interface.name != hardware_interface::HW_IF_ERROR_CODE) &&
            (state_interface.name != hardware_interface::HW_IF_NODE_GUARD_ERROR) &&
			(state_interface.name != hardware_interface::HW_IF_ACTUAL_MOTOR_CURRENT)
			)
       {
            // logger_->error("[{}] - Incorrect type of state interfaces", motor_name_);

            return CallbackReturn::ERROR;
       }

    }
    // fprintf(stderr, "TestSingleJointActuator configured successfully.\n");

    // logger_->info("[{}] - Intialiazation successful", motor_name_);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HarmonicMotorActuator::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    previous_mode = "not_set";

    harmonic_motor_actuator_sockets_ = std::make_shared<HarmonicMotorActuatorSockets>(motor_id_, motor_name_);
    
    initMotor();
	// logger_->info("{}, initialization done", motor_name_);

    encoder_sensor_ = std::make_shared<HarmonicEncoderSensor>();

    encoder_sensor_->initialize(harmonic_motor_actuator_sockets_);
	// enableMotor();
	// logger_->info("{}, enabled", motor_name_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    

    Json::Value config_data;
    config_data["motor_id"] = motor_id_;
    config_data["motor_name"] = motor_name_;
    config_data["motor_axis"] = axis_;

    // harmonic_encoder_sensor = std::make_shared<HarmonicEncoderSensor>();
    // harmonic_encoder_sensor->initialize(config_data, motor_sockets_);
    // read_motor_data_thread_ = std::thread(&HarmonicMotorActuator::readMotorData, this);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HarmonicMotorActuator::on_activate(const rclcpp_lifecycle::State & previous_state){

    // logger_->info("Motor Enable action for: [{}]",motor_name_);
    enableMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

	
	std::cout << "setting default_max_velocity_: " << default_max_velocity_ << std::endl;
	set_profile_velocity(default_max_velocity_);
    
	std::cout << "setting default_acceleration_: " << default_acceleration_ << std::endl;
	set_profile_acc(default_acceleration_);
	set_profile_deacc(default_acceleration_);

    return CallbackReturn::SUCCESS;

}

CallbackReturn HarmonicMotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    // logger_->info("Motor Disable action for: [{}]",motor_name_);
    disableMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> HarmonicMotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_STATUS, &status_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_ERROR_CODE, &error_code_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_NODE_GUARD_ERROR, &node_guard_error_state_));
	state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_ACTUAL_MOTOR_CURRENT, &actual_motor_current_state_));

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> HarmonicMotorActuator::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &max_velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_ACCELERATION, &acceleration_command_));

    return command_interfaces;


}

hardware_interface::return_type HarmonicMotorActuator::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

	// std::cout << "Motor Harmonic Actuator read: " << motor_name_ <<std::endl;
	
    encoder_sensor_->getData(sensor_data);

    if(sensor_data["read_status"].asBool() == false){
        // return hardware_interface::return_type::ERROR;
    }
    
    status_state_ = sensor_data["status"].asInt();
    error_code_state_ = sensor_data["err_code"].asInt();
	actual_motor_current_state_ = sensor_data["actual_motor_current"].asDouble();

    position_state_ = sensor_data["counts"].asInt();
    velocity_state_ = sensor_data["velocity"].asDouble();

    node_guard_error_state_ = sensor_data["guard_err"].asInt();

	// std::cout << "status_state_: " << status_state_ <<std::endl;
	// std::cout << "actual_motor_current_state_: " << actual_motor_current_state_ <<std::endl;
	// std::cout << "error_code_state_: " << error_code_state_ <<std::endl;
	// std::cout << "position_state_: " << position_state_ <<std::endl;
	// std::cout << "velocity_state_: " << velocity_state_ <<std::endl;
	// std::cout << "node_guard_error_state_: " << node_guard_error_state_ <<std::endl;


    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HarmonicMotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

	// std::cout << "Motor Harmonic Actuator write" << std::endl;

	if(previous_max_velocity_command_ != max_velocity_command_){
		
		
		if(!using_default_max_velocity_){
			std::cout << "setting set_profile_velocity: " << max_velocity_command_ << std::endl;
			set_profile_velocity(max_velocity_command_);
		}
	}

    if((acceleration_command_ > (previous_acceleration_command_ + acceleration_epsilon)) || (acceleration_command_ < (previous_acceleration_command_ - acceleration_epsilon))){
		if((acceleration_command_ > (0 + acceleration_epsilon)) || (acceleration_command_ < (0 - acceleration_epsilon))){
			std::cout << "setting set_profile_acc: " << acceleration_command_ << std::endl;

			double degree_per_sec = (acceleration_command_*(180/3.14));
			double revolution_per_sec = abs(degree_per_sec/360.0);
			std::cout << "setting revolution_per_sec: " << revolution_per_sec << std::endl;
			if(!using_default_acceleration_){
				set_profile_acc(acceleration_command_);
				set_profile_deacc(acceleration_command_);
			}
		}
	}

    if(previous_position_command_ != position_command_){
		double angle_in_degree = (position_command_*(180/3.14));
		int counts = static_cast<uint32_t>((angle_in_degree/360)*motor_ppr_);
		std::cout << "setting set_relative_position: " << position_command_ <<  ". counts: " << counts << std::endl;
        set_relative_position( counts);
    }

	

	
    
    previous_position_command_ = position_command_;
    previous_max_velocity_command_ = max_velocity_command_;
    previous_acceleration_command_ = acceleration_command_;


    return hardware_interface::return_type::OK;
}

CallbackReturn HarmonicMotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

CallbackReturn HarmonicMotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

    return CallbackReturn::SUCCESS;

}

void HarmonicMotorActuator::init_json(std::string path){

    Json::Value config_data;
    JsonRead config_parser("/home/rightbot/test_ws/src/ros2_control/rightbot_hardware_interface_pkgs/src/config/config.json");

    if (!config_parser.parse())
    throw std::invalid_argument("Parsing error in config of Controller Manager");

    config_parser.getValue(config_data);

    if(config_data["harmonic_motor_actuator"]["using_default_max_velocity"].asString() == "yes"){
        using_default_max_velocity_ = true;
        default_max_velocity_ = config_data["harmonic_motor_actuator"]["default_max_velocity"].asDouble();
        std::cout << "default_max_velocity_: " << default_max_velocity_ << std::endl;
    }
    else{
        using_default_max_velocity_ = false;
    }

    if(config_data["harmonic_motor_actuator"]["using_default_acceleration"].asString() == "yes"){
        using_default_acceleration_ = true;
        default_acceleration_ = config_data["harmonic_motor_actuator"]["default_acceleration"].asDouble();
        std::cout << "default_acceleration_: " << default_acceleration_ << std::endl;
    }
    else{
        using_default_acceleration_ = false;
    }

}

int HarmonicMotorActuator::initMotor(){
    // 
    int err = 0;

	// err |= motorControlword(motor_id_, Disable_Voltage);

	// err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node); 
	// err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Reset_Comunication); 
	// err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Reset_Node); 

    err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational); 
	if (err != 0) {
        // logger_->debug("Err in NMT_change_state : NMT_Enter_PreOperational for {}", motor_name_);
		return MOTOR_ERROR;
	}
             
	err |= motorConfigNode(motor_id_);
	if (err != 0) {
		// logger_->debug("Err in motorConfigNode for {}", motor_name_);
		return MOTOR_ERROR;
	}

	err |= motorSetmode(Motor_mode_Position); 
	if (err != 0) {
		// logger_->debug("Err in motorSetmode for {}", motor_name_);
		return MOTOR_ERROR;
	}

	return 0;

}

int HarmonicMotorActuator::motorConfigNode(int motor_id){
    int err = 0;
	int num_PDOs;

    //Clear error - required step for EROB motors
	err |= motorControlword(motor_id, Clear_Fault);

    //set the communication parameter for TPDO - transmission on 1 SYNC
	err |= motor_Transmit_PDO_n_Parameter(motor_id, 1, PDO_TX1_ID + motor_id);
	err |= motor_Transmit_PDO_n_Parameter(motor_id, 2, PDO_TX2_ID + motor_id);
    err |= motor_Transmit_PDO_n_Parameter(motor_id, 3, PDO_TX3_ID + motor_id);
    // err |= motor_Transmit_PDO_n_Parameter(motor_id, 4, PDO_TX4_ID + motor_id);

	// PDO TX1 Statusword and High Voltage Reference
    num_PDOs = 3;
    Epos_pdo_mapping status_and_err[] = {
            {0x6041, 0x00, 16},	// Statusword
			{0x603F, 0x00, 16},	// Error Code
			{0x6078, 0x00, 16}	// Actual Motor Current
            // {0x6079, 0x00, 32}	// High Voltage Reference
    };
    err |= motor_Transmit_PDO_n_Mapping(motor_id, 1, num_PDOs, status_and_err);

    // PDO TX2 velocity
    num_PDOs = 1;
    Epos_pdo_mapping vel[] = {
            {0x606C, 0x00, 32} // Speed feedback
    };
    err |= motor_Transmit_PDO_n_Mapping(motor_id, 2, num_PDOs, vel);

    // PDO TX3 Encoder Counts
    num_PDOs = 1;
    Epos_pdo_mapping enc[] = {
            {0x6064, 0x00, 32} // Position Actual value
    };
    err |= motor_Transmit_PDO_n_Mapping(motor_id, 3, num_PDOs, enc);
    //err |= motor_Transmit_PDO_n_Mapping(motor_id, 3, 0, NULL);

    // PDO TX4 Manufacturer Status and Latched Fault
    num_PDOs = 2;
    Epos_pdo_mapping manufacturer_status[] = {
            {0x1002, 0x00, 32},   // Manfa Statusword
            {0x2183, 0x00, 32}    // Latching Fault Status Register
    };
    //err |= motor_Transmit_PDO_n_Mapping(motor_id, 4, num_PDOs, manufacturer_status);

	// err |= set_tpdo1_cobid(motor_id);

	err |= setTPDO_cobid(motor_id, 1);
	err |= setTPDO_cobid(motor_id, 2);
	err |= setTPDO_cobid(motor_id, 3);
	// err |= setTPDO_cobid(motor_id, 4);

	//err |= motor_Transmit_PDO_n_Mapping(motor_id, 5, 0, NULL);
	//err |= motor_Transmit_PDO_n_Mapping(motor_id, 6, 0, NULL);
	//err |= motor_Transmit_PDO_n_Mapping(motor_id, 7, 0, NULL);

	return err;

}

int HarmonicMotorActuator::motorControlword(uint16_t motor_id, enum Epos_ctrl ctrl) {
	
    SDO_data d;
	d.nodeid = motor_id;
	d.index = 0x6040;
	d.subindex = 0x00;
	d.data.size = 2;
	d.data.data = ctrl;

	return SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int HarmonicMotorActuator::motor_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob) {

	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x1800 + n-1;
	d.subindex = 0x02;
	d.data.size = 1;
	d.data.data = 0x01;
	
	return SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int HarmonicMotorActuator::motor_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping* objects) {

	
	int err = 0;

	// Set number of mapped objects to zero
	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x1A00 + n-1;
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = 0;
	err = SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
	if (err != 0) {
		return err;
	}

	// Write objects
	d.data.size = 4;
	for(int i=0; i<num_objects; i++) {
		Epos_pdo_mapping obj = objects[i];

		d.subindex = i+1;
		d.data.data = ((uint32_t)obj.index << 16) | ((uint32_t)obj.subindex<<8) | ((uint32_t)obj.length);
		err = SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
		if (err != 0) {
			return err;
		}
	}

	// Set Correct number of objects
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = num_objects;
	
	return SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int HarmonicMotorActuator::set_tpdo1_cobid(uint16_t node) {
	SDO_data d;
	d.nodeid = node;
	d.index = 0x1800;
	d.subindex = 0x01;
	d.data.size = 4;
	d.data.data = 0x00000180 + node;

	return SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);
}

int HarmonicMotorActuator::setTPDO_cobid(uint16_t node_id, uint8_t n){

	auto tpdo_id = 0x00000180;

	SDO_data d;
	d.nodeid = node_id;
	d.index = 0x1800 + n-1;
	d.subindex = 0x01;
	d.data.size = 4;

	if(n==1){
		tpdo_id = 0x00000180;
	} else if (n==2) {
		tpdo_id = 0x00000280;
	} else if (n==3) {
		tpdo_id = 0x00000380;
	} else if (n==4) {
		tpdo_id = 0x00000480;
	} else {
		// logger_->error("TPDO setting: PDO number not recognized for {}", motor_name_);
	}

	d.data.data = tpdo_id + node_id;
	
	return SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

}

int HarmonicMotorActuator::motorSetmode(enum Motor_mode mode){

	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6060;
	d.subindex = 0x00;
	d.data.size = 1;
	d.data.data = mode;

	err |= SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;

}

int HarmonicMotorActuator::enableMotor(void) { 
	int err = 0;
	//Stop PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);

	err |= motorControlword(motor_id_, Shutdown);
	err |= motorControlword(motor_id_, Switch_On); 
   	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);

	//Open PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Start_Node);

	return err;
}

int HarmonicMotorActuator::disableMotor(void) {
	int err = 0;

	//Stop PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Disable_Voltage);
	
	//Close PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}


int HarmonicMotorActuator::haltMotor(void) {
	int err = 0;

	//Stop PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Quickstop);
	
	//Close PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int HarmonicMotorActuator::resetFault(void) {
	int err = 0;

	//Stop PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Clear_Fault);
	
	//Close PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int HarmonicMotorActuator::quickStopMotor(void) {
	int err = 0;

	//Stop PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Enter_PreOperational);
	err |= motorControlword(motor_id_, Quickstop);
	
	//Close PDO-communication
	err |= NMT_change_state(harmonic_motor_actuator_sockets_->motor_cfg_fd, motor_id_, NMT_Stop_Node);

	return err;
}

int HarmonicMotorActuator::set_profile_velocity(float vel) {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6081;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)rpm_to_countspersec(vel);
	err |= SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;
}

int HarmonicMotorActuator::set_profile_acc(float acc) {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6083;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)motor_rps2_to_cps2(acc);
	err |= SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;
}

int HarmonicMotorActuator::set_profile_deacc(float deacc) {
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x6084;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)motor_rps2_to_cps2(deacc);
	err |= SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;
}

int HarmonicMotorActuator::rpm_to_countspersec(float rpm) {
	int counts_per_min = (int)((rpm)*(EROB_CPR));
	return counts_per_min/60.0;
}

int HarmonicMotorActuator::motor_rps2_to_cps2(float rpss) {
    int m_cps2 = (int)(rpss * EROB_CPR);
    return m_cps2;
}

int HarmonicMotorActuator::set_relative_position(int32_t pos) {
	
	int err = 0;

	SDO_data d;
	d.nodeid = motor_id_;
	d.index = 0x607A;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)pos*axis_;
	err |=  SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);
	err |= motorControlword(motor_id_, Start_Relative);// for trigger

	return err;
}


void HarmonicMotorActuator::writeData(Json::Value &actuator_data){

    actuator_data_["timeout"] = actuator_data["timeout"];
    actuator_data_["mode"] = actuator_data["mode"];
    actuator_data_["velocity"] = actuator_data["velocity"];
    actuator_data_["relative_pos"] = actuator_data["relative_pos"];
    actuator_data_["max_vel"] = actuator_data["max_vel"];
    actuator_data_["accel"] = actuator_data["accel"];
    actuator_data_["decel"] = actuator_data["decel"];

	auto command_type = actuator_data_["mode"].asString();
	auto max_vel = actuator_data_["max_vel"].asDouble();;
	auto accel = actuator_data_["accel"].asDouble();
	auto decel = actuator_data_["decel"].asDouble();
	auto pos = actuator_data_["relative_pos"].asDouble();

	if (command_type == "velocity") {

	}
	else if ((previous_mode == "position") && (command_type == "position")) {

		// logger_->info("'Write Data in position mode for motor [{}]",harmonic_motor_actuator_sockets_->motor_name_);
		set_relative_position(static_cast<int32_t>(pos));
	}
	else if((previous_mode != "position") && (command_type == "position")){

		// logger_->info("'Write Data in position mode for motor [{}]",harmonic_motor_actuator_sockets_->motor_name_);
		motorSetmode(Motor_mode_Position);
        set_profile_velocity(max_vel);
    	set_profile_acc(accel);
    	set_profile_deacc(decel);
        set_relative_position(static_cast<int32_t>(pos));
    
	}
	else{
		// logger_->info("'Write Data mode [{}] not recognized for motor [{}]",command_type, harmonic_motor_actuator_sockets_->motor_name_);
	}

	previous_mode = command_type;

}

void HarmonicMotorActuator::goToInitPos(){

	set_profile_velocity(8);
	set_profile_acc(8);
	set_profile_deacc(8);

	int err = 0;
	err |= set_relative_position(0);
	err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);
	err |= motorControlword(motor_id_, Start_Excercise);

}

void HarmonicMotorActuator::sendNodeGuardingRequest(){

	Socketcan_t data[1];
    uint32_t cob_id;
    uint32_t node_id;

    data[0].size = 0;
    data[0].data = 0x00;

    node_id = motor_id_;
    node_id = ( node_id | (1 << 30) );
    cob_id = NMT_TX + node_id;

	// logger_->info("[{}] Guard sending request", motor_name_);

    // socketcan_write(harmonic_motor_actuator_sockets_->nmt_motor_cfg_fd, cob_id, 1, data);

}

void HarmonicMotorActuator::requestData(){

}

void HarmonicMotorActuator::changeActuatorControlMode(Json::Value &actuator_control_mode){

	if (actuator_control_mode["action"].asString() == "change_control_mode"){
        if (actuator_control_mode["control_mode"].asString() == "motor_reset"){
            // logger_->info("Fault reset for : [{}]",motor_name_);
            resetFault();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_quick_stop"){
            // logger_->info("Quick stop for : [{}]",motor_name_);
            quickStopMotor();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_disable"){
            // logger_->info("Disable action for : [{}]",motor_name_);
            disableMotor();
            
        }
        else if (actuator_control_mode["control_mode"].asString() == "motor_enable"){
            // logger_->info("Enable action for : [{}]",motor_name_);
            enableMotor();
            
        }
        else{
            // logger_->info("Control Mode not recognized for : [{}]",motor_name_);
        }

    }
    else{
        // logger_->info("Control Mode Action not recognized for : [{}]",motor_name_);
    }


}

