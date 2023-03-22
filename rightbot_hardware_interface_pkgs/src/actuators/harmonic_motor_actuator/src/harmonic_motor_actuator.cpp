//
// Created by amrapali on 1/19/23.
//

#include "harmonic_motor_actuator/harmonic_motor_actuator.hpp"
#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(HarmonicMotorActuator, hardware_interface::ActuatorInterface)

HarmonicEncoderData::HarmonicEncoderData() {
    status_m = 0;
    err_code_m =0;
    pos_m = 0;
    vel_m = 0.0;
    guard_err_m = 0;
    time_sys = 0;
    read_status_err_code = false;
    read_status_encoder = false;
    read_status_velocity = false;

}

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
    logger_ = spdlog::get("hardware_interface")->clone("harmonic_motor_actuator");
   
    logger_->info(" Harmonic Motor Actuator Init");
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      
      return CallbackReturn::ERROR;
    }

    // motor_id_ = stoi(info_.hardware_parameters["motor_id"]);
    // motor_name_ = stoi(info_.hardware_parameters["motor_name"]);
    // axis_ = stoi(info_.hardware_parameters["motor_axis"]);

    motor_id_ = 12;
    motor_name_ = "left_wheel";

    // fill motor data from json file here

    // can only control one joint
    // if (info_.joints.size() != 1)
    // {
    //   return CallbackReturn::ERROR;
    // }

    // can only control in position 
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    
    if (command_interfaces.size() != 1)
    {
        logger_->error("[{}] - Incorrect command interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
        
        return CallbackReturn::ERROR;
    }

    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() != 2)
    {
        logger_->error("[{}] - Incorrect state interfaces", motor_name_);
        return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
        if (
            (state_interface.name != hardware_interface::HW_IF_POSITION) &&
            (state_interface.name != hardware_interface::HW_IF_VELOCITY))
       {
            
            return CallbackReturn::ERROR;
       }

    }
    // fprintf(stderr, "TestSingleJointActuator configured successfully.\n");

    logger_->info("[{}] - Intialiazation successful", motor_name_);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HarmonicMotorActuator::on_configure(const rclcpp_lifecycle::State & previous_state){
    
    previous_mode = "not_set";

    initMotor();
	logger_->info("{}, initialization done", motor_name_);
	enableMotor();
	logger_->info("{}, enabled", motor_name_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    

    Json::Value config_data;
    config_data["motor_id"] = motor_id_;
    config_data["motor_name"] = motor_name_;
    config_data["motor_axis"] = axis_;

    // harmonic_encoder_sensor = std::make_shared<HarmonicEncoderSensor>();
    // harmonic_encoder_sensor->initialize(config_data, motor_sockets_);
    read_motor_data_thread_ = std::thread(&HarmonicMotorActuator::readMotorData, this);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn HarmonicMotorActuator::on_activate(const rclcpp_lifecycle::State & previous_state){

    logger_->info("Motor Enable action for: [{}]",motor_name_);
    enableMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));


}

CallbackReturn HarmonicMotorActuator::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    logger_->info("Motor Disable action for: [{}]",motor_name_);
    disableMotor();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

}

std::vector<hardware_interface::StateInterface> HarmonicMotorActuator::export_state_interfaces(){
    
    // We can read a position and a velocity
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &status_state_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &error_code_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &node_guard_error_state_));

    return state_interfaces;


}

std::vector<hardware_interface::CommandInterface> HarmonicMotorActuator::export_command_interfaces(){

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &position_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &max_velocity_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &acceleration_command_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      motor_name_, hardware_interface::HW_IF_POSITION, &deceleration_command_));

    return command_interfaces;


}

hardware_interface::return_type HarmonicMotorActuator::read(const rclcpp::Time & time, const rclcpp::Duration & period) {

    Json::Value sensor_data;
    getData(sensor_data);

    if(sensor_data["read_status"].asBool() == false){
        return hardware_interface::return_type::ERROR;
    }
    
    status_state_ = sensor_data["status"].asInt();
    error_code_state_ = sensor_data["err_code"].asInt();

    position_state_ = sensor_data["counts"].asInt();
    velocity_state_ = sensor_data["velocity"].asDouble();

    node_guard_error_state_ = sensor_data["guard_err"].asInt();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HarmonicMotorActuator::write(const rclcpp::Time & time, const rclcpp::Duration & period) {

    if(previous_position_command_ != position_command_){
        set_relative_position( static_cast<uint32_t>( position_command_), motor_id_);
    }

    if(previous_max_velocity_command_ != max_velocity_command_){
        set_profile_velocity(max_velocity_command_);
    }

    if(previous_acceleration_command_ != acceleration_command_){
        set_profile_acc(acceleration_command_);
    }

    if(previous_deceleration_command_ != deceleration_command_){
        set_profile_deacc(deceleration_command_);
    }
    
    previous_position_command_ = position_command_;
    previous_max_velocity_command_ = max_velocity_command_;
    previous_acceleration_command_ = acceleration_command_;
    previous_deceleration_command_ = deceleration_command_;

    return hardware_interface::return_type::OK;
}

CallbackReturn HarmonicMotorActuator::on_shutdown(const rclcpp_lifecycle::State & previous_state){

}

CallbackReturn HarmonicMotorActuator::on_error(const rclcpp_lifecycle::State & previous_state){

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
        logger_->debug("Err in NMT_change_state : NMT_Enter_PreOperational for {}", motor_name_);
		return MOTOR_ERROR;
	}
             
	err |= motorConfigNode(motor_id_);
	if (err != 0) {
		logger_->debug("Err in motorConfigNode for {}", motor_name_);
		return MOTOR_ERROR;
	}

	err |= motorSetmode(Motor_mode_Position); 
	if (err != 0) {
		logger_->debug("Err in motorSetmode for {}", motor_name_);
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
    num_PDOs = 2;
    Epos_pdo_mapping status_and_err[] = {
            {0x6041, 0x00, 16},	// Statusword
			{0x603F, 0x00, 16}	// Error Code
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
		logger_->error("TPDO setting: PDO number not recognized for {}", motor_name_);
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
	d.data.data = (int32_t)rpm_to_countspersec(acc);
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
	d.data.data = (int32_t)rpm_to_countspersec(deacc);
	err |= SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	return err;
}

int HarmonicMotorActuator::rpm_to_countspersec(float rpm) {
	int counts_per_min = (int)((rpm)*(EROB_CPR));
	return counts_per_min/60.0;
}

int HarmonicMotorActuator::set_relative_position(int32_t pos, uint16_t nodeid) {
	
	int err = 0;

	SDO_data d;
	d.nodeid = nodeid;
	d.index = 0x607A;
	d.subindex = 0x00;
	d.data.size = 4;
	d.data.data = (int32_t)pos;
	err |=  SDO_write(harmonic_motor_actuator_sockets_->motor_cfg_fd, &d);

	// err |= motorControlword(motor_id_, Switch_On_And_Enable_Operation);
	// err |= motorControlword(motor_id_, Start_Relative);// for trigger

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

		logger_->info("'Write Data in position mode for motor [{}]",harmonic_motor_actuator_sockets_->motor_name_);
		set_relative_position(static_cast<int32_t>(pos),motor_id_);
	}
	else if((previous_mode != "position") && (command_type == "position")){

		logger_->info("'Write Data in position mode for motor [{}]",harmonic_motor_actuator_sockets_->motor_name_);
		motorSetmode(Motor_mode_Position);
        set_profile_velocity(max_vel);
    	set_profile_acc(accel);
    	set_profile_deacc(decel);
        set_relative_position(static_cast<int32_t>(pos),motor_id_);
    
	}
	else{
		logger_->info("'Write Data mode [{}] not recognized for motor [{}]",command_type, harmonic_motor_actuator_sockets_->motor_name_);
	}

	previous_mode = command_type;

}

void HarmonicMotorActuator::goToInitPos(){

	set_profile_velocity(8);
	set_profile_acc(8);
	set_profile_deacc(8);

	int err = 0;
	err |= set_relative_position(0, motor_id_);
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
            logger_->info("Fault reset for : [{}]",motor_name_);
            resetFault();
            
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

int HarmonicMotorActuator::motor_status_n_voltage_read(int motor_id, uint16_t *status, uint16_t *err_code, int timeout) {
    int err;
    my_can_frame f;
    err = PDO_read(harmonic_motor_actuator_sockets_->motor_status_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX1_ID + motor_id)) {
        *status = (f.data[0] << 0) | (f.data[1] << 8);
        *err_code = (f.data[2] << 0) | (f.data[3] << 8);
        // *battery_vol = ((uint32_t)f.data[4]<<0) | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24);
        // logger_->debug("test battery vol: [{}]", *battery_vol);

    }

    return err;

}

int HarmonicMotorActuator::motor_enc_read(int motor_id, int32_t *pos, int timeout) {
    int err;
    my_can_frame f;
    uint32_t enc;

    err = PDO_read(harmonic_motor_actuator_sockets_->motor_enc_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX3_ID + motor_id)) {
        //ENCODER COUNT
        enc = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
              ((uint32_t) f.data[3] << 24);
        //rpm = ((uint32_t)f.data[4]<<0) | ((uint32_t)f.data[5]<<8) | ((uint32_t)f.data[6]<<16) | ((uint32_t)f.data[7]<<24);
        // logger_->debug("Harmonic motor counts init: [{}]", enc);
        
        *pos = enc;
        // if (init_enc) {
        //     *pos = -enc - err_enc;
        // } else {
        //     err_enc = -enc;
        //     init_enc = true;
        // }
        //*vel = rpm*0.1;//motor_rpm_to_mmsec(-rpm);
    }

    return err;
}

int HarmonicMotorActuator::motor_vel_read(int motor_id, double *vel, int timeout) {
    int err;
    my_can_frame f;
    int32_t rpm;
    int32_t register_cps;
    double cps;

    err = PDO_read(harmonic_motor_actuator_sockets_->motor_vel_pdo_fd, &f, timeout);

    if (err != 0) {
        // Read error, or no data
        return err;
    }

    if (f.id == (PDO_TX2_ID + motor_id)) {
        //RPM OF LEFT
        register_cps = ((uint32_t) f.data[0] << 0) | ((uint32_t) f.data[1] << 8) | ((uint32_t) f.data[2] << 16) |
                       ((uint32_t) f.data[3] << 24);
        cps = register_cps;
        *vel = (double) motor_cps_to_rpm(cps);
    }

    return err;
}

double HarmonicMotorActuator::motor_cps_to_rpm(double counts_per_sec) {

    double m_per_sec = (counts_per_sec) / EROB_CPR;
    return m_per_sec * 60;
}

int HarmonicMotorActuator::node_guarding_response_read(uint16_t *response, int timeout){
    int err;
    my_can_frame f;
    err = PDO_read(harmonic_motor_actuator_sockets_->nmt_motor_cfg_fd, &f, timeout);

    if(err != 0) {
        // loop executed when read error, or no data
        return err;
    }

    if(f.data[0] <= 0){
        return -1;
    }

    if (f.id == (NMT_TX + motor_id_)) {
        *response = ((uint16_t)f.data[0]);
    }

    return 0;
}


int HarmonicMotorActuator::readData(HarmonicEncoderData *encoder_data) {

    uint16_t status_register_fb_[1]= {0};
    uint16_t err_code_fb_[1] = {0};
    int32_t encoder_fb_[1]= {0};
    double vel_fb_[1]= {0};
    int guard_err_fb_= -1;

    auto err_pdo_1_ = motor_status_n_voltage_read(motor_id_, status_register_fb_, err_code_fb_, 1);
    auto err_pdo_2_ = motor_enc_read(motor_id_, encoder_fb_, 1);
    auto err_pdo_3_ = motor_vel_read(motor_id_, vel_fb_, 1);
    
    guard_err_fb_ = err_pdo_2_;

    if (0 == err_pdo_1_) {

        encoder_data->status_m = status_register_fb_[0];
        encoder_data->err_code_m = err_code_fb_[0];
        encoder_data->read_status_err_code = true;

    }
    else{
        encoder_data->read_status_err_code = false;
    }

    if (0 == err_pdo_2_) {
        encoder_data->pos_m = encoder_fb_[0];
        // logger_->debug("Encoder_position: [{}]", encoder_fb_[0]);
        encoder_data->read_status_encoder = true;
    }
    else{
        encoder_data->read_status_encoder = false;
    }

    if (0 == err_pdo_3_) {
        encoder_data->vel_m = vel_fb_[0];
        // logger_->debug("Encoder_Velocity: [{}]", vel_fb_[0]);
        encoder_data->read_status_velocity = true;
    }
    {
        encoder_data->read_status_velocity = false;
    }

    encoder_data->guard_err_m = guard_err_fb_;

    auto now = std::chrono::high_resolution_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    long duration = value.count();
    encoder_data->time_sys = duration;

    bool status = encoder_data->read_status_encoder;
    int rt_value = -1;
    if(true == status) {
        rt_value = 0;
	logger_->debug("{} pos read successful",harmonic_motor_actuator_sockets_->motor_name_);
    }
    return 0;

}

void HarmonicMotorActuator::readMotorData() {


    while (true) {

        auto start_time = std::chrono::system_clock::now();

        {
            
            if(reading_loop_started) {
                int err = readData( &encoder_data_);

                if (err == 0) {

                    read_mutex_.lock();
                    q_encoder_data_.push_back(encoder_data_);
                    read_mutex_.unlock();
                }
                else {
                    logger_->warn("incomplete data received, not pushing to sensor data q");
                }
            }
            

        }

        auto time_passed_in_read = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now() - start_time);
        logger_->debug("Time in execution [ readMotorData() ]: [{}] us", time_passed_in_read.count());

        std::this_thread::sleep_for(std::chrono::microseconds(10000 - time_passed_in_read.count()));

    }

}

void HarmonicMotorActuator::getData(Json::Value &sensor_data) {

    read_mutex_.lock();
    reading_loop_started = true;

    if (!q_encoder_data_.empty()) {


        auto encoder_data_q_element = q_encoder_data_.back();
        q_encoder_data_.pop_front();

        logger_->debug("Read deque size after pop: {}", q_encoder_data_.size());
        if (q_encoder_data_.size() > 10) {
            //logger_->error("Read deque size : [{}]", q_encoder_data_.size());
            q_encoder_data_.clear();
        }

        sensor_data["status"] = encoder_data_q_element.status_m;
        sensor_data["err_code"] = encoder_data_q_element.err_code_m;
        sensor_data["counts"] = encoder_data_q_element.pos_m;
        sensor_data["velocity"] = encoder_data_q_element.vel_m;
        sensor_data["timestamp"] = std::to_string(encoder_data_q_element.time_sys);
        sensor_data["guard_err"] = encoder_data_q_element.guard_err_m;
        sensor_data["read_status"] = true;
        sensor_data["read_status_err_code"] = encoder_data_q_element.read_status_err_code;
        sensor_data["read_status_encoder"] = encoder_data_q_element.read_status_encoder;
        sensor_data["read_status_velocity"] = encoder_data_q_element.read_status_velocity;
        
        logger_->debug("[{}] Status: [{}], Error Code: [{}]", harmonic_motor_actuator_sockets_->motor_name_, encoder_data_q_element.status_m, encoder_data_q_element.err_code_m);
        logger_->debug("[{}] Position: {} counts, Velocity: {} rpm", harmonic_motor_actuator_sockets_->motor_name_, encoder_data_q_element.pos_m, encoder_data_q_element.vel_m);
        logger_->debug("[{}] Guard Err: {}", harmonic_motor_actuator_sockets_->motor_name_, encoder_data_q_element.guard_err_m);

    } else {
        sensor_data["read_status"] = false;
        logger_->debug("[{}] Sensor Data Queue Empty. ", harmonic_motor_actuator_sockets_->motor_name_);
    }


    read_mutex_.unlock();
}
