#include <motor/motor_controls.hpp>

MotorControls::MotorControls(Sockets::SocketsSPtr motor_sockets_) {
    motor_sockets = motor_sockets_;
    // logger_ = spdlog::get("hardware_interface")->clone("motor_controls");
    previous_mode = "not_set";
}

MotorControls::~MotorControls() {

}

int MotorControls::motor_request(void) {

    Socketcan_t data[1];
    data[0].size = 1;
    data[0].data = 0x00;
    return socketcan_write(motor_sockets->motor_sync_fd, 128, 1, data);
}

int MotorControls::motor_rpm_to_cps(float rpm) {
    int m_cps = rpm * DRIVING_MOTOR_RPM_TO_COUNTS_PER_SEC;
    return m_cps;
}


int MotorControls::motor_rps2_to_cps2(float rpss) // RPS2 to 10 counts/sec2 conversion
{
    int m_cps2 = rpss * MOTOR_ACC_MULTIPLIER;
    return m_cps2;
}

int MotorControls::set_vel_speed(uint16_t nodeid, int axis, float vel) {
    int err = 0;
    const int32_t rpm = motor_rpm_to_cps(axis * vel);
    Socketcan_t target_vel[2] = {
            {2, Switch_On_And_Enable_Operation},
            {4, rpm}};

    err = PDO_send(motor_sockets->motor_vel_pdo_fd, PDO_RX4_ID + nodeid, 2, target_vel);
    return err;
}

int MotorControls::set_relative_position(uint16_t node_id, int axis, uint32_t position) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x607A;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = axis * position;//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.switch_on = 1;
    md_control_register.control_s.enable_voltage = 1;
    md_control_register.control_s.quick_stop = 1;
    md_control_register.control_s.enable_operation = 1;
    md_control_register.control_s.absolute_or_relative = 0; // 1 for relative
    md_control_register.control_s.new_setpoint = 0;

    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.absolute_or_relative = 1;
    md_control_register.control_s.new_setpoint = 1;
    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);
    return err;
}

int MotorControls::set_relative_position_immediate(uint16_t node_id, int axis, uint32_t position) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x607A;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = axis * position;//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.switch_on = 1;
    md_control_register.control_s.enable_voltage = 1;
    md_control_register.control_s.quick_stop = 1;
    md_control_register.control_s.enable_operation = 1;
    md_control_register.control_s.absolute_or_relative = 0; // 1 for relative
    md_control_register.control_s.new_setpoint = 0;
    md_control_register.control_s.instantaneous_change_set = 1;

    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.absolute_or_relative = 1;
    md_control_register.control_s.new_setpoint = 1;
    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);
    return err;
}

int MotorControls::set_absolute_position(uint16_t node_id, int axis, uint32_t position) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x607A;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = axis * position;//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.switch_on = 1;
    md_control_register.control_s.enable_voltage = 1;
    md_control_register.control_s.quick_stop = 1;
    md_control_register.control_s.enable_operation = 1;
    md_control_register.control_s.absolute_or_relative = 0; // 1 for relative
    md_control_register.control_s.new_setpoint = 0;
    md_control_register.control_s.instantaneous_change_set = 1;

    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    md_control_register.control_s.absolute_or_relative = 0;
    md_control_register.control_s.new_setpoint = 1;
    d.index = 0x6040;
    d.subindex = 0x00;
    d.data.size = 2;
    d.data.data = md_control_register.control_word;
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);
    return err;
}

int MotorControls::set_profile_velocity(uint16_t node_id, uint32_t speed) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x6081;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = motor_rpm_to_cps(speed);//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    return err;
}

int MotorControls::set_profile_acc(uint16_t node_id, float acc) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x6083;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = motor_rps2_to_cps2(acc);//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    return err;
}

int MotorControls::set_profile_deacc(uint16_t node_id, float deacc) {
    int err = 0;

    SDO_data d;
    d.nodeid = node_id;
    d.index = 0x6084;
    d.subindex = 0x00;
    d.data.size = 4;
    d.data.data = motor_rps2_to_cps2(deacc);//in milliseconds
    err |= SDO_write(motor_sockets->motor_cfg_fd, &d);

    return err;
}

int MotorControls::set_driving_motor_position_mode_params(uint16_t node_id, double position_loop_acc,
                                                          double position_loop_deacc, double position_loop_speed) {
    int err = 0;
    err |= set_profile_velocity(node_id, motor_rpm_to_cps(position_loop_speed));
    err |= set_profile_acc(node_id, motor_rps2_to_cps2(position_loop_acc));
    err |= set_profile_deacc(node_id, motor_rps2_to_cps2(position_loop_deacc));
    return err;
}

bool MotorControls::motor_command(int motor_id, int axis, std::string command_type, position_cmd_t position_cmd_element,
                                  velocity_cmd_t velocity_cmd_element) {
    // logger_->debug("Motor Sockets ptr in motor controls : {}", motor_sockets);

    if ((previous_mode == "velocity") && (command_type == "velocity")){

        set_vel_speed(motor_id, axis, velocity_cmd_element.velocity);

    }
    else if((previous_mode != "velocity") && (command_type == "velocity")){

        motor_setmode_sdo(motor_id, Motor_mode_Velocity);
        set_driving_motor_position_mode_params(motor_id, velocity_cmd_element.accel, velocity_cmd_element.decel,
                                              velocity_cmd_element.max_vel);
        set_vel_speed(motor_id, axis, velocity_cmd_element.velocity);

    }
    else if((previous_mode == "position") && (command_type == "position")){

        set_driving_motor_position_mode_params(motor_id, position_cmd_element.accel, position_cmd_element.decel,
                                               position_cmd_element.max_vel);
        set_relative_position(motor_id, axis, static_cast<int32_t>(position_cmd_element.relative_pos));

    }
    else if((previous_mode != "position") && (command_type == "position")){
        motor_setmode_sdo(motor_id, Motor_mode_Position);
        set_driving_motor_position_mode_params(motor_id, position_cmd_element.accel, position_cmd_element.decel,
                                               position_cmd_element.max_vel);

        set_relative_position(motor_id, axis, static_cast<int32_t>(position_cmd_element.relative_pos));
    }
    else if((previous_mode == "position_immediate") && (command_type == "position_immediate")){
        
        // set_driving_motor_position_mode_params(motor_id, position_cmd_element.accel, position_cmd_element.decel,
                                            //    position_cmd_element.max_vel);

        set_relative_position_immediate(motor_id, axis, static_cast<int32_t>(position_cmd_element.relative_pos));

    }
    else if((previous_mode != "position_immediate") && (command_type == "position_immediate")){
        
        motor_setmode_sdo(motor_id, Motor_mode_Position);
        set_driving_motor_position_mode_params(motor_id, position_cmd_element.accel, position_cmd_element.decel,
                                               position_cmd_element.max_vel);

        set_relative_position_immediate(motor_id, axis, static_cast<int32_t>(position_cmd_element.relative_pos));

    }
    else{
        // logger_->error("Motor command not recognized");
    }

    previous_mode = command_type;

}

int MotorControls::motor_setmode_sdo(uint16_t motor_id, enum Motor_mode mode) {
    int err = 0;

    SDO_data d;
    d.nodeid = motor_id;
    d.index = 0x6060;
    d.subindex = 0x00;
    d.data.size = 1;
    d.data.data = mode;

    return SDO_write(motor_sockets->motor_cfg_fd, &d);
}

int MotorControls::nodeGuardingRequest(){

    Socketcan_t data[1];
    uint32_t cob_id;
    uint32_t node_id;

    data[0].size = 0;
    data[0].data = 0x00;

    node_id = motor_sockets->motor_can_id;
    node_id = ( node_id | (1 << 30) );
    cob_id = NMT_TX + node_id;

    return socketcan_write(motor_sockets->nmt_motor_cfg_fd, cob_id, 1, data);

}