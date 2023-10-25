#ifndef MOTOR_HPP_
#define MOTOR_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#include <canopen_pkg/canopen.h>
#include <motor/sockets.hpp>
#include <motor/motor_controls.hpp>
#include <motor/motor_feedback.hpp>

# define MOTOR_ERROR (-1)

class Motor {

private:
    std::string motor_name;
    int motor_id;

    typedef struct {
        uint16_t index;
        uint8_t subindex;
        uint8_t length;
    } Epos_pdo_mapping;

    enum Motor_mode {
        Motor_mode_Velocity = 3,
        Motor_mode_Position = 1,

    };

    enum Epos_ctrl {
        Shutdown = 0x06,
        Switch_On = 0x07,
        Switch_On_And_Enable_Operation = 0x0f,
        Disable_Voltage = 0x00,
        Quickstop = 0x02,
        Disable_Operation = 0x07,
        Enable_Operation = 0x0f,
        Position_Trigger = 0x5f,
        Reset_Fault = 0x80,
    };

public:
    Motor(Sockets::SocketsSPtr motor_sockets_);

    ~Motor();

    typedef std::shared_ptr<Motor> MotorSPtr;
    typedef std::unique_ptr<Motor> MotorUPtr;

    Sockets::SocketsSPtr motor_sockets;

    MotorControls::MotorControlsSPtr motor_controls;
    MotorFeedback::MotorFeedbackSPtr motor_feedback;

    std::shared_ptr<spdlog::logger> logger_;


    bool motor_init(int motor_id);

    int motor_config_node(int motor_id);

    int motor_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob);

    int motor_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping *objects);

    int motor_setmode_sdo(uint16_t motor_id, enum Motor_mode mode);

    int set_guard_time(uint16_t motor_id, uint16_t value);

    int set_life_time_factor(uint16_t motor_id, uint8_t value);

    int motor_enable(int motor_id);

    int motor_disable(int motor_id);

    int motor_reset(int motor_id);

    int motor_quick_stop(int motor_id);

    int motor_request(void);

    int motion_profile_type(uint16_t value);

    //***Motor Control***///
    // MotorControls::position_cmd_t position_cmd_element_;
    // MotorControls::velocity_cmd_t velocity_cmd_element_;

    bool motorCommand(int motor_id, std::string command_type, MotorControls::position_cmd_t position_cmd_element,
                      MotorControls::velocity_cmd_t velocity_cmd_element);

    uint16_t status_register_fb_[1] = {0};
    float battery_vol_fb_[1] = {0};
    uint32_t encoder_fb_[1] = {0};
    double vel_fb_[1] = {0};
    uint32_t manufacturer_reg_fb_[1] = {0};
    uint32_t latched_fault_fb_[1] = {0};

    bool nmtChangeState(std::string state_name);

};

#endif
