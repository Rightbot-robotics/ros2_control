#ifndef AMC_MOTOR_ACTUATOR_PARAMS_HPP
#define AMC_MOTOR_ACTUATOR_PARAMS_HPP

#define MOTOR_SOCKETS_ERROR   (-1)
#define MOTOR_ERROR       (-1)
#define AMC_TIMEOUT	(-2)

/*** Motor CPR ***/
#define AMC_CPR	4096 

typedef struct {
	uint16_t index;
	uint8_t subindex;
	uint8_t length;
} Epos_pdo_mapping;

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

enum Motor_mode {
	Motor_mode_Velocity = 3, //Contour speed mode=3 - mode selection
	Motor_mode_Position = 1    
};

#endif //AMC_MOTOR_ACTUATOR_PARAMS_HPP
