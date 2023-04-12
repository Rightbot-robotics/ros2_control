//
// Created by amrapali on 11/30/22.
//

#ifndef HARMONIC_MOTOR_ACTUATOR_PARAMS_HPP
#define HARMONIC_MOTOR_ACTUATOR_PARAMS_HPP

#define MOTOR_SOCKETS_ERROR   (-1)
#define MOTOR_ERROR       (-1)
#define EROB_TIMEOUT	(-2)

/*** Motor CPR ***/
#define EROB_CPR	524287 

typedef struct {
	uint16_t index;
	uint8_t subindex;
	uint8_t length;
} Epos_pdo_mapping;

enum Epos_ctrl {
	Shutdown = 0x06,	//servo free -> servo ready (Ready to Switch On)
	Switch_On = 0x07,	//servo ready (Ready to Switch On) -> wait to turn on servo enable (Switched On)
	Switch_On_And_Enable_Operation = 0x0f,	//wait to turn on servo enable (Switched On) -> servo run (Operation Enable)
	Disable_Voltage = 0x00,	//wait to turn on servo enable (Switched On) -> servo free
	Quickstop = 0x02, //fast shutdown -> servo operation
	Disable_Operation = 0x07,
	Enable_Operation = 0x0f,
	Clear_Fault = 0x80, //clear motor fault during initialization
	Start_Excercise = 0x1F, //start excercise (profile position mode)
	Start_Relative = 0x5F,
	Start_Excercise_Pos_Immediate = 0x3F, 
	Start_Relative_Pos_Immediate = 0x7F
};

enum Motor_mode {
	Motor_mode_Velocity = 3, //Contour speed mode=3 - mode selection
	Motor_mode_Position = 1, //Contour position mode=1 - mode selection
	Motor_mode_Interpolated_Position_Mode = 7
    
};

#endif //HARMONIC_MOTOR_ACTUATOR_PARAMS_HPP
