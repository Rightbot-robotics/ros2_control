#ifndef MOTOR_SENSOR_HPP_
#define MOTOR_SENSOR_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

class MotorSensor{

    private:
    std::string motor_name;
    int motor_id;
    std::shared_ptr<spdlog::logger> logger_;

    public:
    MotorSensor();
    ~MotorSensor();

    typedef std::shared_ptr<MotorSensor> MotorSensorSPtr;
    typedef std::unique_ptr<MotorSensor> MotorSensorUPtr;

    uint16_t status;
    float battery_vol;
    int32_t position;
    double vel
    uint32_t manufacturer_reg; 
    uint32_t latched_fault;

};

#endif