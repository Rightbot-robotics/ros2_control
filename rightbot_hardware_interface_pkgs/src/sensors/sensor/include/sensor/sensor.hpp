#ifndef SENSOR_H_
#define SENSOR_H_

#include <iostream>
#include <memory>
#include <json_reader/json_read.h>

class Sensor{

    public:

        typedef std::shared_ptr<Sensor> SensorSPtr;
        typedef std::unique_ptr<Sensor> SensorUPtr;

        Sensor();
        ~Sensor();

        virtual void getData(Json::Value &sensor_data) = 0;


};

#endif //SENSOR_H_