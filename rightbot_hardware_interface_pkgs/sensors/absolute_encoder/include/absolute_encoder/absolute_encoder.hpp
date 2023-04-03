//
// Created by amrapali on 11/30/22.
//

#ifndef ABSOLUTE_ENCODER_HPP
#define ABSOLUTE_ENCODER_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <deque>

#include <sensor/sensor.hpp>
#include <json_reader/json_read.h>

#include "absolute_encoder/absolute_encoder_sockets.hpp"

class AbsoluteEncoder : public Sensor {

public:

    typedef std::shared_ptr <AbsoluteEncoder> AbsoluteEncoderSPtr;
    typedef std::unique_ptr <AbsoluteEncoder> AbsoluteEncoderUPtr;

    AbsoluteEncoder(int sensor_id);

    ~AbsoluteEncoder();

    void getData(Json::Value &sensor_data);

    int requestData();

    // 
    int readEncCounts(float* angle);

private:
    int initSensor();
    int enableSensor();
    float convertToAngle(int counts);
    void readData();

    typedef struct {
        uint16_t index;
        uint8_t subindex;
        uint8_t length;
    } Epos_pdo_mapping;

    typedef struct feedback_ {
        float angle;
        uint64_t time_sys;
        bool read_status;

    } feedback_s;

    int encoder_Transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob);
    int encoder_Transmit_PDO_n_Mapping(uint16_t node_id, uint8_t n, uint8_t num_objects, Epos_pdo_mapping *objects);


    std::shared_ptr<spdlog::logger> logger_;
    AbsoluteEncoderSockets::AbsoluteEncoderSocketsSPtr absolute_encoder_sockets_;

    std::deque<feedback_s> q_angle_data_;
    std::mutex read_mutex_;
    std::thread read_enc_data_thread_;

    int sensor_id_;
    int absolute_encoder_init_pos;
    int abs_motor_ppr;

    bool reading_loop_started;
    

};

#endif //ABSOLUTE_ENCODER_HPP
