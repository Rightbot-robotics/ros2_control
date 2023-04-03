#include <gls_barcode/gls_barcode.hpp>


GlsBarcode::GlsBarcode(int barcode_id, int barcode_base_id) {

    barcode_sockets_ = std::make_shared<BarcodeSockets>(barcode_id, barcode_base_id);

    logger_ = spdlog::get("hardware_interface")->clone("gls_barcode");

//    logger_->info("Barcode Sockets ptr in Gls Barcode constructor: {}", barcode_sockets_);

    x_fb_[1] = {0};
    y_fb_[1] = {0};
    ang_fb_[1] = {0};
    sensor_time_fb_[1] = {0};
    decode_time_fb_[1] = {0};
    tag_x_fb_[1] = {0};
    tag_y_fb_[1] = {0};
    flag_b_ = 0;

    err_pdo_ = 0;

}

GlsBarcode::~GlsBarcode() {

}

int GlsBarcode::gls_transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob) {
    int err = 0;
    SDO_data d;

    d.nodeid = node_id;
    d.index = 0x1800 + n - 1;

    // d.subindex = 0x02;
    // d.data.size = 1;
    // d.data.data = 0xFE;
    // err |= SDO_write(gls_cfg_fd, &d);

    // d.subindex = 0x05;
    // d.data.size = 1;
    // d.data.data = 10;
    // err |= SDO_write(gls_cfg_fd, &d);

    return err;
}

int GlsBarcode::gls_config_node(int barcode_id) {

    int err = 0;
    int num_PDOs;

    // PDO COB-IDs. Change PDO parameters.
    err |= gls_transmit_PDO_n_Parameter(barcode_id, 1, PDO_TX1_ID + barcode_id);
    err |= gls_transmit_PDO_n_Parameter(barcode_id, 2, PDO_TX2_ID + barcode_id);
    err |= gls_transmit_PDO_n_Parameter(barcode_id, 3, PDO_TX3_ID + barcode_id);
    err |= gls_transmit_PDO_n_Parameter(barcode_id, 4, PDO_TX4_ID + barcode_id);

    return err;
}


bool GlsBarcode::gls_init(int barcode_id) {

    int err = 0;
    // logger_->debug("Motor Sockets ptr in init: {}", motor_sockets);

    err |= NMT_change_state(barcode_sockets_->gls_cfg_fd, barcode_id, NMT_Enter_PreOperational);
    if (err != 0) {
        return GLS_ERROR;
    }

    // err |= motor_config_node(motor_id);
    // if (err != 0)
    // {
    // 	return MOTOR_ERROR;
    // }

    return err;
}

int GlsBarcode::gls_enable(int barcode_id) {
    int err = 0;

    // Open PDO-communication
    err |= NMT_change_state(barcode_sockets_->gls_cfg_fd, barcode_id, NMT_Start_Node);
    if (err != 0) {
        return GLS_ERROR;
    }

    return err;
}

int GlsBarcode::gls_request(int barcode_id) {

    int err = 0;
    Socketcan_t trigger_on[1] = {
            {1, 0x01}
    };
    err |= PDO_send(barcode_sockets_->gls_one_pdo_fd, PDO_RX1_ID + barcode_id, 1, trigger_on);

    usleep(35000);
    Socketcan_t trigger_off[1] = {
            {1, 0x00}
    };
    err |= PDO_send(barcode_sockets_->gls_one_pdo_fd, PDO_RX1_ID + barcode_id, 1, trigger_off);

    return err;
}


int GlsBarcode::gls_data_read(double *x, double *y, double *ang, uint64_t *sensor_time, uint64_t *decode_time,
                              uint64_t *tag_x, uint64_t *tag_y, int *flag, int timeout) {

    int err1, err2, err3, err4, err5, err6, err7;
    int err = 0;

    volatile static uint8_t data_capture = 0;
    volatile static uint8_t counter = 0;
    volatile static uint8_t prev_counter = 0;

    static uint8_t result[70];

    my_can_frame f1, f2, f3, f4, f5, f6, f7;

    err1 = PDO_read(barcode_sockets_->gls_one_pdo_fd, &f1, timeout);
    err2 = PDO_read(barcode_sockets_->gls_two_pdo_fd, &f2, timeout);
    err3 = PDO_read(barcode_sockets_->gls_three_pdo_fd, &f3, timeout);
    err4 = PDO_read(barcode_sockets_->gls_four_pdo_fd, &f4, timeout);
    err5 = PDO_read(barcode_sockets_->gls_five_pdo_fd, &f5, timeout);
    err6 = PDO_read(barcode_sockets_->gls_six_pdo_fd, &f6, timeout);
    err7 = PDO_read(barcode_sockets_->gls_seven_pdo_fd, &f7, timeout);

    err = err1 | err2 | err3 | err4 | err5 | err6 | err7;

    if (err1 != 0 && err2 != 0 && err3 != 0 && err4 != 0 && err5 != 0 && err6 != 0 && err7 != 0) {
        *flag = 2;
        *ang = -1;
        data_capture = 0;
        return err;

    }

    if (err1 == 0) {
        //std::cout << "Entered PDO 1" << std::endl;

        if (f1.data[0] != counter) {
            counter = f1.data[0];
            data_capture = 0;
            // std::cout << "PDO 1 - New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i] = f1.data[i + 1];
            // std::cout << "f1   :" << f1.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err2 == 0) {
        //std::cout << "Entered PDO 2" << std::endl;
        if (f2.data[0] != counter) {
            counter = f2.data[0];
            data_capture = 0;
            // std::cout << "PDO 2 - New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i + 7] = f2.data[i + 1];
            // std::cout << "f2   :" << f2.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err3 == 0) {
        //std::cout << "Entered PDO 3" << std::endl;
        if (f3.data[0] != counter) {
            counter = f3.data[0];
            data_capture = 0;
            // std::cout << "PDO 3 - New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i + 14] = f3.data[i + 1];
            // std::cout << "f3   :" << f3.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err4 == 0) {
        //std::cout << "Entered PDO 4" << std::endl;

        if (f4.data[0] != counter) {
            counter = f4.data[0];
            data_capture = 0;
            // std::cout << "PDO 4 -New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i + 21] = f4.data[i + 1];
            // std::cout << "f4   :" << f4.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err5 == 0) {
        //std::cout << "Entered PDO 4" << std::endl;

        if (f5.data[0] != counter) {
            counter = f5.data[0];
            data_capture = 0;
            // std::cout << "PDO 4 -New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i + 28] = f5.data[i + 1];
            // std::cout << "f5   :" << f5.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err6 == 0) {
        //std::cout << "Entered PDO 4" << std::endl;

        if (f6.data[0] != counter) {
            counter = f6.data[0];
            data_capture = 0;
            // std::cout << "PDO 4 -New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            result[i + 35] = f6.data[i + 1];
            // std::cout << "f6   :" << f6.data[i+1] << std::endl;
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    if (err7 == 0) {
        //std::cout << "Entered PDO 4" << std::endl;

        if (f7.data[0] != counter) {
            counter = f7.data[0];
            data_capture = 0;
            // std::cout << "PDO 4 -New data encountered - " << counter << std::endl;
        }
        data_capture++;
        // std::cout << " data capture: "<< unsigned(data_capture) << std::endl;
        for (int i = 0; i < 7; i++) {
            // std::cout << "f7   :" << f7.data[i+1] << std::endl;
            result[i + 42] = f7.data[i + 1];
        }

        if (data_capture == 7) {
            //std::cout << "Writing data" << std::endl;
            get_tag(result, tag_x, tag_y);
            get_x_y_ang_time(result, x, y, ang, sensor_time, decode_time);
            data_capture = 0;
            *flag = 0;
        }
    }

    return 0;

}

void GlsBarcode::get_tag(uint8_t *result, uint64_t *tag_x, uint64_t *tag_y) {
    static char tag_str_x[5] = {0};
    static char tag_str_y[5] = {0};
    memcpy(&tag_str_x[0], &result[4], 4);
    memcpy(&tag_str_y[0], &result[9], 4);
    tag_str_x[4] = '\0';
    tag_str_y[4] = '\0';
    // *tag_x = strtoul(tag_str_x, NULL, 5);
    // *tag_y = strtoul(tag_str_y, NULL, 5);
    *tag_x = strtod(tag_str_x, NULL);
    *tag_y = strtod(tag_str_y, NULL);
    // char tag_str[9] = {0};
    // memcpy(&tag_str[0], &result[4], 4);
    // memcpy(&tag_str[4], &result[9], 4);
    // tag_str[9] = '\0';
    // uint64_t tag_val = st	data_capture = 0;rtoul(tag_str, NULL, 10);
    // *tag = tag_val;
}

void GlsBarcode::get_x_y_ang_time(uint8_t *result, double *x, double *y, double *ang, uint64_t *sensor_time,
                                  uint64_t *decode_time) {
    static char x_str[5] = {0};
    int xi = 0;
    static char y_str[5] = {0};
    int yi = 0;
    static char ang_str[15] = {0};
    int ang_i = 0;
    static char time_str[20] = {0};
    int time_i = 0;
    static char decode_time_str[20] = {0};
    int decode_time_i = 0;


    int term_count = 0;

    for (int i = 14; result[i] != '\0'; i++) {

        if (result[i] == ';')
            term_count++;
        else {
            if (term_count == 0) {
                //get x
                x_str[xi] = result[i];
                xi++;
            } else if (term_count == 1) {
                //get y
                x_str[xi] = '\0';
                y_str[yi] = result[i];
                yi++;
            } else if (term_count == 2) {
                //get angle
                y_str[yi] = '\0';
                ang_str[ang_i] = result[i];
                ang_i++;
            } else if (term_count == 3) {
                //get sensor time
                ang_str[ang_i] = '\0';
                time_str[time_i] = result[i];
                time_i++;

            } else if (term_count == 4) {
                //get decode time
                time_str[time_i] = '\0';
                decode_time_str[decode_time_i] = result[i];
                decode_time_i++;

            }
        }
    }
    decode_time_str[decode_time_i] = '\0';

    *x = strtod(x_str, NULL);
    *y = strtod(y_str, NULL);
    *ang = strtod(ang_str, NULL);
    *sensor_time = strtod(time_str, NULL);
    *decode_time = strtod(decode_time_str, NULL);

    // std::cout << "sensor_time" <<*sensor_time << std::endl;
    // std::cout << "decode_time" << *decode_time << std::endl;
}

int GlsBarcode::barcodeFeedback(int barcode_id, GlsBarcode::feedback_s *feedback_s_b) {

    err_pdo_ = gls_data_read(x_fb_, y_fb_, ang_fb_, sensor_time_fb_, decode_time_fb_, tag_x_fb_, tag_y_fb_, &flag_b_,
                             1);

    if (err_pdo_ == 0) {
        feedback_s_b->x_b = x_fb_[0];
        feedback_s_b->y_b = y_fb_[0];
        feedback_s_b->ang_b = ang_fb_[0];
        feedback_s_b->sensor_time_b = sensor_time_fb_[0];
        feedback_s_b->decode_time_b = decode_time_fb_[0];
        feedback_s_b->tag_x_b = tag_x_fb_[0];
        feedback_s_b->tag_y_b = tag_y_fb_[0];
        feedback_s_b->read_status = true;

        auto now = std::chrono::high_resolution_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
        long duration = value.count();
        feedback_s_b->time_sys = duration;
    }
    else{
        feedback_s_b->x_b = 0;
        feedback_s_b->y_b = 0;
        feedback_s_b->ang_b = -1;
        feedback_s_b->sensor_time_b = 0;
        feedback_s_b->decode_time_b = 0;
        feedback_s_b->tag_x_b = 0;
        feedback_s_b->tag_y_b = 0;
        feedback_s_b->read_status = false;
    }

    return err_pdo_;
}