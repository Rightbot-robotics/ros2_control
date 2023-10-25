#ifndef GLS_BARCODE_HPP_
#define GLS_BARCODE_HPP_


#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

#include<canopen_pkg/canopen.h>
#include <gls_barcode/sockets.hpp>


#define GLS_ERROR (-1)
#define GLS_TIMEOUT (-2)

class GlsBarcode {

public:

    GlsBarcode(int barcode_id, int barcode_base_id);

    ~GlsBarcode();

    typedef std::shared_ptr<GlsBarcode> GlsBarcodeSPtr;
    typedef std::unique_ptr<GlsBarcode> GlsBarcodeUPtr;

    bool gls_init(int barcode_id);

    int gls_enable(int barcode_id);


    typedef struct feedback_ {
        double x_b;
        double y_b;
        double ang_b;
        uint64_t sensor_time_b;
        uint64_t decode_time_b;
        uint64_t tag_x_b;
        uint64_t tag_y_b;
        uint64_t time_sys;
        bool read_status;

    } feedback_s;

    int barcodeFeedback(int barcode_id, GlsBarcode::feedback_s *feedback_s_b);


private:

    int motor_id;

    typedef struct {
        uint16_t index;
        uint8_t subindex;
        uint8_t length;
    } gls_pdo_mapping;


    double x_fb_[1];
    double y_fb_[1];
    double ang_fb_[1];
    uint64_t sensor_time_fb_[1];
    uint64_t decode_time_fb_[1];
    uint64_t tag_x_fb_[1];
    uint64_t tag_y_fb_[1];
    int flag_b_;

    int err_pdo_;
    

    BarcodeSockets::BarcodeSocketsSPtr barcode_sockets_;

    std::shared_ptr<spdlog::logger> logger_;


    int gls_config_node(int barcode_id);

    int gls_transmit_PDO_n_Parameter(uint16_t node_id, uint8_t n, uint32_t cob);

    int gls_request(int barcode_id);

    int gls_data_read(double *x, double *y, double *ang, uint64_t *sensor_time, uint64_t *decode_time, uint64_t *tag_x,
                      uint64_t *tag_y, int *flag, int timeout);

    void get_tag(uint8_t *result, uint64_t *tag_x, uint64_t *tag_y);

    void
    get_x_y_ang_time(uint8_t *result, double *x, double *y, double *ang, uint64_t *sensor_time, uint64_t *decode_time);


};

#endif
