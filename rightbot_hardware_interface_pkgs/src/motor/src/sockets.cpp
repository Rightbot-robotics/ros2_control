#include <motor/sockets.hpp>

Sockets::Sockets(int motor_id, std::string motor_name) {

    motor_pdo_fd = -1;    //!< Process CAN-connection.
    motor_cfg_fd = -1;    //!< Configuration CAN-connection.
    motor_sync_fd = -1;  //!< Sync CAN-connection.

    nmt_motor_cfg_fd = -1;

    motor_status_pdo_fd = -1;
    motor_vel_pdo_fd = -1;
    motor_enc_pdo_fd = -1;
    motor_system_status_pdo_fd = -1;
    motor_vel_read_pdo_fd = -1;

    motor_can_id = motor_id;
    motor_name_ = motor_name;
    createSockets(motor_id);
}

Sockets::~Sockets() {

}

bool Sockets::createSockets(int motor_id) {

    std::string can_interface_id = "can2";
    if(motor_name_ == "camera_rotation_joint") {
        can_interface_id = "can2";
    }

    uint32_t motor_status_pdo_masks[1] = {COB_MASK};
    uint32_t motor_status_pdo_filters[1] = {
            PDO_TX1_ID + motor_id
    };
    motor_status_pdo_fd = socketcan_open(can_interface_id, motor_status_pdo_filters, motor_status_pdo_masks, 1);

    uint32_t motor_vel_pdo_masks[1] = {COB_MASK};
    uint32_t motor_vel_pdo_filters[1] = {
            PDO_TX2_ID + motor_id
    };
    motor_vel_pdo_fd = socketcan_open(can_interface_id, motor_vel_pdo_filters, motor_vel_pdo_masks, 1);

    uint32_t motor_enc_pdo_masks[1] = {COB_MASK};
    uint32_t motor_enc_pdo_filters[1] = {
            PDO_TX3_ID + motor_id
    };
    motor_enc_pdo_fd = socketcan_open(can_interface_id, motor_enc_pdo_filters, motor_enc_pdo_masks, 1);

    uint32_t motor_system_status_pdo_masks[1] = {COB_MASK};
    uint32_t motor_system_status_pdo_filters[1] = {
            PDO_TX4_ID + motor_id
    };
    motor_system_status_pdo_fd = socketcan_open(can_interface_id, motor_system_status_pdo_filters, motor_system_status_pdo_masks, 1);

    uint32_t cfg_masks[2] = {COB_MASK, COB_MASK};
    uint32_t cfg_filters[2] = {
            0x00,
            SDO_TX + motor_id
    };
    motor_cfg_fd = socketcan_open(can_interface_id, cfg_filters, cfg_masks, 2);

    uint32_t motor_vel_read_pdo_masks[1] = {COB_MASK};
    uint32_t motor_vel_read_pdo_filters[1] = {
            PDO_RX4_ID + motor_id
    };
    motor_vel_read_pdo_fd = socketcan_open(can_interface_id, motor_vel_read_pdo_filters, motor_vel_read_pdo_masks, 1);

    uint32_t nmt_cfg_masks[1] = {COB_MASK};
    uint32_t nmt_cfg_filters[1] = {
            NMT_TX + motor_id};
    nmt_motor_cfg_fd = socketcan_open(can_interface_id, nmt_cfg_filters, nmt_cfg_masks, 1);

    uint32_t sync_masks[1] = {COB_MASK};
    uint32_t sync_filters[1] = {
            SYNC_TX};
    motor_sync_fd = socketcan_open(can_interface_id, sync_filters, sync_masks, 1);

    if (motor_status_pdo_fd == -1 || motor_vel_pdo_fd == -1 || motor_enc_pdo_fd == -1 ||
        motor_system_status_pdo_fd == -1 || motor_vel_read_pdo_fd == -1
        || motor_cfg_fd == -1 || nmt_motor_cfg_fd == -1 || motor_sync_fd == -1) {
        return SOCKETS_ERROR;
    }
    return 0;
}
