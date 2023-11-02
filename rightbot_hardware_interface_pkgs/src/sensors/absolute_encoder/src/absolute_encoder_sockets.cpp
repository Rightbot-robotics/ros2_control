#include <absolute_encoder/absolute_encoder_sockets.hpp>


AbsoluteEncoderSockets::AbsoluteEncoderSockets(int sensor_id) {

    abs_pdo_fd = -1;    //!< Process CAN-connection.
    abs_cfg_fd = -1;    //!< Configuration CAN-connection.
    abs_sync_fd = -1;  //!< Sync CAN-connection.

    createSockets(sensor_id);
}

AbsoluteEncoderSockets::~AbsoluteEncoderSockets() {

}

bool AbsoluteEncoderSockets::createSockets(int sensor_id) {

    std::string can_interface_id = "can0";

    uint32_t pdo_masks_one[1] = {COB_MASK};
    uint32_t pdo_filters_one[1] = {
            PDO_TX1_ID + sensor_id
    };

    abs_pdo_fd = socketcan_open(can_interface_id, pdo_filters_one, pdo_masks_one, 1);

    uint32_t cfg_masks[3] = {COB_MASK, COB_MASK, COB_MASK};
    uint32_t cfg_filters[3] = {
            0x00,
            NMT_TX + sensor_id,
            SDO_TX + sensor_id,
    };

    abs_cfg_fd = socketcan_open(can_interface_id, cfg_filters, cfg_masks, 3);

    uint32_t sync_masks[1] = {COB_MASK};
    uint32_t sync_filters[1] = {
            SYNC_TX};

    abs_sync_fd = socketcan_open(can_interface_id, sync_filters, sync_masks, 1);

    if (abs_cfg_fd == -1 || abs_sync_fd == -1
        || abs_pdo_fd == -1) {
        return SOCKETS_ERROR;
    }
    return 0;
}