#include <gls_barcode/sockets.hpp>

BarcodeSockets::BarcodeSockets(int barcode_id, int barcode_base_id) {

    gls_one_pdo_fd = -1;    //!< Process CAN-connection.
    gls_two_pdo_fd = -1;
    gls_three_pdo_fd = -1;
    gls_four_pdo_fd = -1;
    gls_five_pdo_fd = -1;
    gls_six_pdo_fd = -1;
    gls_seven_pdo_fd = -1;

    gls_cfg_fd = -1;    //!< Configuration CAN-connection.
    gls_sync_fd = -1;  //!< Sync CAN-connection.

    createBarcodeSockets(barcode_id, barcode_base_id);
}

BarcodeSockets::~BarcodeSockets() {

}

bool BarcodeSockets::createBarcodeSockets(int barcode_id, int barcode_base_id) {

    uint32_t pdo_masks_one[1] = {COB_MASK};
    uint32_t pdo_filters_one[1] = {
            barcode_base_id + 0x00,
    };
    gls_one_pdo_fd = socketcan_open("can1", pdo_filters_one, pdo_masks_one, 1);

    uint32_t pdo_masks_two[1] = {COB_MASK};
    uint32_t pdo_filters_two[1] = {
            barcode_base_id + 0x01,
    };
    gls_two_pdo_fd = socketcan_open("can1", pdo_filters_two, pdo_masks_two, 1);

    uint32_t pdo_masks_three[1] = {COB_MASK};
    uint32_t pdo_filters_three[1] = {
            barcode_base_id + 0x02,
    };
    gls_three_pdo_fd = socketcan_open("can1", pdo_filters_three, pdo_masks_three, 1);

    uint32_t pdo_masks_four[1] = {COB_MASK};
    uint32_t pdo_filters_four[1] = {
            barcode_base_id + 0x03,
    };
    gls_four_pdo_fd = socketcan_open("can1", pdo_filters_four, pdo_masks_four, 1);

    uint32_t pdo_masks_five[1] = {COB_MASK};
    uint32_t pdo_filters_five[1] = {
            barcode_base_id + 0x04,
    };
    gls_five_pdo_fd = socketcan_open("can1", pdo_filters_five, pdo_masks_five, 1);

    uint32_t pdo_masks_six[1] = {COB_MASK};
    uint32_t pdo_filters_six[1] = {
            barcode_base_id + 0x05,
    };
    gls_six_pdo_fd = socketcan_open("can1", pdo_filters_six, pdo_masks_six, 1);

    uint32_t pdo_masks_seven[1] = {COB_MASK};
    uint32_t pdo_filters_seven[1] = {
            barcode_base_id + 0x06,
    };
    gls_seven_pdo_fd = socketcan_open("can1", pdo_filters_seven, pdo_masks_seven, 1);

    uint32_t cfg_masks[3] = {COB_MASK, COB_MASK, COB_MASK};
    uint32_t cfg_filters[3] = {
            0x00,
            NMT_TX + barcode_id,
            SDO_TX + barcode_id,
    };
    gls_cfg_fd = socketcan_open("can1", cfg_filters, cfg_masks, 3);

    uint32_t sync_masks[1] = {COB_MASK};
    uint32_t sync_filters[1] = {
            SYNC_TX};
    gls_sync_fd = socketcan_open("can1", sync_filters, sync_masks, 1);

    if (gls_cfg_fd == -1 || gls_sync_fd == -1
        || gls_one_pdo_fd == -1 || gls_two_pdo_fd == -1
        || gls_three_pdo_fd == -1 || gls_four_pdo_fd == -1
        || gls_five_pdo_fd == -1 || gls_six_pdo_fd == -1
        || gls_seven_pdo_fd == -1) {
        return SOCKETS_ERROR;
    }
    return 0;
}