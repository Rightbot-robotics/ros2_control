#include <canopen_pkg/SDO.h>
#include <socketcan_pkg/socketcan.h>
#include <socketcan_pkg/printd.h>


#define SDO_RESPONSE_ERROR     0x80
#define SDO_RESPONSE_WRITE_OK  0x60
#define SDO_RESPONSE_READ_OK   0x40


/** Automatic calculation of ccd-field for requests **/
uint8_t SDO_calculate_ccd(char rw, int size) {
    uint8_t base = 0x40;
    if (rw == 'w') {
        base = 0x20;
    }

    switch (size) {
        case 0:
            return base;
        case 1:
            return base + 0x0F;
        case 2:
            return base + 0x0B;
        case 3:
            return base + 0x07;
        case 4:
            return base + 0x03;
    }
    return 0;
}

uint32_t SDO_calculate_size(uint8_t cdd) {
    uint32_t size = 0;
    switch (cdd) {
        case 0x80:
            size = 0;
            break;
        case 0x4F:
            size = 1;
            break;
        case 0x4B:
            size = 2;
            break;
        case 0x47:
            size = 3;
            break;
        case 0x43:
            size = 4;
            break;
    }
    return size;
}


int SDO_write(int fd, const SDO_data *d) {
    int err;
    int fillerbytes;
    const int timeout = 30;
    const int buffer = 15;
    uint16_t cob, cob_r;
    uint8_t ccd, msb, lsb;
    my_can_frame f;

    // Define data
    cob = SDO_RX + d->nodeid;
    ccd = SDO_calculate_ccd('w', d->data.size);
    fillerbytes = 8 - 4 - d->data.size;
    Socketcan_t data[5] = {
            {1, ccd},
            {2, d->index},
            {1, d->subindex},
            d->data,
            {fillerbytes, 0x00}
    };



    // Send write request
    err = socketcan_write(fd, cob, 5, data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }

    cob_r = SDO_TX + d->nodeid;
    msb = (d->index >> 8 & 0xff);
    lsb = (d->index & 0xff);

    // Wait for result
    for (int i = 0; i < buffer; i++) {
        err = socketcan_read(fd, &f, timeout / buffer);
//		printf("err=%d node=0x%x index=0x%x sub=0x%x from=0x%x res=0x%x\n", err, d->nodeid, d->index, d->subindex, f.id, f.data[0]);

        if (err == 0 && f.dlc >= 4 && f.id == cob_r && f.data[1] == lsb && f.data[2] == msb &&
            f.data[3] == d->subindex) {
            // Response recived
            if (f.data[0] == SDO_RESPONSE_WRITE_OK) {
                //printf("ok\n\n");
                return 0;
            } else {
                printd(LOG_ERROR, "socketcan SDO: response error node=%d index=0x%x subindex=0x%x\n", d->nodeid,
                       d->index, d->subindex);
                return SOCKETCAN_ERROR;
            }
        }
    }

    printd(LOG_WARN, "socketcan SDO: timeout node=%d index=0x%x RE-SENDING COMMAND \n", d->nodeid, d->index);

    // Send write request
    err = socketcan_write(fd, cob, 5, data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }

    // Wait for result
    for (int i = 0; i < buffer; i++) {
        err = socketcan_read(fd, &f, timeout / buffer);
//		printf("err=%d node=0x%x index=0x%x sub=0x%x from=0x%x res=0x%x\n", err, d->nodeid, d->index, d->subindex, f.id, f.data[0]);

        if (err == 0 && f.dlc >= 4 && f.id == cob_r && f.data[1] == lsb && f.data[2] == msb &&
            f.data[3] == d->subindex) {
            // Response recived
            if (f.data[0] == SDO_RESPONSE_WRITE_OK) {
                //printf("ok\n\n");
                return 0;
            } else {
                printd(LOG_ERROR, "socketcan SDO: response error node=%d index=0x%x subindex=0x%x\n", d->nodeid,
                       d->index, d->subindex);
                return SOCKETCAN_ERROR;
            }
        }
    }

    printd(LOG_WARN, "socketcan SDO: timeout node=%d index=0x%x EXIT \n", d->nodeid, d->index);

    
    return SOCKETCAN_TIMEOUT;
}


int SDO_write_multi_byte(int fd, const SDO_data *d, uint8_t *value) {
    int err;
    int fillerbytes;
    const int timeout = 30;
    const int buffer = 15;
    uint16_t cob, cob_r;
    uint8_t ccd, msb, lsb;
    my_can_frame f;

    // Define data
    cob = SDO_RX + d->nodeid;
    ccd = 0x21;
    fillerbytes = 8 - 4 - d->data.size;
    Socketcan_t data[5] = {
            {1, ccd},
            {2, d->index},
            {1, d->subindex},
            d->data,
            {fillerbytes, 0x00}
    };
    
    // Send write request
    err = socketcan_write(fd, cob, 5,  data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }

    cob_r = SDO_TX + d->nodeid;
    msb = (d->index >> 8 & 0xff);
    lsb = (d->index & 0xff);

    // Wait for result
    for (int i = 0; i < buffer; i++) {
        err = socketcan_read(fd, &f, timeout / buffer);
//		printf("err=%d node=0x%x index=0x%x sub=0x%x from=0x%x res=0x%x\n", err, d->nodeid, d->index, d->subindex, f.id, f.data[0]);

        if (err == 0 && f.dlc >= 4 && f.id == cob_r && f.data[1] == lsb && f.data[2] == msb &&
            f.data[3] == d->subindex) {
            // Response recived
            if (f.data[0] == SDO_RESPONSE_WRITE_OK) {
                Socketcan_t data[8] = {
                    {1, 0x00},
                    {1, value[0]},
                    {1, value[1]},
                    {1, value[2]},
                    {1, value[3]},
                    {1, value[4]},
                    {1, value[5]},
                    {1, value[6]},
                    };
                err = socketcan_write(fd, cob, 8, data);
                if (d->data.size == 8)
                {
                    Socketcan_t data2[3] = {
                    {1, 0x11},
                    {1, value[7]},
                    {6, 0x00},
                    };
                    err = socketcan_write(fd, cob, 3, data2);
                    return err;
                }
            } else {
                printd(LOG_ERROR, "socketcan SDO: response error node=%d index=0x%x subindex=0x%x\n", d->nodeid,
                       d->index, d->subindex);
                return SOCKETCAN_ERROR;
            }
        }
    }

    printd(LOG_WARN, "socketcan SDO: timeout node=%d index=0x%x EXIT \n", d->nodeid, d->index);
    
    return SOCKETCAN_TIMEOUT;
}

int SDO_sub_write(int fd, const SDO_data *d) {
    int err;
    int fillerbytes;
    const int timeout = 30;
    const int buffer = 15;
    uint16_t cob, cob_r;
    uint8_t ccd, msb, lsb;
    my_can_frame f;

    // Define data
    cob = SDO_RX + d->nodeid;
    ccd = 0x21;
    fillerbytes = 8 - 4 - d->data.size;
    Socketcan_t data[5] = {
            {1, ccd},
            {2, d->index},
            {1, d->subindex},
            d->data,
            {fillerbytes, 0x00}
    };
    // Send write request
    err = socketcan_write(fd, cob, 5, data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }
}

int SDO_write_no_wait(int fd, const SDO_data *d) {
    int err;
    int fillerbytes;
    const int timeout = 30;
    const int buffer = 15;
    uint16_t cob, cob_r;
    uint8_t ccd, msb, lsb;
    my_can_frame f;

    // Define data
    cob = SDO_RX + d->nodeid;
    ccd = SDO_calculate_ccd('w', d->data.size);
    fillerbytes = 8 - 4 - d->data.size;
    Socketcan_t data[5] = {
            {1, ccd},
            {2, d->index},
            {1, d->subindex},
            d->data,
            {fillerbytes, 0x00}
    };



    // Send write request
    err = socketcan_write(fd, cob, 5, data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }
    
    return err;
}

int SDO_read(int fd, SDO_data *d, SDO_data *resp) {
    int err;
    int fillerbytes;
    const int timeout = 30;
    const int buffer = 15;
    uint16_t cob, cob_r;
    uint8_t ccd, msb, lsb;
    my_can_frame f;

    // Define data
    cob = SDO_RX + d->nodeid;
    ccd = SDO_calculate_ccd('r', d->data.size);
    fillerbytes = 8 - 4 - d->data.size;
    Socketcan_t data[5] = {
            {1, ccd},
            {2, d->index},
            {1, d->subindex},
            d->data,
            {fillerbytes, 0x00}
    };



    // Send write request
    err = socketcan_write(fd, cob, 5, data);
    if (err != 0) {
        printd(LOG_ERROR, "socketcan SDO: Could not write to the CAN-bus fd=%d.\n", fd);
        return err;
    }

    cob_r = SDO_TX + d->nodeid;
    msb = (d->index >> 8 & 0xff);
    lsb = (d->index & 0xff);

    // Wait for result
    for (int i = 0; i < buffer; i++) {
        err = socketcan_read(fd, &f, timeout / buffer);
//		printf("err=%d node=0x%x index=0x%x sub=0x%x from=0x%x res=0x%x\n", err, d->nodeid, d->index, d->subindex, f.id, f.data[0]);

        if (err == 0 && f.dlc >= 4 && f.id == cob_r && f.data[1] == lsb && f.data[2] == msb &&
            f.data[3] == d->subindex) {
            // Response recived
            if ((f.data[0] & 0xF0) == SDO_RESPONSE_READ_OK) {
                resp->nodeid = d->nodeid;
                resp->index = d->index;
                resp->subindex = d->subindex;
                resp->data.size = SDO_calculate_size(f.data[0]);
                resp->data.data = 0;
                for(uint32_t data_index = 0; data_index < resp->data.size; data_index++) {
                    uint32_t new_data = (resp->data.data << 8) | f.data[3 + resp->data.size - data_index];
                    resp->data.data = new_data;
                }
                return 0;
            } else {
                printd(LOG_ERROR, "socketcan SDO: response error node=%d index=0x%x subindex=0x%x\n", d->nodeid,
                       d->index, d->subindex);
                return SOCKETCAN_ERROR;
            }
        }
    }

    printd(LOG_WARN, "socketcan SDO: timeout node=%d index=0x%x\n", d->nodeid, d->index);
    return SOCKETCAN_TIMEOUT;
}


int SDO_acknowledge(int fd, const my_can_frame *f) {
    Socketcan_t ack[4];
    ack[0].size = 1;
    ack[0].data = SDO_RESPONSE_WRITE_OK;

    ack[1].size = 1;
    ack[1].data = f->data[1]; // index lsb

    ack[2].size = 1;
    ack[2].data = f->data[2]; // index msb

    ack[3].size = 1;
    ack[3].data = f->data[3]; // subindex

    int nodeid = f->id - SDO_RX;
    return socketcan_write(fd, SDO_TX + nodeid, 4, ack);
}
