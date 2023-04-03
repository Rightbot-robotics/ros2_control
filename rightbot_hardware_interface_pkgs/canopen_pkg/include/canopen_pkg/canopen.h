#ifndef CANOPEN_H
#define CANOPEN_H

#include <canopen_pkg/NMT.h>
#include <canopen_pkg/PDO.h>
#include <canopen_pkg/SDO.h>

#define CANOPEN_ERROR   -1
#define CANOPEN_TIMEOUT -2

#define CANOPEN_BROADCAST_ID 0x00

// Canopen Identifier masks
#define COB_MASK      0x7ff
#define COB_ID_MASK   0x07f
#define COB_FUNC_MASK 0x780

#endif
