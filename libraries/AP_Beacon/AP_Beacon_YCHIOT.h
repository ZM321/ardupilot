#pragma once

#include "AP_Beacon_Backend.h"
#include "trilateration.h"

#define AP_Beacon_YCHIOT_MSG_LEN_MAX         65
#define AP_Beacon_YCHIOT_DEBUG               0
#define AP_Beacon_YCHIOT_A0_HEALTHY          0x00000001
#define AP_Beacon_YCHIOT_A1_HEALTHY          0x00000002
#define AP_Beacon_YCHIOT_A2_HEALTHY          0x00000004
#define AP_Beacon_YCHIOT_A3_HEALTHY          0x00000008
#define AP_Beacon_YCHIOT_MAX_JUMP_M          5.0f

class AP_Beacon_YCHIOT : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_YCHIOT(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy();

    // update
    void update();

private:
    void fill_buffer(char c, char* buf);
    void parse_buffer();

    AP_HAL::UARTDriver *uart = nullptr;
    bool     get_head;
    char     linebuf[AP_Beacon_YCHIOT_MSG_LEN_MAX];
    uint8_t  linebuf_len = 0;
    uint32_t last_update_ms = 0;
    char mid[3], ata[5];
    int  mask, range[4], nranges, rseq, debug;
};
