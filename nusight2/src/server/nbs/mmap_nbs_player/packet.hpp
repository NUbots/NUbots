#ifndef PACKET_H
#define PACKET_H

#include <cstdint>

struct Packet {
    uint64_t timestamp;
    uint8_t* hash;
    uint8_t* payload;
    uint32_t size;
};

#endif  // PACKET_H
