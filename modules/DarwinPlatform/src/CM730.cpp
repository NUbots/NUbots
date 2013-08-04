#include "CM730.h"

// Pass our arguments to our parent's constructor
Darwin::CM730::CM730(UART& coms, int id) : DarwinDevice(coms, id) {}

void Darwin::CM730::turnOnDynamixel() {
    
    // Write true to the DXL_POWER byte
    write(Address::DXL_POWER, true);
}