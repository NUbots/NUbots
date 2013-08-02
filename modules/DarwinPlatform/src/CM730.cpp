#include "CM730.h"

Darwin::CM730::CM730(UART& coms, int id) : DarwinDevice(coms, id) {
    
}

void Darwin::CM730::turnOnDynamixel() {
    write(Address::DXL_POWER, true);
}