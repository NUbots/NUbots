#include "UART.h"

uint8_t Darwin::calculateChecksum(void* command) {
    
    uint8_t* data = static_cast<uint8_t*>(command);
    uint8_t checksum = 0x00;
    // Skip over the magic numbers and checksum the rest of the packet
    for(int i=2; i < data[3] + 3; ++i) {
        checksum += data[i];
    }
    return (~checksum);
}

uint8_t Darwin::calculateChecksum(const CommandResult& result) {
    
    uint8_t checksum = 0x00;
    
    checksum += result.header.id;
    checksum += result.header.length;
    checksum += result.header.errorcode;
    
    for(int i = 0; i < result.data.size() - 1; ++i) {
        checksum += result.data[i];
    }
    
    return (~checksum);
}


Darwin::UART::UART(const char* name) {
    
    termios newtio;
	double baudrate = 1000000.0; //bps (1Mbps)
    
    m_fd = open(name, O_RDWR|O_NOCTTY|O_NONBLOCK);
    
    if(m_fd < 0) {
        // TODO "The UART failed to connect to the CM730"
    }
    
    // You must set 38400bps!
	memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    tcsetattr(m_fd, TCSANOW, &newtio);
    
#ifdef __linux__
    
    serial_struct serinfo;
    
    // Set non-standard baudrate
    if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0) {
        // TODO error "Cannot get serial info"
    }
    
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
    if(ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        // TODO error "Cannot set serial info"
	}
    
	tcflush(m_Socket_fd, TCIFLUSH);
    
    m_ByteTransferTime = (1000.0 / baudrate) * 12.0;
#endif

}