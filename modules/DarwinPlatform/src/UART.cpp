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
    
    for(int i = 0; i < result.data.size(); ++i) {
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
	
    if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0) {
        // TODO error "Cannot set serial info"
	}
    
	tcflush(m_Socket_fd, TCIFLUSH);
    
    m_ByteTransferTime = (1000.0 / baudrate) * 12.0;
#endif

}

Darwin::CommandResult Darwin::UART::readPacket() {
    
    // Our result
    CommandResult result;
    
    // Record the time we start working on this so we can timeout
    auto start = std::chrono::steady_clock::now();
    
    // Keep attempting to read until we either timeout or see two consecutive 0xFF bytes
    for(int sync = 0, byte = 0;
        sync < 2;
        read(m_fd, &byte, 1), sync = byte == 0xFF ? sync + 1 : 0) {
        
        // Timeout if we have taken too long
        if (std::chrono::steady_clock::now() - start > TIMEOUT)
            return CommandResult();
    }
    
    // Read our remaining 4 byte packet header
    uint8_t headerBytes[4];
    for(int done = 0; done < 4; done += read(m_fd, &headerBytes[done], 4 - done)) {
        
        // Timeout if we have taken too long
        if (std::chrono::steady_clock::now() - start > TIMEOUT)
            return CommandResult();
    }
    
    // Make our Header object
    result.header = *reinterpret_cast<Header*>(headerBytes);
    
    // Here we adjust our "length" to mean the length of the payload rather then the length of bytes after the length
    result.header.length = result.header.length - 2;
    result.data.resize(result.header.length);
    
    // Read our payload
    for(int done = 0;
        done < result.header.length;
        done += read(m_fd, &result.data.data()[done], (result.header.length) - done)) {
        
        // Timeout if we have taken too long
        if (std::chrono::steady_clock::now() - start > TIMEOUT)
            return CommandResult();
    }
    
    // Read our checksum
    for(int done = 0;
        done < 1;
        done += read(m_fd, &result.checksum, 1)) {
        
        // Timeout if we have taken too long
        if (std::chrono::steady_clock::now() - start > TIMEOUT)
            return CommandResult();
    }
    
    // Validate our checksum
    if(result.checksum == calculateChecksum(result)) {
        CommandResult result;
        result.header.errorcode = ErrorCode::CORRUPT_DATA;
        return result;
    }
    
    // Return the packet we recieved
    return result;
}

std::vector<Darwin::CommandResult> Darwin::UART::executeBulk(const std::vector<uint8_t>& command) {
    
    int responses = (command[Packet::LENGTH]-3) / 3;
    std::vector<CommandResult> results(responses);
    
    // Lock our mutex
    std::unique_lock<std::mutex>(m_mutex);
    
    // We flush our buffer, just in case there was anything random in it
    tcflush(m_fd,TCIOFLUSH);
    
    // Write the command as usual
    write(m_fd, command.data(), command.size());
    
    // Read our responses for each of the packets
    for (int i = 0; i < responses; ++i) {
        results[i] = readPacket();
    }
    
    return results;
}