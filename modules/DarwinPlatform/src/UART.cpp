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
    
    for(size_t i = 0; i < result.data.size(); ++i) {
        checksum += result.data[i];
    }
    
    return (~checksum);
}

#ifndef __linux__

// This is only defined here so it will compile in xcode during testing
struct serial_struct {
    int     type;
    int     line;
    unsigned int    port;
    int     irq;
    int     flags;
    int     xmit_fifo_size;
    int     custom_divisor;
    int     baud_base;
    unsigned short  close_delay;
    char    io_type;
    char    reserved_char[1];
    int     hub6;
    unsigned short  closing_wait; /* time to wait before closing */
    unsigned short  closing_wait2; /* no longer used... */
    unsigned char   *iomem_base;
    unsigned short  iomem_reg_shift;
    unsigned int    port_high;
    unsigned long   iomap_base;     /* cookie passed into ioremap */
};

#define TIOCGSERIAL 0
#define TIOCSSERIAL 0
#define ASYNC_SPD_MASK 0
#define ASYNC_SPD_CUST 0

#endif

#include <iostream>

Darwin::UART::UART(const char* name) {
    
    double baud = 1000000; // (1mb/s)
    
    m_fd = open(name, O_RDWR|O_NOCTTY|O_NONBLOCK);
    
    // If we have a valid file handle, and were able to configure it correctly (custom baud)
    if(m_fd < 0 || !configure(baud)) {
        // TODO there was an error connecting to the serial port
    }
}

int Darwin::UART::configure(double baud) {
    
    // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate aliasing"
    //http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
    termios tio;
	memset(&tio, 0, sizeof(tio));
    // B38400 for alising, CS8 (8bit,no parity,1 stopbit), CLOCAL (local connection, no modem contol), CREAD (enable receiving characters)
    tio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    // IGNPAR (ignore incoming parity bits as we don't have parity)
    tio.c_iflag      = IGNPAR;
    // 0 means raw output
    tio.c_oflag      = 0;
    // No ICANON so we read immediantly rather then line by line
    tio.c_lflag     &= ~ICANON;
    tio.c_cc[VTIME]  = 0;
    tio.c_cc[VMIN]   = 0;
    // Set the settings
    tcsetattr(m_fd, TCSANOW, &tio);
    
    // Here we do the baud rate aliasing in order to set the custom baud rate
    serial_struct serinfo;
    
    // Get our serial_info from the system
    if(ioctl(m_fd, TIOCGSERIAL, &serinfo) < 0) {
        return false;
    }
    
    // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    
    // Set our custom divsor for our speed
    serinfo.custom_divisor = serinfo.baud_base / baud;
	
    // Set our custom speed in the system
    if(ioctl(m_fd, TIOCSSERIAL, &serinfo) < 0) {
        return false;
	}
    
    // Flush our connection to remove all existing data
	tcflush(m_fd, TCIFLUSH);
    
    return true;
}

Darwin::CommandResult Darwin::UART::readPacket() {
    
    // We will wait this long for an initial packet header
    int PACKET_WAIT = 100;
    // We will only wait a maximum of 12 microseconds between bytes in a packet (assumes baud of 1000000bps)
    int BYTE_WAIT = 12;
    
    // Our result
    CommandResult result;
    
    // Clear our connection set and put in our serial device
    fd_set connectionset;
    timeval timeout;
    timeout.tv_sec = 0;
    FD_ZERO(&connectionset);
    FD_SET(m_fd, &connectionset);
    
    // First we find the packet magic number in order to sync with the channel
    timeout.tv_usec = PACKET_WAIT;
    for(int sync = 0; sync < 2;) {
        if(select(m_fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {
            
            uint8_t byte;
            read(m_fd, &byte, 1);
            sync = byte == 0xFF ? sync + 1 : 0;
        }
        else {
            // The result is pre initialized as a timeout
            return result;
        }
    }
    
    // We now are now waiting for 4 bytes
    timeout.tv_usec = BYTE_WAIT * sizeof(Header);
    uint8_t headerBytes[sizeof(Header)];
    for(size_t done = 0; done < sizeof(Header);) {
        if(select(m_fd + 1, & connectionset, nullptr, nullptr, &timeout) == 1) {
            
            done += read(m_fd, &headerBytes[done], sizeof(Header) - done);
        }
        else {
            // The result is pre initialized as a timeout
            return result;
        }
    }
    
    // Make our Header object
    result.header = *reinterpret_cast<Header*>(headerBytes);
    
    // Here we adjust our "length" to mean the length of the payload rather then the length of bytes after the length
    result.header.length = result.header.length - 2;
    
    // We now are now waiting for our data
    timeout.tv_usec = BYTE_WAIT * result.header.length;
    result.data.resize(result.header.length);
    for(int done = 0; done < result.header.length;) {
        if(select(m_fd + 1, & connectionset, nullptr, nullptr, &timeout) == 1) {
            
            done += read(m_fd, &result.data[done], result.header.length - done);
        }
        else {
            // Set our packet header to timeout and return it
            result.header.errorcode = ErrorCode::NO_RESPONSE;
            return result;
        }
    }
    
    // We just read the checksum now
    timeout.tv_usec = BYTE_WAIT * 1;
    if(select(m_fd + 1, & connectionset, nullptr, nullptr, &timeout) == 1) {
        
        read(m_fd, &result.checksum, 1);
    }
    else {
        // If all we are missing is the checksum, just assume the data is corrupt
        result.header.errorcode = ErrorCode::CORRUPT_DATA;
        return result;
    }
    
    // Validate our checksum
    if(result.checksum != calculateChecksum(result)) {
        CommandResult result;
        result.header.errorcode = ErrorCode::CORRUPT_DATA;
        return result;
    }
    
    // Return the packet we recieved
    return result;
}

std::vector<Darwin::CommandResult> Darwin::UART::executeBulk(const std::vector<uint8_t>& command) {
    
    // We can work out how many responses to expect based on our packets length
    int responses = (command[Packet::LENGTH]-3) / 3;
    std::vector<CommandResult> results(responses);
    
    // Lock our mutex
    std::unique_lock<std::mutex>(m_mutex);
    
    // We flush our buffer, just in case there was anything random in it
    tcflush(m_fd,TCIFLUSH);
    
    // Write the command as usual
    write(m_fd, command.data(), command.size());
    
    // Read our responses for each of the packets
    for (int i = 0; i < responses; ++i) {
        results[i] = readPacket();
    }
    
    return results;
}

void Darwin::UART::executeBroadcast(const std::vector<uint8_t>& command) {
    
    // Lock our mutex
    std::unique_lock<std::mutex>(m_mutex);
    
    // We flush our buffer, just in case there was anything random in it
    tcflush(m_fd,TCIFLUSH);
    
    // Write the command as usual
    write(m_fd, command.data(), command.size());

    // There are no responses for broadcast commands
}