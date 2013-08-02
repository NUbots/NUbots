#ifndef UART_H
#define UART_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <mutex>
#include <vector>

namespace Darwin
{
    // If a command takes more then 10µs to complete then it has timedout
    const std::chrono::microseconds TIMEOUT(10);
    
    enum ErrorCode {
        NO_RESPONSE     = -1,
        NONE            = 0x0000,
        INPUT_VOLTAGE   = 0x0001,
        ANGLE_LIMIT     = 0x0002,
        OVERHEATING     = 0x0004,
        RANGE           = 0x0008,
        CHECKSUM        = 0x0010,
        OVERLOAD        = 0x0020,
        INSTRUCTION     = 0x0040,
        CORRUPT_DATA    = 0x0080
    };
    
    // This is the header that is contained in the CommandResult
    struct Header {
        uint8_t id;
        uint8_t length = 0;
        uint8_t errorcode = -1;
    };
    
    // This is the object that is returned when a command is run
    struct CommandResult {
        Header header;
        std::vector<uint8_t> data;
    };
    
    // This value calculates the checksum for a packet (the command argument is assumed to be in the CM730 format)
    uint8_t calculateChecksum(void* command);
    uint8_t calculateChecksum(const CommandResult& result);
    
    class UART {
    private:
        int m_fd;
        std::mutex m_mutex;
        
    public:
        UART(const char* name);
        
        template <typename TPacket>
        CommandResult execute(const TPacket& command) {
            
            CommandResult result;
            
            // Lock the mutex
            std::unique_lock<std::mutex> lock(m_mutex);
            
            // Record the time we start working on this so we can timeout
            auto start = std::chrono::steady_clock::now();
            
            // Write our command to the UART
            write(m_fd, &command, sizeof(TPacket));
            
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
            result.data.resize(result.header.length);
            
            // Read our payload
            for(int done = 0;
                done < result.header.length;
                done += read(m_fd, &result.data.data()[done], result.header.length - done)) {
                
                // Timeout if we have taken too long
                if (std::chrono::steady_clock::now() - start > TIMEOUT)
                    return CommandResult();
            }
            
            // Validate our checksum
            if(result.data[result.header.length - 1] == calculateChecksum(result)) {
                CommandResult result;
                result.header.errorcode = ErrorCode::CORRUPT_DATA;
                return result;
            }
            
            // Return the packet we recieved
            return result;
        }
    };
}

#endif
