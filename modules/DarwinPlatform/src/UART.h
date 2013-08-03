#ifndef UART_H
#define UART_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <cstring>
#include <mutex>
#include <vector>
#ifdef __linux__
#include <linux/serial.h>
#endif

namespace Darwin
{
    // If a command takes more then 10µs to complete then it has timedout
    const std::chrono::microseconds TIMEOUT(10);
    
    namespace Packet {
        enum Packet {
            MAGIC       = 0,
            ID          = 2,
            LENGTH      = 3,
            INSTRUCTION = 4,
            ERRBIT      = 4,
            PARAMETER   = 5
        };
    }
    
    namespace ErrorCode {
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
    }
    
    // This is the header that is contained in the CommandResult
    #pragma pack(push, 1)// Make sure that this struct is not cache alligned
    struct Header {
        uint8_t id;
        uint8_t length = 0;
        uint8_t errorcode = -1;
    };
    // Check that this struct is not cache alligned
    static_assert(sizeof(Header) == 3, "The compiler is adding padding to this struct, Bad compiler!");
    #pragma pack(pop)
    
    // This is the object that is returned when a command is run
    struct CommandResult {
        Header header;
        std::vector<uint8_t> data;
        uint8_t checksum;
    };
    
    // This value calculates the checksum for a packet (the command argument is assumed to be in the CM730 format)
    uint8_t calculateChecksum(void* command);
    uint8_t calculateChecksum(const CommandResult& result);
    
    class UART {
    private:
        int m_byteTransferTime;
        int m_fd;
        std::mutex m_mutex;
        
    public:
        UART(const char* name);
        
        CommandResult readPacket();
        
        template <typename TPacket>
        CommandResult execute(const TPacket& command) {
            
            // Lock the mutex
            std::unique_lock<std::mutex> lock(m_mutex);
            
            // We flush our buffer, just in case there was anything random in it
            tcflush(m_fd,TCIOFLUSH);
            
            // Write our command to the UART
            write(m_fd, &command, sizeof(TPacket));
            
            return readPacket();
        }
        
        std::vector<CommandResult> executeBulk(const std::vector<uint8_t>& command);
    };
}

#endif
