#ifndef DARWINDEVICE_H
#define DARWINDEVICE_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <mutex>

#include "UART.h"

namespace Darwin
{
    class DarwinDevice {
        
    private:
        
        enum Instruction {
            PING = 1,
            READ = 2,
            WRITE = 3,
            REG_WRITE = 4,
            ACTION = 5,
            RESET = 6,
            SYNC_WRITE = 83,
            BULK_READ = 146
        };
        
        struct BulkReadCommand {
            const uint16_t magic = 0xFFFF;
            const uint8_t id = 254; // Broadcast
            const uint8_t length;
            const uint8_t instruction = Instruction::BULK_READ;
            const uint8_t param = 0x0;
            // Now we just repeat ID, LENGTH, ADDRESS for everything we want to read
        };
        
        template <typename TType>
        struct ReadCommand {
            
            ReadCommand(uint8_t id, uint8_t address) : id(id), address(address) {};
            
            /// Magic number that heads up every packet
            const uint16_t magic = 0xFFFF;
            /// The ID of the device that we are communicating with
            const uint8_t id;
            /// The total length of the data packet (always 4)
            const uint8_t length = 4;
            /// The instruction that we will be executing (The READ instruction)
            const uint8_t instruction = Instruction::READ;
            /// The address to read from
            const uint8_t address;
            /// The number of bytes to read
            const uint8_t size = sizeof(TType);
            /// Our checksum for this command
            const uint8_t checksum = calculateChecksum(this);
        };
        
        template <typename TType>
        struct WriteCommand {
            
            WriteCommand(uint8_t id, uint8_t address, TType data) : id(id), address(address), data(data) {};
            
            /// Magic number that heads up every packet
            const uint16_t magic = 0xFFFF;
            /// The ID of the device that we are communicating with
            const uint8_t id;
            /// The total length of the data packet (4 plus however many bytes we are writing)
            const uint8_t length = 3 + sizeof(TType);
            /// The instruction that we will be executing (The WRITE instruction)
            const uint8_t instruction = Instruction::WRITE;
            /// The address we are writing to
            const uint8_t address;
            /// The bytes that we are writing
            const TType data;
            /// Our checksum for this command
            const uint8_t checksum = calculateChecksum(this);
        };
        
        struct PingCommand {
            
            PingCommand(uint8_t id) : id(id) {};
            
            /// Magic number that heads up every packet
            const uint16_t magic = 0xFFFF;
            /// The ID of the device that we are communicating with
            const uint8_t id;
            /// The total length of the data packet (always 2)
            const uint8_t length = 2;
            /// The instruction that we will be executing (The PING instruction)
            const uint8_t instruction = Instruction::PING;
            /// Our checksum for this command
            const uint8_t checksum = calculateChecksum(this);
        };
        
        UART& m_coms;
        int m_id;
        
    public:
        DarwinDevice(UART& coms, int id);
        
        template <typename TType>
        TType read(uint8_t address) {
            
            // Execute our Read command through the uart
            CommandResult result = m_coms.execute(ReadCommand<TType>(m_id, address));
            
            // Copy our resulting data into our return type
            TType data;
            memcpy(&data, result.data.data(), sizeof(TType));
            
            // Return our data
            return data;
        }
        
        template <typename TType>
        void write(uint8_t address, TType data) {
            
            // Write our data over the UART
            CommandResult result = m_coms.execute(WriteCommand<TType>(m_id, address, data));
            
            // Check our resutling error code
            if(result.header.errorcode != ErrorCode::NONE) {
                // TODO we have an error
            };
        }
        
        bool ping();
    };
}

#endif
