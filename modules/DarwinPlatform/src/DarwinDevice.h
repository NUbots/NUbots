#ifndef DARWINDEVICE_H
#define DARWINDEVICE_H

#include <stdint.h>

#include "UART.h"

namespace Darwin
{
    /**
     * @brief The darwin device is a device on the serial port that will respond to the command types.
     *
     * @details
     *  This class is extended by the CM730, MX28 and FSR's, this allows them to access various memory locations
     *  using the common functionality provided by this class
     *
     * @author Trent Houliston
     */
    class DarwinDevice {
        
    public:
        /**
         * @brief The list of valid instructions for the CM730 and related components.
         */
        enum Instruction {
            PING = 1,
            READ = 2,
            WRITE = 3,
            REG_WRITE = 4,
            ACTION = 5,
            RESET = 6,
            SYNC_WRITE = 131,
            BULK_READ = 146
        };
        
        /**
         * @brief This struct mimics the expected data structure for a Write command.
         *
         * @details
         *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
         *  you cannot add or remove members from this type (unless for some reason the API to the CM730 changes). The
         *  template argument and parameter allows you to read any type. For example if you read with a uint16_t then
         *  it will read two bytes to the device. And if you use a struct with 3 uint16_t's in it, then you can directly
         *  read a paramter with an x, y and z bytes (e.g. the accelerometer)
         * @tparam TType the type of data to be written
         *
         * @author Trent Houliston
         */
        #pragma pack(push, 1) // Make it so that the compiler reads this struct "as is" (no padding bytes)
        template <typename TType>
        struct ReadCommand {
            
            ReadCommand(uint8_t id, uint8_t address) : id(id), address(address), size(sizeof(TType)) {};
            
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
            const uint8_t size;
            /// Our checksum for this command
            const uint8_t checksum = calculateChecksum(this);
        };
        #pragma pack(pop)
        
        /**
         * @brief This struct mimics the expected data structure for a Write command.
         *
         * @details
         *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
         *  you cannot add or remove members from this type (unless for some reason the API to the CM730 changes). The
         *  template argument and parameter allows you to write any type. For example if you write with a uint16_t then
         *  it will write two bytes to the device. And if you use a struct with 3 uint16_t's in it, then you directly
         *  write to something with an x, y and z byte.
         *
         * @tparam TType the type of data to be written
         *
         * @author Trent Houliston
         */
        #pragma pack(push, 1) // Make it so that the compiler reads this struct "as is" (no padding bytes)
        template <typename TType>
        struct WriteCommand {
            
            WriteCommand(uint8_t id, uint8_t address, TType data) : id(id), length(3 + sizeof(TType)), address(address), data(data) {};
            
            /// Magic number that heads up every packet
            const uint16_t magic = 0xFFFF;
            /// The ID of the device that we are communicating with
            const uint8_t id;
            /// The total length of the data packet (3 plus however many bytes we are writing)
            const uint8_t length;
            /// The instruction that we will be executing (The WRITE instruction)
            const uint8_t instruction = Instruction::WRITE;
            /// The address we are writing to
            const uint8_t address;
            /// The bytes that we are writing
            const TType data;
            /// Our checksum for this command
            const uint8_t checksum = calculateChecksum(this);
        };
        #pragma pack(pop)
        
        /**
         * @brief This struct mimics the expected data structure for a Ping command.
         *
         * @details
         *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
         *  you cannot add or remove members from this type (unless for some reason the API to the CM730 changes).
         *
         * @author Trent Houliston
         */
        #pragma pack(push, 1) // Make it so that the compiler reads this struct "as is" (no padding bytes)
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
        // Check that this struct is not cache alligned
        static_assert(sizeof(PingCommand) == 6, "The compiler is adding padding to this struct, Bad compiler!");
        #pragma pack(pop)
        
    private:
        UART& m_coms;
        int m_id;
        
    public:
        DarwinDevice(UART& coms, int id);
        
        /**
         * @brief Reads from this device at the given memory address.
         *
         * @details
         *  This will read from the current device at the given memory address, and return an object of type TType.
         *  It will read enough consecutive bytes to fill the TType with data. For example, if you read a 2 byte wide
         *  datatype, it will read 2 consecutive bytes from the device to fill it.
         *
         * @tparam TType the type of data we are reading
         *
         * @param address the address of the data that we are reading
         *
         * @returns the data that was read from the device
         */
        template <typename TType>
        TType read(uint8_t address) {
            
            // Check that this struct is not cache alligned
            static_assert(sizeof(ReadCommand<TType>) == 8, "The compiler is adding padding to this struct, Bad compiler!");
            
            // Execute our Read command through the uart
            CommandResult result = m_coms.execute(ReadCommand<TType>(m_id, address));
            
            // If there was an error then try reading again
            if(result.data.size() != sizeof(TType)) {
                result = m_coms.execute(ReadCommand<TType>(m_id, address));
            }
            
            // If it's still bad then throw a runtime error
            if(result.data.size() != sizeof(TType)) {
                throw std::runtime_error("There was an error while trying to read from the device");
            }
            
            // Copy our resulting data into our return type
            TType data;
            memcpy(&data, result.data.data(), sizeof(TType));
            
            // Return our data
            return data;
        }
        
        /**
         * @brief Writes data to this device at the given memory address.
         *
         * @details
         *  This will write the passed object to the current device at the given memory address. It will write
         *  consecutive bytes for the size of TType. For example, if you write a 2 byte wide datatype, it will write 2
         *  consecutive bytes from the device.
         *
         * @tparam TType the type of data we are writing
         *
         * @param address   the address of the data that we are reading
         * @param data      the data that we are writing to the device
         *
         * @returns the ErrorCode that was returned by the device
         */
        template <typename TType>
        uint8_t write(uint8_t address, TType data) {
            
            // Check that this struct is not cache alligned
            static_assert(sizeof(WriteCommand<TType>) == 7 + sizeof(TType), "The compiler is adding padding to this struct, Bad compiler!");
            
            // Write our data over the UART
            CommandResult result = m_coms.execute(WriteCommand<TType>(m_id, address, data));
            
            // Return our resulting error code
            return result.header.errorcode;
        }
        
        /**
         * @brief This will send a ping request to the device.
         *
         * @returns true if the device is working, false if the device is not working
         */
        bool ping();
    };
}

#endif
