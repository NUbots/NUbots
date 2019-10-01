/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef DARWIN_DARWINDEVICE_H
#define DARWIN_DARWINDEVICE_H

#include <stdint.h>

#include "UART.h"

namespace Darwin {
/**
 * @brief The darwin device is a device on the serial port that will respond to the command types.
 *
 * @details
 *  This class is extended by the CM740, MX28 and FSR's, this allows them to access various memory locations
 *  using the common functionality provided by this class
 *
 * @author Trent Houliston
 */
class DarwinDevice {

public:
    /**
     * @brief The list of valid instructions for the CM740 and related components.
     */
    enum Instruction {
        PING       = 1,
        READ       = 2,
        WRITE      = 3,
        REG_WRITE  = 4,
        ACTION     = 5,
        RESET      = 6,
        SYNC_READ  = 130,
        SYNC_WRITE = 131,
        BULK_READ  = 146,
        BULK_WRITE = 147
    };

/**
 * @brief This struct mimics the expected data structure for a Write command.
 *
 * @details
 *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
 *  you cannot add or remove members from this type (unless for some reason the API to the CM740 changes). The
 *  template argument and parameter allows you to read any type. For example if you read with a uint16_t then
 *  it will read two bytes to the device. And if you use a struct with 3 uint16_t's in it, then you can directly
 *  read a paramter with an x, y and z bytes (e.g. the accelerometer)
 * @tparam TType the type of data to be written
 *
 * @author Trent Houliston
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    template <typename TType>
    struct ReadCommand {

        ReadCommand(uint8_t id, uint8_t address) : id(id), address(address), size(sizeof(TType)) {}

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
 *  you cannot add or remove members from this type (unless for some reason the API to the CM740 changes). The
 *  template argument and parameter allows you to write any type. For example if you write with a uint16_t then
 *  it will write two bytes to the device. And if you use a struct with 3 uint16_t's in it, then you directly
 *  write to something with an x, y and z byte.
 *
 * @tparam TType the type of data to be written
 *
 * @author Trent Houliston
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    template <typename TType>
    struct WriteCommand {

        WriteCommand(uint8_t id, uint8_t address, TType data)
            : id(id), length(3 + sizeof(TType)), address(address), data(data) {}

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
 *  you cannot add or remove members from this type (unless for some reason the API to the CM740 changes).
 *
 * @author Trent Houliston
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    struct PingCommand {
        explicit PingCommand(uint8_t id) : id(id) {}

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

protected:
    UART& coms;
    int id;

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
     * @return the data that was read from the device
     */
    template <typename TType>
    TType read(uint8_t address) {

        // Check that this struct is not cache alligned
        static_assert(sizeof(ReadCommand<TType>) == 8, "The compiler is adding padding to this struct, Bad compiler!");

        // Execute our Read command through the uart
        CommandResult result = coms.executeRead(ReadCommand<TType>(id, address));

        // If there was an error then try reading again
        if (result.data.size() != sizeof(TType)) {
            result = coms.executeRead(ReadCommand<TType>(id, address));
        }

        // If it's still bad then throw a runtime error
        if (result.data.size() != sizeof(TType)) {
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
     */
    template <typename TType>
    void write(uint8_t address, TType data) {

        // Check that this struct is not cache alligned
        static_assert(sizeof(WriteCommand<TType>) == 7 + sizeof(TType),
                      "The compiler is adding padding to this struct, Bad compiler!");

        // Write our data over the UART
        coms.executeWrite(WriteCommand<TType>(id, address, data));
    }

    /**
     * @brief Reads a specified number of bytes from this device.
     *
     * @param data Vector to store the read data in. Vector will be resized to hold the requested data.
     *       Contents of the vector will be overwritten.
     *
     * @param count Number of bytes to read from this device.
     *
     * @return the number of bytes.
     */
    size_t readBytes(std::vector<uint8_t>& data, size_t count) {

        // Make sure the vector has enough space in it.
        data.resize(count);
        return coms.readBytes(data.data(), count);
    }

    /**
     * @brief Writes a specified number of bytes to this device.
     *
     * @param data Vector of bytes to send.
     *
     * @return the number of bytes.
     */
    size_t writeBytes(const std::vector<uint8_t>& data) {

        return coms.writeBytes(data.data(), data.size());
    }

    /**
     * @brief This will send a ping request to the device.
     *
     * @return true if the device is working, false if the device is not working
     */
    bool ping();
};
}  // namespace Darwin

#endif
