/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef DYNAMIXEL_V2_SYNCWRITE_HPP
#define DYNAMIXEL_V2_SYNCWRITE_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

#include <array>
#include <cstdint>
#include <type_traits>

namespace dynamixel::v2 {

    /**
     * @brief This struct mimics the expected data structure for a Sync Write command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type. The template argument and parameter allows you to read any
     *  type. For example if you read with a uint16_t then it will read two bytes to the device. And if you use a
     *  struct with 3 uint16_t's in it, then you can directly read a parameter with an x, y and z bytes (e.g. the
     *  accelerometer)
     * @tparam T the type of data to be written
     * @tparam N the number of devices to read from
     *
     * @author Alex Biddulph
     */
    template <typename T>
    struct SyncWriteData {
        static_assert(std::is_trivial<T>::value && std::is_standard_layout<T>::value, "Values must be trivial data");
        SyncWriteData() : id(0), data() {}
        SyncWriteData(uint8_t id, T data) : id(id), data(data) {}

        /// The ID of the device that we are communicating with
        uint8_t id;
        /// The bytes that we are writing
        T data;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

    template <typename T, size_t N>
    struct SyncWriteCommand {

        constexpr SyncWriteCommand(uint16_t address, const SyncWriteData<T> (&data_arr)[N])
            : magic(0x00FDFFFF)
            , id(0xFE)
            , length(3 + sizeof(address) + sizeof(size) + N * sizeof(data_arr[0]))
            , instruction(Instruction::SYNC_WRITE)
            , address(address)
            , size(sizeof(T))
            , data{}
            , checksum(0) {
            for (size_t i = 0; i < N; ++i) {
                const_cast<SyncWriteData<T>&>(data[i]) = data_arr[i];
            }
            const_cast<uint16_t&>(checksum) = calculate_checksum(this);
        }

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (always 5)
        const uint16_t length;
        /// The instruction that we will be executing
        const uint8_t instruction;
        /// The address to read from
        const uint16_t address;
        /// The number of bytes to read
        const uint16_t size;
        /// List of device IDs to read from
        const SyncWriteData<T> data[N];
        /// Our checksum for this command
        const uint16_t checksum;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel::v2

#endif  // DYNAMIXEL_V2_SYNCWRITE_HPP
