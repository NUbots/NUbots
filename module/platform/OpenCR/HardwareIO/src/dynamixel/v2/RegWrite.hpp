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
#ifndef DYNAMIXEL_V2_REGWRITE_HPP
#define DYNAMIXEL_V2_REGWRITE_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

namespace dynamixel::v2 {
    /**
     * @brief This struct mimics the expected data structure for a RegWrite command.
     *
     * @details
     *  Instruction that registers the Instruction Packet to a standby status; Packet is later executed through
     *  the Action command. This type has it's members arranged in the same way as a raw array of this command would.
     *  Because of this you cannot add or remove members from this type. The template argument and parameter allows you
     *  to write any type. For example if you write with a uint16_t then it will write two bytes to the device. And if
     *  you use a struct with 3 uint16_t's in it, then you directly write to something with an x, y and z byte.
     *
     * @tparam T the type of data to be written
     *
     * @author Alex Biddulph
     */
    template <typename T>
    struct RegWriteCommand {

        RegWriteCommand(uint8_t id, uint16_t address, T data)
            : magic(0x00FDFFFF)
            , id(id)
            , length(3 + sizeof(address) + sizeof(T))
            , instruction(Instruction::REG_WRITE)
            , address(address)
            , data(data)
            , checksum(calculate_checksum(this)) {}

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (3 plus however many bytes we are writing)
        const uint16_t length;
        /// The instruction that we will be executing
        const uint8_t instruction;
        /// The address we are writing to
        const uint16_t address;
        /// The bytes that we are writing
        const T data;
        /// Our checksum for this command
        const uint16_t checksum;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel::v2

#endif  // DYNAMIXEL_V2_REGWRITE_HPP
