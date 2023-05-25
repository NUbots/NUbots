#ifndef DYNAMIXEL_V2_READ_HPP
#define DYNAMIXEL_V2_READ_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

namespace dynamixel::v2 {

    /**
     * @brief This struct mimics the expected data structure for a Read command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type. The template argument and parameter allows you to read any
     *  type. For example if you read with a uint16_t then it will read two bytes to the device. And if you use a
     *  struct with 3 uint16_t's in it, then you can directly read a parameter with an x, y and z bytes (e.g. the
     *  accelerometer)
     * @tparam T the type of data to be read
     *
     * @author Trent Houliston
     */
    struct ReadCommand {

        ReadCommand(uint8_t id, uint16_t address, uint16_t size)
            : magic(0x00FDFFFF)
            , id(id)
            , length(7)
            , instruction(Instruction::READ)
            , address(address)
            , size(size)
            , checksum(calculate_checksum(this)) {}

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
        /// Our checksum for this command
        const uint16_t checksum;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)
    // Check that this struct is not cache aligned
    static_assert(sizeof(ReadCommand) == 14, "The compiler is adding padding to this struct, Bad compiler!");

}  // namespace dynamixel::v2

#endif  // DYNAMIXEL_V2_READ_HPP
