#ifndef DYNAMIXEL_V2_WRITE_HPP
#define DYNAMIXEL_V2_WRITE_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

namespace dynamixel::v2 {
    /**
     * @brief This struct mimics the expected data structure for a Write command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would. Because of this
     *  you cannot add or remove members from this type. The template argument and parameter allows you to write
     *  any type. For example if you write with a uint16_t then it will write two bytes to the device. And if you
     *  use a struct with 3 uint16_t's in it, then you directly write to something with an x, y and z byte.
     *
     * @tparam T the type of data to be written
     *
     * @author Trent Houliston
     */
    template <typename T>
    struct WriteCommand {

        WriteCommand(uint8_t id, uint16_t address, T data)
            : magic(0x00FDFFFF)
            , id(id)
            , length(3 + sizeof(address) + sizeof(T))
            , instruction(Instruction::WRITE)
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

#endif  // DYNAMIXEL_V2_WRITE_HPP
