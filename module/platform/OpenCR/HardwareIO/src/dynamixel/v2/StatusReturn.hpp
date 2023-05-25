#ifndef DYNAMIXEL_V2_STATUSRETURN_HPP
#define DYNAMIXEL_V2_STATUSRETURN_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

namespace dynamixel::v2 {

    // Result Fail       Failed to process the sent Instruction Packet
    // Instruction Error Undefined Dynamixel.hppas been used
    //                   Action has been used without Reg Write
    // CRC Error         CRC of the sent Packet does not match
    // Data Range Error  Data to be written in the corresponding Address is outside the range of the min/max value
    // Data Length Error Data that is shorter than the data length of the corresponding Address
    //                   (e.g: using 2 bytes of a item that has been defined as 4 bytes)
    // Data Limit Error  Data to be written in the corresponding Address is outside of the Limit value
    // Access Error      Attempt to write a value in an Address that is Read Only or has not been defined
    //                   Attempt to read a value in an Address that is Write Only or has not been defined
    //                   Attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock)
    enum class CommandError : uint8_t {
        NO_ERROR          = 0x00,
        RESULT_FAIL       = 0x01,
        INSTRUCTION_ERROR = 0x02,
        CRC_ERROR         = 0x03,
        RANGE_ERROR       = 0x04,
        LENGTH_ERROR      = 0x05,
        LIMIT_ERROR       = 0x06,
        ACCESS_ERROR      = 0x07
    };

    /**
     * @brief This struct mimics the expected data structure for a Status Return command.
     *
     * @details
     *  This type has it's members arranged in the same way as a raw array of this command would.
     *  Because of this you cannot add or remove members from this type. The template argument and parameter allows you
     *  to write any type. For example if you write with a uint16_t then it will write two bytes to the device. And if
     *  you use a struct with 3 uint16_t's in it, then you directly write to something with an x, y and z byte.
     *
     * @tparam T the type of data to be written
     *
     * @author Alex Biddulph
     */
    template <int N>
    struct StatusReturnCommand {

        StatusReturnCommand(uint8_t id, const CommandError& error, const std::vector<uint8_t>& data, uint16_t checksum)
            : magic(0x00FDFFFF)
            , id(id)
            , length(N + 4)
            , instruction(Instruction::STATUS_RETURN)
            , error(error)
            , data(data)
            , checksum(checksum) {}

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (4 plus the size of data)
        const uint16_t length;
        /// Will always be Instruction::STATUS_RETURN
        const uint8_t instruction;
        /// Error value
        const CommandError error;
        /// Expected return data
        std::array<uint8_t, N> data;
        /// Our checksum for this command
        const uint16_t checksum;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel::v2

#endif  // DYNAMIXEL_V2_STATUSRETURN_HPP
