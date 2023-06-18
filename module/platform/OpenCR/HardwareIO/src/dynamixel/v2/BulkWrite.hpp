#ifndef DYNAMIXEL_V2_BULKWRITE_HPP
#define DYNAMIXEL_V2_BULKWRITE_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

#include <array>
#include <type_traits>

namespace dynamixel::v2 {

    /**
     * @brief This struct mimics the expected data structure for a Bulk Write command.
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
    struct BulkWriteData {
        static_assert(std::is_trivial<T>::value && std::is_standard_layout<T>::value, "Values must be trivial data");
        BulkWriteData(uint8_t id, uint16_t address, T data) : id(id), address(address), size(sizeof(T)), data(data) {}

        /// The ID of the device that we are communicating with
        uint8_t id;
        /// The starting address to read from
        uint16_t address;
        /// The number of bytes to read
        uint16_t size;
        /// The bytes that we are writing
        T data;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)


    template <typename T, size_t N>
    struct BulkWriteCommand {

        BulkWriteCommand(const std::array<BulkWriteData<T>, N>& data)
            : magic(0x00FDFFFF)
            , id(0xFE)
            , length(3 + N * sizeof(data[0]))
            , instruction(Instruction::BULK_WRITE)
            , data(data)
            , checksum(calculate_checksum(this)) {}

        /// Magic number that heads up every packet
        const uint32_t magic;
        /// The ID of the device that we are communicating with
        const uint8_t id;
        /// The total length of the data packet (always 5)
        const uint16_t length;
        /// The instruction that we will be executing
        const uint8_t instruction;
        /// DEvice data
        const std::array<BulkWriteData<T>, N> data;
        /// Our checksum for this command
        const uint16_t checksum;
    } __attribute__((packed));  // Make it so that the compiler reads this struct "as is" (no padding bytes)

}  // namespace dynamixel::v2

#endif  // DYNAMIXEL_V2_BULKWRITE_HPP
