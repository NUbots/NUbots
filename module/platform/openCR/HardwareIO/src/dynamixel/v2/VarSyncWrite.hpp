#ifndef DYNAMIXEL_V2_VAR_SYNCWRITE_HPP
#define DYNAMIXEL_V2_VAR_SYNCWRITE_HPP

#ifndef DYNAMIXEL_V2_INTERNAL
    #error Do not include this file on its own. Include Dynamixel.hpp instead.
#endif

#include <array>
#include <type_traits>

namespace dynamixel {
    namespace v2 {

/**
 * @brief This struct mimics the expected data structure for a Sync Write command of variable runtime defined
 * length.
 */
#pragma pack(push, 1)  // Make it so that the compiler reads this struct "as is" (no padding bytes)
        template <typename T>
        struct VarSyncWriteCommand {

            VarSyncWriteCommand(uint16_t address, const std::vector<SyncWriteData<T>>& data)
                : magic(0x00FDFFFF)
                , id(0xFE)
                , length(3 + sizeof(address) + sizeof(size) + data.size() * sizeof(data[0]))
                , instruction(Instruction::SYNC_WRITE)
                , address(address)
                , size(sizeof(T))
                , data(data)
                , checksum(calculateChecksum(this)) {}

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
            const std::vector<SyncWriteData<T>> data;
            /// Our checksum for this command
            const uint16_t checksum;
        };
#pragma pack(pop)


    }  // namespace v2
}  // namespace dynamixel

#endif  // DYNAMIXEL_V2_VAR_SYNCWRITE_HPP
