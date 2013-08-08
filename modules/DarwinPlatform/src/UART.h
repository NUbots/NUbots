/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinPlatform.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef DARWIN_UART_H
#define DARWIN_UART_H

#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <linux/serial.h>

#include <mutex>
#include <cstring>
#include <vector>


namespace Darwin {
    namespace Packet {
        enum {
            MAGIC       = 0,
            ID          = 2,
            LENGTH      = 3,
            INSTRUCTION = 4,
            ERRBIT      = 4,
            PARAMETER   = 5
        };
    }  // namespace Packet

    namespace ErrorCode {
        enum {
            NO_RESPONSE     = -1,
            NONE            = 0x0000,
            INPUT_VOLTAGE   = 0x0001,
            ANGLE_LIMIT     = 0x0002,
            OVERHEATING     = 0x0004,
            RANGE           = 0x0008,
            CHECKSUM        = 0x0010,
            OVERLOAD        = 0x0020,
            INSTRUCTION     = 0x0040,
            CORRUPT_DATA    = 0x0080
        };
    }  // namespace ErrorCode

    // This is the header that is contained in the CommandResult
    #pragma pack(push, 1)  // Make sure that this struct is not cache alligned
    struct Header {
        uint8_t id = -1;
        uint8_t length = 0;
        uint8_t errorcode = -1;
    };
    // Check that this struct is not cache alligned
    static_assert(sizeof(Header) == 3, "The compiler is adding padding to this struct, Bad compiler!");
    #pragma pack(pop)

    // This is the object that is returned when a command is run
    struct CommandResult {
        Header header;
        std::vector<uint8_t> data;
        uint8_t checksum;
    };

    // This value calculates the checksum for a packet (the command argument is assumed to be in the CM730 format)
    uint8_t calculateChecksum(void* command);
    uint8_t calculateChecksum(const CommandResult& result);

    class UART {
    private:
        int fd;
        std::mutex mutex;

        int configure(double baud);

    public:
        explicit UART(const char* name);

        CommandResult readPacket();

        template <typename TPacket>
        CommandResult execute(const TPacket& command) {

            // Lock the mutex
            std::unique_lock<std::mutex> lock(mutex);

            // We flush our buffer, just in case there was anything random in it
            tcflush(fd, TCIFLUSH);

            // Write our command to the UART
            write(fd, &command, sizeof(TPacket));

            return readPacket();
        }

        std::vector<CommandResult> executeBulk(const std::vector<uint8_t>& command);
        void executeBroadcast(const std::vector<uint8_t>& command);
    };
}  // namespace Darwin

#endif
