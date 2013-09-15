/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef DARWIN_UART_H
#define DARWIN_UART_H

#include <cassert>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <linux/serial.h>

#include <mutex>
#include <cstring>
#include <vector>


namespace darwin {
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

    /**
     * @brief Communicates with the components via the UART (through a USB TTY device)
     *
     * @details
     *  This class handles the communication with the hardware, It has methods that implement the ZigBee protocol used
     *    in the darwin in order to communicate with the various devices.
     *
     * @author Trent Houliston
     */
    class UART {
    private:
        /// @brief The file descriptor for the USB TTY device we use to communicate with the devices
        int fd;
        /// @brief A mutex which is used for flow control on the USB TTY device
        std::mutex mutex;

        /**
         * @brief Configures our serial port to use the passed Baud rate
         *
         * @param baud the baud rate to use
         *
         * @return if the configuration was successful
         */
        bool configure(double baud);

    public:
        /**
         * @brief Constructs a new UART instance using the passed device path as the TTY device
         *
         * @param name the path to the USB TTY device
         */
        explicit UART(const char* name);

        /**
         * @brief reads a single packet back from the uart, and returns error codes if they timeout
         *
         * @return The command result, or a command result with an error flag if there was an error
         */
        CommandResult readPacket();

        /**
         * @brief Executes a passed packet and then waits for a response, Used for single commands (read write ping)
         *
         * @tparam TPacket the type of packet we are executing
         *
         * @param command the command we are executing
         *
         * @return the return value from executing this command
         */
        template <typename TPacket>
        CommandResult execute(const TPacket& command) {

            // Lock the mutex
            std::lock_guard<std::mutex> lock(mutex);

            // We flush our buffer, just in case there was anything random in it
            tcflush(fd, TCIFLUSH);

            // Write our command to the UART
            int written = write(fd, &command, sizeof(TPacket));
            assert(written == sizeof(TPacket));
            // If compiled with NDEBUG then technically written is unused, suppress that warning
            (void) written;

            // Read the packet that we get in response
            return readPacket();
        }

        /**
         * @brief This is used to execute a bulk read request.
         *
         * @param command the packet that we are going to send to get the response
         *
         * @return a vector of command results, one for each of the responding devices
         */
        std::vector<CommandResult> executeBulk(const std::vector<uint8_t>& command);

        /**
         * @brief This is used to execute a broadcast command (to the broadcast address), these expect no response
         *
         * @param command the command to execute
         */
        void executeBroadcast(const std::vector<uint8_t>& command);
    };
}  // namespace Darwin

#endif
