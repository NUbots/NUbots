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

#ifndef DARWIN_UART_H
#define DARWIN_UART_H

#include <errno.h>
#include <linux/serial.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <mutex>
#include <vector>

#include "extension/Configuration.h"


namespace Darwin {
namespace Packet {
    enum { MAGIC = 0, ID = 2, LENGTH = 3, INSTRUCTION = 4, ERRBIT = 4, PARAMETER = 5 };
}  // namespace Packet

namespace ErrorCode {
    enum {
        NONE          = 0x00,
        INPUT_VOLTAGE = 0x01,
        ANGLE_LIMIT   = 0x02,
        OVERHEATING   = 0x04,
        RANGE         = 0x08,
        CHECKSUM      = 0x10,
        OVERLOAD      = 0x20,
        INSTRUCTION   = 0x40,
        CORRUPT_DATA  = 0x80,
        NO_RESPONSE   = 0xFF
    };
}  // namespace ErrorCode

// This is the header that is contained in the CommandResult
#pragma pack(push, 1)  // Make sure that this struct is not cache aligned
struct Header {
    Header() {}
    uint8_t id        = -1;
    uint8_t length    = 0;
    uint8_t errorcode = -1;
};
// Check that this struct is not cache aligned
static_assert(sizeof(Header) == 3, "The compiler is adding padding to this struct, Bad compiler!");
#pragma pack(pop)

// This is the object that is returned when a command is run
struct CommandResult {
    CommandResult() : header(), data(), checksum(0) {}
    Header header;
    std::vector<uint8_t> data;
    uint8_t checksum;
};

// This value calculates the checksum for a packet (the command argument is assumed to be in the CM740 format)
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
    const char* devName;

    // We will wait this long for an initial packet header
    int PACKET_WAIT = 10000;
    // We will only wait a maximum of BYTE_WAIT microseconds between bytes in a packet (assumes baud of 1000000bps)
    int BYTE_WAIT              = 1000;
    int BUS_RESET_WAIT_TIME_uS = 100000;

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

    /**
     * @brief Connects to the serial port
     */
    void connect();

    /**
     * @brief Reconnects to the serial port
     */
    void reconnect();

public:
    void setConfig(const extension::Configuration& config);
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
    CommandResult executeRead(const TPacket& command) {

        // Lock the mutex
        std::lock_guard<std::mutex> lock(mutex);

        // Write our command to the UART
        int written = write(fd, &command, sizeof(TPacket));

        // Wait until we finish writing before continuing (no buffering)
        tcdrain(fd);

        // We flush our buffer, just in case there was anything random in it
        tcflush(fd, TCIFLUSH);

        assert(written == sizeof(TPacket));
        // If compiled with NDEBUG then technically written is unused, suppress that warning
        (void) written;

        // Read the packet that we get in response
        return readPacket();
    }

    /**
     * @brief Executes a passed packet and does not wait for a response (for writes)
     *
     * @tparam TPacket the type of packet we are executing
     *
     * @param command the command we are executing
     */
    template <typename TPacket>
    void executeWrite(const TPacket& command) {

        // Lock the mutex
        std::lock_guard<std::mutex> lock(mutex);

        // Write our command to the UART
        int written = write(fd, &command, sizeof(TPacket));

        // Wait until we finish writing before continuing (no buffering)
        tcdrain(fd);

        assert(written == sizeof(TPacket));
        // If compiled with NDEBUG then technically written is unused, suppress that warning
        (void) written;
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

    /**
     * @brief Reads a specified number of bytes from the serial port.
     *
     * @param buf Pointer to a location to store the read data.
     *
     * @param count Number of bytes to read from the serial port.
     *
     * @return the number of bytes.
     */
    size_t readBytes(void* buf, size_t count);

    /**
     * @brief Writes a specified number of bytes to the serial port.
     *
     * @param buf Pointer to the data to write.
     *
     * @param count Number of bytes to write to the serial port.
     *
     * @return the number of bytes.
     */
    size_t writeBytes(const void* buf, size_t count);
};
}  // namespace Darwin

#endif
