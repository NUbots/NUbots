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

#include "UART.h"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <iostream>
#include <thread>

namespace Darwin {
uint8_t calculateChecksum(void* command) {

    uint8_t* data    = static_cast<uint8_t*>(command);
    uint8_t checksum = 0x00;
    // Skip over the magic numbers and checksum the rest of the packet
    for (int i = 2; i < data[Packet::LENGTH] + 3; ++i) {
        checksum += data[i];
    }
    return (~checksum);
}

uint8_t calculateChecksum(const CommandResult& result) {

    uint8_t checksum = 0x00;

    checksum += result.header.id;
    checksum += result.header.length;
    checksum += result.header.errorcode;

    for (size_t i = 0; i < result.data.size(); ++i) {
        checksum += result.data[i];
    }

    return (~checksum);
}

void UART::setConfig(const extension::Configuration& config) {
    PACKET_WAIT            = config["packet_wait"].as<int>();
    BYTE_WAIT              = config["byte_wait"].as<int>();
    BUS_RESET_WAIT_TIME_uS = config["bus_reset_wait_time_us"].as<int>();
}

UART::UART(const char* name) : devName(name), fd(-1), mutex() {
    connect();
}

void UART::connect() {
    double baud = 1000000;  // (1mb/s)

    fd = open(devName, O_RDWR | O_NOCTTY | O_NONBLOCK);

    // If we have a valid file handle, and were able to configure it correctly (custom baud)
    if (fd < 0 || !configure(baud)) {
        // There was an exception connecting
        throw std::runtime_error("There was an error setting up the serial connection to the CM740");
    }

    NUClear::log<NUClear::DEBUG>("Device '", devName, "' successfully opened.");
}

bool UART::configure(double baud) {

    // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate aliasing"
    // http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
    termios tio;
    memset(&tio, 0, sizeof(tio));
    // B38400 for alising, CS8 (8bit,no parity,1 stopbit), CLOCAL (local connection, no modem contol), CREAD (enable
    // receiving characters)
    tio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    // IGNPAR (ignore incoming parity bits as we don't have parity)
    tio.c_iflag = IGNPAR;
    // 0 means raw output
    tio.c_oflag = 0;
    // No ICANON so we read immediantly rather then line by line
    tio.c_lflag &= ~ICANON;
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN]  = 0;
    // Set the settings
    tcsetattr(fd, TCSANOW, &tio);

    // Here we do the baud rate aliasing in order to set the custom baud rate
    serial_struct serinfo;

    // Get our serial_info from the system
    if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
        return false;
    }

    // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;

    // Set our serial port to use low latency mode (otherwise the USB driver buffers for 16ms before sending data)
    serinfo.flags |= ASYNC_LOW_LATENCY;

    // Set our custom divsor for our speed
    serinfo.custom_divisor = serinfo.baud_base / baud;

    // Set our custom speed in the system
    if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
        throw std::runtime_error("There was an error configuring up the serial connection to the CM740");
        return false;
    }

    // Flush our connection to remove all existing data
    tcflush(fd, TCIFLUSH);

    NUClear::log<NUClear::DEBUG>("Device '", devName, "' successfully configured.");
    return true;
}

size_t UART::writeBytes(const void* buf, size_t count) {
    uint8_t reconnects = 0;
    int bytesWritten   = 0;

    // We flush our buffer, just in case there was anything random in it
    tcflush(fd, TCIFLUSH);

    while ((bytesWritten != (int) count) && (reconnects < 3)) {
        bytesWritten = write(fd, buf, count);

        if (bytesWritten < (int) count) {
            reconnect();
            reconnects++;
        }
    }

    if (reconnects > 0) {
        std::cout << "Bytes Written: " << (int) bytesWritten << " Reconnects: " << (int) reconnects << "\r"
                  << std::endl;
    }

    assert(reconnects < 3);
    (void) reconnects;  // Make the compiler happy when NDEBUG is set

    return (bytesWritten);
}

size_t UART::readBytes(void* buf, size_t count) {
    uint8_t reconnects = 0;
    int bytesRead      = 0;

    while ((bytesRead != (int) count) && (reconnects < 3)) {
        bytesRead = read(fd, buf, count);

        if ((errno == EAGAIN) || ((bytesRead < (int) count) && (bytesRead > 0))) {
            break;
        }

        if (bytesRead < 0) {
            reconnect();
            reconnects++;
        }
    }

    if (reconnects > 0) {
        std::cout << "Bytes Read: " << (int) bytesRead << " Reconnects: " << (int) reconnects << " Data: ";
        for (int i = 0; i < bytesRead; i++) {
            std::cout << (int) (*((uint8_t*) buf + i)) << " ";
        }
        std::cout << "\r" << std::endl;
    }

    assert(reconnects < 3);
    (void) reconnects;  // Make the compiler happy when NDEBUG is set

    return (bytesRead);
}

void UART::reconnect() {
    std::cout << "Failed to read from '" << devName << "' Error: '" << strerror(errno) << "'\r" << std::endl;
    NUClear::log<NUClear::WARN>("Failed to read from '", devName, "' Error: '", strerror(errno), "'");

    // Close the connection.
    close(fd);

    // Sleep for a brief period to allow things to clean up and reconnect.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Connect to the serial port again.
    connect();
}

CommandResult UART::readPacket() {

    // Our result
    CommandResult result;

    // Clear our connection set and put in our serial device
    fd_set connectionset;
    timeval timeout;
    timeout.tv_sec = 0;
    FD_ZERO(&connectionset);
    FD_SET(fd, &connectionset);

    // First we find the packet magic number in order to sync with the channel
    timeout.tv_usec = PACKET_WAIT;
    for (int sync = 0; sync < 2;) {
        if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {
            uint8_t byte;

            if (readBytes(&byte, 1) > 0) {
                sync = (byte == 0xFF) ? (sync + 1) : 0;
            }
        }
        else {
            // The result is pre initialized as a timeout
            return result;
        }
    }

    // We now are now waiting for 4 bytes
    timeout.tv_usec      = BYTE_WAIT * sizeof(Header);
    uint8_t* headerBytes = reinterpret_cast<uint8_t*>(&result.header);
    for (size_t done = 0; done < sizeof(Header);) {
        if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {
            done += readBytes(&headerBytes[done], sizeof(Header) - done);
        }
        else {
            // The result is pre initialized as a timeout
            return result;
        }
    }

    // Here we adjust our "length" to mean the length of the payload rather then the length of bytes after the length
    int length = 0;
    if (result.header.length < 2) {
        std::cout << "Length: " << (int) result.header.length << ", " << (int) (result.header.length - 2) << "\r"
                  << std::endl;
    }

    else {
        length = result.header.length - 2;
    }

    // We now are now waiting for our data
    timeout.tv_usec = BYTE_WAIT * length;
    result.data.resize(length);
    for (int done = 0; done < length;) {
        if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {
            done += readBytes(&result.data[done], length - done);
        }
        else {
            // Set our packet header to timeout and return it
            result.header.errorcode = ErrorCode::NO_RESPONSE;
            return result;
        }
    }

    // We just read the checksum now
    timeout.tv_usec = 2000;
    if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {
        // If we fail to read the checksum then just assume corrupt data.
        if (readBytes(&result.checksum, 1) != 1) {
            result.header.errorcode |= ErrorCode::CORRUPT_DATA;
            return result;
        }
    }
    else {
        // If all we are missing is the checksum, just assume the data is corrupt
        result.header.errorcode |= ErrorCode::CORRUPT_DATA;
        return result;
    }

    // Validate our checksum
    if (result.checksum != calculateChecksum(result)) {
        CommandResult result;
        result.checksum = 0;  // GCC doesn't like that this isn't initalized
        result.header.errorcode |= ErrorCode::CORRUPT_DATA;
        return result;
    }

    // Return the packet we recieved
    return result;
}

std::vector<CommandResult> UART::executeBulk(const std::vector<uint8_t>& command) {

    // We can work out how many responses to expect based on our packets length
    int responses = (command[Packet::LENGTH] - 3) / 3;
    std::vector<CommandResult> results(responses);

    // Lock our mutex
    std::lock_guard<std::mutex> lock(mutex);

    writeBytes(command.data(), command.size());

    // Read our responses for each of the packets
    for (int i = 0; i < responses; ++i) {
        results[i] = readPacket();
        // If we get a timeout don't wait for other packets (other errors are fine)
        if (results[i].header.errorcode == ErrorCode::NO_RESPONSE) {
            // Set our timedout IDs
            results[i].header.id = command[7 + i * 3];
            for (i++; i < responses; ++i) {
                results[i].header.id        = command[7 + i * 3];
                results[i].header.errorcode = ErrorCode::CORRUPT_DATA;
            }

            // Wait for 100ms for the bus to reset
            std::this_thread::sleep_for(std::chrono::microseconds(BUS_RESET_WAIT_TIME_uS));

            // Stop trying to read future packets
            break;
        }
    }

    return results;
}

void UART::executeBroadcast(const std::vector<uint8_t>& command) {

    // Lock our mutex
    std::lock_guard<std::mutex> lock(mutex);

    writeBytes(command.data(), command.size());

    // There are no responses for broadcast commands
}
}  // namespace Darwin
