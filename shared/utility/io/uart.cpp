/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#include "uart.hpp"

#include <cerrno>
#include <cstring>
#include <fmt/format.h>
#include <system_error>
#include <utility>

extern "C" {
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
}

namespace utility::io {

    uart::uart(const std::string& device, const unsigned int& baud)
        : device(device), fd(::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) {

        if (fd >= 0) {

            // We want exclusive access to this uart
            if (::ioctl(fd, TIOCEXCL) == -1) {
                throw std::system_error(errno,
                                        std::system_category(),
                                        fmt::format("Failed to set exclusive access for {}", device));
            }

            // Set our baud rate
            set_baud(baud);
        }
        else {
            throw std::system_error(errno, std::system_category(), fmt::format("Failed to connect to {}", device));
        }
    }

    uart::uart(uart&& other) noexcept : device(std::move(other.device)), fd(std::exchange(other.fd, -1)) {}

    uart& uart::operator=(uart&& rhs) noexcept {
        if (this != &rhs) {
            this->close();  // Close if we already have a device open
            this->device = std::move(rhs.device);
            this->fd     = std::exchange(rhs.fd, -1);
        }
        return *this;
    }

    void uart::close() {
        if (fd >= 0) {
            ::close(fd);
        }
        fd = -1;
    }

    uart::~uart() {
        this->close();
    }

    int uart::native_handle() const {
        return fd;
    }

    void uart::set_baud(const unsigned int& baud) {

        // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate
        // aliasing" http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
        ::termios tio{};
        std::memset(&tio, 0, sizeof(tio));
        // B38400 for aliasing, CS8 (8bit,no parity,1 stopbit), CLOCAL (local connection, no modem control), CREAD
        // (enable receiving characters)

        bool known_baud = true;

        switch (baud) {
            case 50: tio.c_cflag = B50; break;
            case 75: tio.c_cflag = B75; break;
            case 110: tio.c_cflag = B110; break;
            case 134: tio.c_cflag = B134; break;
            case 150: tio.c_cflag = B150; break;
            case 200: tio.c_cflag = B200; break;
            case 300: tio.c_cflag = B300; break;
            case 600: tio.c_cflag = B600; break;
            case 1200: tio.c_cflag = B1200; break;
            case 1800: tio.c_cflag = B1800; break;
            case 2400: tio.c_cflag = B2400; break;
            case 4800: tio.c_cflag = B4800; break;
            case 9600: tio.c_cflag = B9600; break;
            case 19200: tio.c_cflag = B19200; break;
            case 38400: tio.c_cflag = B38400; break;
#if defined(B57600)
            case 57600: tio.c_cflag = B57600; break;
#endif
#if defined(B115200)
            case 115200: tio.c_cflag = B115200; break;
#endif
#if defined(B230400)
            case 230400: tio.c_cflag = B230400; break;
#endif
#if defined(B460800)
            case 460800: tio.c_cflag = B460800; break;
#endif
#if defined(B500000)
            case 500000: tio.c_cflag = B500000; break;
#endif
#if defined(B576000)
            case 576000: tio.c_cflag = B576000; break;
#endif
#if defined(B921600)
            case 921600: tio.c_cflag = B921600; break;
#endif
#if defined(B1000000)
            case 1000000: tio.c_cflag = B1000000; break;
#endif
#if defined(B1152000)
            case 1152000: tio.c_cflag = B1152000; break;
#endif
#if defined(B1500000)
            case 1500000: tio.c_cflag = B1500000; break;
#endif
#if defined(B2000000)
            case 2000000: tio.c_cflag = B2000000; break;
#endif
#if defined(B2500000)
            case 2500000: tio.c_cflag = B2500000; break;
#endif
#if defined(B3000000)
            case 3000000: tio.c_cflag = B3000000; break;
#endif
#if defined(B3500000)
            case 3500000: tio.c_cflag = B3500000; break;
#endif
#if defined(B4000000)
            case 4000000: tio.c_cflag = B4000000; break;
#endif
            default:
                tio.c_cflag = B38400;
                known_baud  = false;
                break;
        }

        tio.c_cflag |= CS8 | CLOCAL | CREAD;

        // IGNPAR (ignore incoming parity bits as we don't have parity)
        tio.c_iflag = IGNPAR;
        // 0 means raw output
        tio.c_oflag = 0;
        // No ICANON so we read immediately rather then line by line
        tio.c_lflag &= ~ICANON;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN]  = 0;
        // Set the settings
        ::tcsetattr(fd, TCSANOW, &tio);

        // Here we do the baud rate aliasing in order to set the custom baud rate
        serial_struct serinfo{};

        // Get our serial_info from the system
        if (::ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    fmt::format("There was an error setting the baud rate for {}", device));
        }

        if (!known_baud) {
            // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
            serinfo.flags &= int(~ASYNC_SPD_MASK);
            serinfo.flags |= ASYNC_SPD_CUST;
            serinfo.custom_divisor = int(serinfo.baud_base / baud);
        }

        // Set our serial port to use low latency mode
        // otherwise the USB driver buffers for 16ms before sending data
        serinfo.flags |= ASYNC_LOW_LATENCY;

        // Set our custom speed in the system
        if (::ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    fmt::format("There was an error setting serial info flags for {}", device));
        }

        // Flush our connection to remove all existing data
        ::tcflush(fd, TCIFLUSH);
    }

    bool uart::connected() const {
        return !(::fcntl(fd, F_GETFL) < 0 && errno == EBADF);
    }

    // `read` `write` and `get` can technically be `const`, but that's deceptive because they change the state of the
    // buffer associated with the fd, so it can be thought of as "changing" the fd
    // NOLINTNEXTLINE(readability-make-member-function-const)
    ssize_t uart::read(void* buf, size_t count) {
        return ::read(fd, buf, count);
    }
    // NOLINTNEXTLINE(readability-make-member-function-const)
    ssize_t uart::write(const void* buf, size_t count) {
        return ::write(fd, buf, count);
    }

    int uart::available() const {
        if (connected()) {
            int nbytes = 0;
            ::ioctl(fd, FIONREAD, &nbytes);
            return nbytes;
        }
        return -1;
    }

    int uart::get() {
        uint8_t c = 0;
        if (read(&c, 1) == 1) {
            return c;
        }
        return -1;
    }

    void uart::flush() {
        ::tcflush(fd, TCIFLUSH);
    }

}  // namespace utility::io
