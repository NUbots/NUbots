#include "uart.h"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace utility {
namespace io {

    uart::uart() : device(""), fd(-1) {}

    uart::uart(const std::string& device, const unsigned int& baud) : device(device), fd(-1) {
        open(device, baud);
    }

    void uart::open(const std::string& device, const unsigned int& baud) {

        // Open our file descriptor for read/write with no controlling TTY and nonblock mode
        fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd >= 0) {

            // We want exclusive access to this uart
            if (ioctl(fd, TIOCEXCL) == -1) {
                throw std::runtime_error("Failed to set exclusive access for " + device);
            }

            // Set our baud rate
            set_baud(baud);
        }
        else {
            throw std::runtime_error("Failed to connect to " + device);
        }
    }

    void uart::close() {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    uart::~uart() {
        close();
    }

    int uart::native_handle() {
        return fd;
    }

    void uart::set_baud(const int& baud) {

        // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate
        // aliasing" http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
        termios tio;
        memset(&tio, 0, sizeof(tio));
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
#elif defined(B115200)
            case 115200: tio.c_cflag = B115200; break;
#elif defined(B230400)
            case 230400: tio.c_cflag = B230400; break;
#elif defined(B460800)
            case 460800: tio.c_cflag = B460800; break;
#elif defined(B500000)
            case 500000: tio.c_cflag = B500000; break;
#elif defined(B576000)
            case 576000: tio.c_cflag = B576000; break;
#elif defined(B921600)
            case 921600: tio.c_cflag = B921600; break;
#elif defined(B1000000)
            case 1000000: tio.c_cflag = B1000000; break;
#elif defined(B1152000)
            case 1152000: tio.c_cflag = B1152000; break;
#elif defined(B1500000)
            case 1500000: tio.c_cflag = B1500000; break;
#elif defined(B2000000)
            case 2000000: tio.c_cflag = B2000000; break;
#elif defined(B2500000)
            case 2500000: tio.c_cflag = B2500000; break;
#elif defined(B3000000)
            case 3000000: tio.c_cflag = B3000000; break;
#elif defined(B3500000)
            case 3500000: tio.c_cflag = B3500000; break;
#elif defined(B4000000)
            case 4000000: tio.c_cflag = B4000000; break;
#endif
            default:
                tio.c_cflag |= B38400;
                known_baud = false;
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
        tcsetattr(fd, TCSANOW, &tio);

        if (!known_baud) {

            // Here we do the baud rate aliasing in order to set the custom baud rate
            serial_struct serinfo;

            // Get our serial_info from the system
            if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
                throw std::runtime_error("There was an error setting the baud rate for " + device);
            }

            // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
            serinfo.flags &= ~ASYNC_SPD_MASK;
            serinfo.flags |= ASYNC_SPD_CUST;

            // Set our serial port to use low latency mode (otherwise the USB driver buffers for 16ms before sending
            // data)
            serinfo.flags |= ASYNC_LOW_LATENCY;

            // Set our custom divsor for our speed
            serinfo.custom_divisor = serinfo.baud_base / baud;

            // Set our custom speed in the system
            if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
                throw std::runtime_error("There was an error setting the baud rate for " + device);
            }
        }

        // Flush our connection to remove all existing data
        tcflush(fd, TCIFLUSH);
    }

    bool uart::connected() const {
        return !(fcntl(fd, F_GETFL) < 0 && errno == EBADF);
    }

    ssize_t uart::read(void* buf, size_t count) {
        return ::read(fd, buf, count);
    }

    ssize_t uart::write(const void* buf, size_t count) {
        return ::write(fd, buf, count);
    }
}  // namespace io
}  // namespace utility
