#ifndef UTILITY_IO_UART_H
#define UTILITY_IO_UART_H

#include <string>

namespace utility {
namespace io {

    class uart {
    private:
        std::string device;
        int fd;

        void set_baud(const int& baud);

    public:
        using char_type = char;

        explicit uart();

        explicit uart(const std::string& device, const unsigned int& baud_rate = 115200);

        ~uart();

        int native_handle();

        bool good() const;

        void flush();

        int get();

        void set_rts(int value);
        void set_dtr(int value);

        ssize_t read(void* buf, size_t count);

        ssize_t write(const void* buf, size_t count);
        ssize_t blocking_write(const void* buf, size_t count);

        void open(const std::string& device, const unsigned int& baud_rate = 115200);

        void close();
    };

}  // namespace io
}  // namespace utility

#endif  // UTILITY_IO_UART_H