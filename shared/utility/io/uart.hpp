#ifndef UTILITY_IO_UART_HPP
#define UTILITY_IO_UART_HPP

#include <string>

namespace utility {
namespace io {

    /**
     * @brief Class for managing a connection to a serial device
     */
    class uart {
    private:
        std::string device;
        int fd;

        /**
         * @brief Set the baud rate of the device
         *
         * @param baud the baud rate to set
         */
        void set_baud(const int& baud);

    public:
        /**
         * @brief The type that is read by this device
         */
        using char_type = char;

        /**
         * @brief Construct a new unconnected uart class
         */
        uart();

        /**
         * @brief Create a new uart class that is connected to the device `device`
         *
         * @param device the file path of the device to connect to
         * @param int the baud rate to connect to
         */
        uart(const std::string& device, const unsigned int& baud_rate = 57600);

        /**
         * @brief We can't copy these because otherwise we might close the device twice
         */
        uart(const uart& uart) = delete;
        uart(uart&&)           = default;

        /**
         * @brief Destructor, close the device on destruction
         */
        ~uart();

        /**
         * @brief Get the native file descriptor used by this class. Useful for using in poll or select.
         *
         * @return the native file descriptor
         */
        int native_handle();

        /**
         * @brief Return true if the connection is valid
         *
         * @return true if the uart is connected and working, false otherwise
         */
        bool connected() const;

        /**
         * @brief Read from the device into a buffer
         *
         * @param buf buffer to read into
         * @param count the number of bytes to read
         *
         * @return the number of bytes that were actually read, or -1 if fail. See ::read
         */
        ssize_t read(void* buf, size_t count);

        /**
         * @brief Write bytes to the uart
         *
         * @param buf the buffer to write bytes from
         * @param count the number of bytes to write
         *
         * @return the number of bytes that were written
         */
        ssize_t write(const void* buf, size_t count);

        /**
         * @brief Open the uart for the given file descriptor. Closes any currently open file.
         *
         * @param device the path to the device to open
         * @param int the baud rate to open with
         */
        void open(const std::string& device, const unsigned int& baud_rate = 57600);

        /**
         * @brief Close the open file descriptor
         */
        void close();
    };

}  // namespace io
}  // namespace utility

#endif  // UTILITY_IO_UART_HPP
