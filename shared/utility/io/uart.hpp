#ifndef UTILITY_IO_UART_HPP
#define UTILITY_IO_UART_HPP

#include <string>

namespace utility::io {

    /**
     * Class for managing a connection to a serial device
     */
    class uart {
    private:
        std::string device{};
        int fd = -1;

        /**
         * Set the baud rate of the device
         *
         * @param baud the baud rate to set
         */
        void set_baud(const unsigned int& baud);

    public:
        /**
         * Construct a new unconnected uart class
         */
        uart() = default;

        /**
         * Create a new uart class that is connected to the device `device`
         *
         * @param device the file path of the device to connect to
         * @param int the baud rate to connect to
         */
        uart(const std::string& device, const unsigned int& baud_rate = 57600);

        /**
         * Move construct a new uart class
         *
         * @param other the uart class to move from
         */
        uart(uart&& other) noexcept;

        /**
         * Move assign a new uart class
         *
         * @param rhs the uart class to move from
         */
        uart& operator=(uart&& rhs) noexcept;

        // We can't copy these because otherwise we might close the device twice
        uart(const uart& uart)            = delete;
        uart& operator=(const uart& uart) = delete;

        /**
         * Destructor, close the device on destruction
         */
        ~uart();

        /**
         * Get the native file descriptor used by this class. Useful for using in poll or select.
         *
         * @return the native file descriptor
         */
        [[nodiscard]] int native_handle() const;

        /**
         * Return true if the connection is valid
         *
         * @return true if the uart is connected and working, false otherwise
         */
        [[nodiscard]] bool connected() const;

        /**
         * Read from the device into a buffer
         *
         * @param buf buffer to read into
         * @param count the number of bytes to read
         *
         * @return the number of bytes that were actually read, or -1 if fail. See ::read
         */
        ssize_t read(void* buf, size_t count);

        /**
         * Write bytes to the uart
         *
         * @param buf the buffer to write bytes from
         * @param count the number of bytes to write
         *
         * @return the number of bytes that were written
         */
        ssize_t write(const void* buf, size_t count);

        /**
         * Read a single character from the uart device, returning -1 if there is no data
         *
         * @return the character read from the device, or -1 if there is no data
         */
        [[nodiscard]] int get();

        /**
         * Close the open file descriptor then reset fd = -1
         */
        void close();
    };

}  // namespace utility::io

#endif  // UTILITY_IO_UART_HPP
