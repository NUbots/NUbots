#ifndef EXTENSION_TCP_SEND_HPP
#define EXTENSION_TCP_SEND_HPP

#include <fmt/format.h>
#include <nuclear>
#include <system_error>

#include "utility/TCP/Connection.hpp"

namespace extension::TCP {
    using NUClear::fd_t;
    using NUClear::util::serialise::Serialise;
    using utility::TCP::Connection;
    /**
     * @brief
     *
     * @tparam DataType The type of data
     */
    template <typename DataType>
    struct Send {
    private:
        /**
         * @brief Sends data, with its size, to a socket
         *
         * @param data The data to send
         * @param size The size of the object that data points to
         * @param fd The connection to send to
         */
        static inline void send_it(void* data, uint32_t size, fd_t fd) {
            const uint32_t n_size = htonl(size);

            if (fd < 0) {
                throw std::system_error(network_errno,
                                        std::system_category(),
                                        fmt::format("The fd passed in was bad, {}", strerror(errno)));
            }
            if (::send(fd, &n_size, sizeof(size_t), 0) != sizeof(size_t)) {
                throw std::system_error(network_errno,
                                        std::system_category(),
                                        fmt::format("Error sending size, {}", strerror(errno)));
            }
            if (::send(fd, data, size, 0) != size) {
                throw std::system_error(network_errno,
                                        std::system_category(),
                                        fmt::format("Error sending data, {}", strerror(errno)));
            }
        }

    public:
        /**
         * @brief Sends a message, with its size, to a TCP connection
         *
         * @param pp Here for api
         * @param data The data to send
         * @param size The number of bytes of data
         * @param fd The TCP connection
         */
        static inline void emit(NUClear::PowerPlant&, std::shared_ptr<DataType> data, uint32_t size, fd_t fd) {
            send_it(data.get(), size, fd);
        }

        /**
         * @brief Serialise and send a message, with its size, to a TCP connection
         *
         * @param pp Here for api
         * @param data The data to serialise and send
         * @param fd The TCP connection
         */
        static inline void emit(NUClear::PowerPlant&, std::shared_ptr<DataType> data, fd_t fd) {
            auto s_data = Serialise<DataType>::serialise(*data);
            send_it(s_data.data(), s_data.size() * sizeof(DataType), fd);
        }

        /**
         * @brief Send a message, with its size, to a TCP connection
         *
         * @param pp Here for api
         * @param data The data to send
         * @param size The number of bytes of data
         * @param connection The TCP connection
         */
        static inline void emit(NUClear::PowerPlant& pp,
                                std::shared_ptr<DataType> data,
                                uint32_t size,
                                Connection connection) {
            emit(pp, data, size, connection.fd);
        }

        /**
         * @brief Serialise and send a message, with its size, to a TCP connection
         *
         * @param pp Here for api
         * @param data The data to serialise and send
         * @param connection The TCP connection
         */
        static inline void emit(NUClear::PowerPlant& pp, std::shared_ptr<DataType> data, Connection connection) {
            emit(pp, data, connection.fd);
        }
    };
}  // namespace extension::TCP

#endif  // EXTENSION_TCP_SEND_HPP
