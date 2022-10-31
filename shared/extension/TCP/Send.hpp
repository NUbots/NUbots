#ifndef EXTENSION_TCP_SEND_HPP
#define EXTENSION_TCP_SEND_HPP

#include <nuclear>
#include <system_error>

#include "utility/tcp/Connection.hpp"

namespace extension::TCP {
struct Send {
private:
    using NUClear::util::serialise;
    static inline void send_it(void* data, uint32_t size, fd_t fd){
        const uint32_t n_size = htonl(size);

        if(fd < 0){
            throw std::system_error(network_errno, std::system_category(), "The fd passed in was bad");
        }
        if(::send(fd, &n_size, sizeof(size_t), 0) != sizeof(size_t)){
            throw std::system_error(network_errno, std::system_category(), "Error sending size");
        }
        if(::send(fd, data, size, 0) != size){
            throw std::system_error(network_errno, std::system_category(), "Error sending data");
        }
    }
public:
    /**
     * @brief Send a message to a TCP connection
     *
     * @tparam DataType The type of data
     * @param data The data to send
     * @param size The number of bytes of data
     * @param fd The TCP connection
     */
    template <typename DataType>
    static inline void emit(NUClear::PowerPlant&, std::shared_ptr<DataType> data, uint32_t size, fd_t fd){
        send_it(data.get(), size, fd);
    }

    /**
     * @brief Serialise and send a message to a TCP connection
     *
     * @tparam DataType The type of data
     * @param data The data to serialise and send
     * @param fd  The TCP connection
     */
    template <typename DataType>
    static inline void emit(NUClear::PowerPlant& pp, std::shared_ptr<DataType> data, fd_t fd){
        auto s_data = serialise::Serialise::serialise(data.get());
        emit(pp, s_data.data(), s_data.size(), fd);
    }

    template <typename DataType>
    static inline void emit(NUClear::PowerPlant& pp, std::shared_ptr<DataType> data, uint32_t size, Connection connection){
        emit(pp, data, size, connection.fd);
    }
    template <typename DataType>
    static inline void emit(NUClear::PowerPlant& pp, std::shared_ptr<DataType> data, Connection connection){
        emit(pp, data, connection.fd);
    }
};
}

#endif // EXTENSION_TCP_SEND_HPP
