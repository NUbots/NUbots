#ifndef EXTENSION_TCP_Connect_HPP
#define EXTENSION_TCP_Connect_HPP

#include <nuclear>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

#include "extension/TCP/Connection.hpp"
namespace  extension::TCP {
    template <typename>
    struct Connect;

    template <>
    struct Connect {
        template<typename DSL>
        static inline Connection
            bind(cosnt std::shared_ptr<NUClear::threading::Reaction>& reaction,
                 const std::string server_address, const std::string server_port){
            // Hints for the connection type
            addrinfo hints{};
            memset(&hints, 0, sizeof(addrinfo));  // Defaults on what we do not explicitly set
            hints.ai_family   = AF_UNSPEC;        // IPv4 or IPv6
            hints.ai_socktype = SOCK_STREAM;      // TCP

            // Store the ip address information that we will connect to
            addrinfo* address = nullptr;

            const int error = getaddrinfo(server_address.c_str(), server_port.c_str(), &hints, &address);
            if (error != 0) {
                throw std::system_error(network_errno, std::system_category(),
                                        fmt::format("Cannot resolve server name: {}. Error {}. Error code {}",
                                                server_address,
                                                gai_strerror(error),
                                                error));
            }

            // Loop through the linked list of potential options for connecting. In order of best to worst.
            for (addrinfo* addr_ptr = address; addr_ptr != nullptr; addr_ptr = addr_ptr->ai_next) {
                const int fd = socket(addr_ptr->ai_family, addr_ptr->ai_socktype, addr_ptr->ai_protocol);
                if (fd == -1) {
                    // Bad fd
                    continue;
                }
                if (connect(fd, addr_ptr->ai_addr, addr_ptr->ai_addrlen) != -1) {
                    // Connection successful
                    // Gather required information before we free the address information
                    in6_addr connected_addr;
                    in_port_t connected_port;
                    switch (addr_ptr->ai_addr->sa_family) {
                        case AF_INET:
                            // TODO check that setting the first 16 bits is correct, rather than the last 16.
                            connected_addr = std::reinterpret_cast<*sockaddr_in>(addr_ptr->ai_addr)->sin_addr
                            connected_port = ntohs(std::reinterpret_cast<*sockaddr_in>(addr_ptr->ai_addr)->sin_port);
                            break;
                        case AF_INET6:
                            connected_addr = std::reinterpret_cast<*sockaddr_in6>(addr_ptr->ai_addr)->sin6_addr;
                            connected_port = ntohs(std::reinterpret_cast<*sockaddr_in6>(addr_ptr->ai_addr)->sin6_port);
                            break;
                        default:
                            throw std::system_error(network_errno, std::system_category(), "Socket Address faimily was not AF_INET or AF_INET6");
                    }
                    freeaddrinfo(address);
                    // Generate a reaction for the IO system that closes on death
                    reaction->unbinders.push_back([](const threading::Reaction& r) {
                        r.reactor.emit<emit::Direct>(std::make_unique<operation::Unbind<IO>>(r.id));
                    });
                    reaction->unbinders.push_back([fd](const threading::Reaction&) {
                        shutdown(fd, SHUT_RDWR);
                        close(fd);
                    });

                    auto io_config = std::make_unique<IOConfiguration>(IOConfiguration{fd, IO::READ | IO::CLOSE | IO::ERROR, reaction});

                    // Send our configuration out
                    reaction->reactor.emit<emit::Direct>(io_config);
                    return std::make_tuple(connected_addr, connected_port, fd);
                    }
                    // Connection was not successful
                    close(fd);
                }

            // No connection was successful
            freeaddrinfo(address);
            throw std::system_error(network_errno, std::system_category(), fmt::format("Cannot connect to server: {}:{}", server_address, server_port));
        }

        template <typename DSL>
        static inline IO::Event get(NUClear::threading::Reaction& r) {
            auto event = IO::get<DSL>(r);
            if ((event.events & IO::READ) != 0) {
                return event;
            } else if ((event.events & IO::ERROR) != 0) {
                // TODO reset connection
                return event;
            } else if ((event.events & IO::CLOSE) != 0) {
                // TODO reset connection
                return event;
            }
        }
    };
    
    template <typename DataType>
    struct Connect {
    private:
        using NUClear::util::serialise;

        static bool more_data;
        static std::vector<uint8_t> buffer;

        static inline void read_into_buffer(IO::Event& event){
            auto fd = event.fd;
            // Work out how many bytes are available to read in the buffer and ensure we have
            // enough space to read them in our data buffer
            unsigned long available = 0;
            if (::ioctl(fd, FIONREAD, &available) < 0) {
                log<NUClear::ERROR>(
                    fmt::format("Error querying for available data, {}", strerror(errno)));
                return;
            }
            const size_t old_size = buffer.size();
            buffer.resize(old_size + available);

            // Read data into our buffer and resize it to the new data we read
            const auto bytes_read = ::read(fd, buffer.data() + old_size, available);
            // Shrink the buffer to the size that was actually read.
            buffer.resize(old_size + bytes_read);
        }

        static inline DataType serialise_one(NUClear::threading::Reaction& r){
            // Function to read the payload length from the buffer
            auto read_length = [](const std::vector<uint8_t>& buffer) {
                return buffer.size() >= sizeof(uint32_t)
                           ? ntohl(*reinterpret_cast<const uint32_t*>(buffer.data()))
                           : 0u;
            };
            
            DataType data;
            uint32_t length = read_length(buffer);
            if (buffer.size() >= lenght + sizeof(lenght)) {
                // Decode the protocol buffer and emit it as a message
                char* payload = reinterpret_cast<char*>(buffer.data()) + sizeof(length);
                auto data = serialise::Serialise<DataType>::deserialise(payload, length));
                // Delete the packet we just read ready to read the next one
                buffer.erase(buffer.begin(), std::next(buffer.begin(), sizeof(length) + length));
            }

            length = read_length(buffer);
            if(buffer.size() >= length + sizeog(length)) {
                more_data = true;
                auto task = r.get_task();
                if (task) {
                    reactor.powerplant.submit(std::move(task));
                }
            } else {
                more_data = false;
            }
            
            return data;
        }

    public:
        template<typename DSL>
        static inline Connection
            bind(cosnt std::shared_ptr<NUClear::threading::Reaction>& reaction,
                 const std::string server_address, const std::string server_port){
            auto connection = Connect<>::bind(reaction, server_address, server_port);
            BufferStore.set(std::make_shared<std::vector<uint8_t>>());
            MoreDataStore::value = nullptr;
            return connection;
        }

        template <typename DSL>
        static inline DataType get(NUClear::threading::Reaction& r) {
            if(more_data){
                serialise_one();
            } else {
                auto event = IO::get<DSL>(r);
                if ((event.events & IO::READ) != 0) {
                    read_into_buffer(event);
                    return serialise_one();
                } else if ((event.events & IO::ERROR) != 0) {
                    // TODO reset connection
                    return DataType();
                } else if ((event.events & IO::CLOSE) != 0) {
                    // TODO reset connection
                    return DataType();
                }
                // TODO throw
                return DataType();
            }
        }
    }

    template <typename DataType>
    bool Connect<DataType>::more_data = false;
    template <typename DataType>
    std::vector<uint8_t> Connect<DataType>::buffer;
}

#endif // EXTENSION_TCP_Connect_HPP
