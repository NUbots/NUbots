#ifndef EXTENSION_TCP_Connect_HPP
#define EXTENSION_TCP_Connect_HPP

#include <nuclear>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

// TODO maybe make this a utility function rather than a reactor :shrug:

#include "extension/TCP/Connection.hpp"
namespace  extension::TCP {
    template <typename DataType>
    struct AutoIODeserialise : public IO {
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
