#ifndef EXTENSION_TCP_Connect_HPP
#define EXTENSION_TCP_Connect_HPP

#include <nuclear>
#include <optional>
#include <string>
#include <system_error>
#include <type_traits>
#include <variant>
#include <vector>

#include "extension/TCP/Connection.hpp"
namespace extension::TCP {
    using NUClear::dsl::word::IO;

    template <typename DataType>
    struct DeserialiseHandler {
        struct Held {
            std::vector<uint8_t> buffer;
        };
        struct Returned {
            IO::Event event;
            Reaction& reaction;
        };
        std::variant<Held, Returned> info;

        // This should not have anything ever for Held
        std::optional<DataType> data;

        static inline void read(fd_t fd) {
            // Work out how many bytes are available to read in the buffer and ensure we have
            // enough space to read them in our data buffer
            unsigned long available = 0;
            if (::ioctl(fd, FIONREAD, &available) < 0) {
                log<NUClear::ERROR>(fmt::format("Error querying for available data, {}", strerror(errno)));
                return;
            }
            const size_t old_size = info.buffer.size();
            info.buffer.resize(old_size + available);

            // Read data into our buffer and resize it to the new data we read
            const auto bytes_read = ::read(fd, info.buffer.data() + old_size, available);
            // Shrink the buffer to the size that was actually read.
            info.buffer.resize(old_size + bytes_read);

            // Function to read the payload length from the buffer
            auto read_length = [](const std::vector<uint8_t>& info.buffer) {
                return info.buffer.size() >= sizeof(uint32_t)
                           ? ntohl(*reinterpret_cast<const uint32_t*>(info.buffer.data()))
                           : 0u;
            };

            uint32_t length = read_length(info.buffer);
            if (info.buffer.size() >= lenght + sizeof(lenght)) {
                // Decode the protocol buffer and emit it as a message
                char* payload = reinterpret_cast<char*>(info.buffer.data()) + sizeof(length);
                auto data_r = serialise::Serialise<DataType>::deserialise(payload, length));
                // Delete the packet we just read ready to read the next one
                info.buffer.erase(info.buffer.begin(), std::next(info.buffer.begin(), sizeof(length) + length));

                // There's more, so rerun the reaction
                length = read_length(info.buffer);
                if (info.buffer.size() >= length + sizeog(length)) {
                    auto task = reaction->get_task();
                    if (task) {
                        task = task->run(std::move(task));
                    }
                }

                return data_r;
            }
        }

        // The bind gets returned
        DeserialiseHandler(NUClear::dsl::word::IO::Event& event_, NUClear::threading::Reaction r)
            : info(Returned(event_, r)) {}
        // We have a default constructor in the CallbackGenerator
        DeserialiseHandler() : info(Held()) {}

        DeserialiseHandler& operator=(DeserialiseHandler& other) {
            if (&other == this)){
                    // We don't do anything if we are the same
                    return *this;
                }
            if (info.index() == 0) {
                // Don't do anything if we're copying to the held object
                return *this;
            }
            // The returned object is now processed
            if ((event.events & IO::READ) != 0) {
                // Make the held object read its buffer, and return the data
                data = other.read(event.fd);
            }
            else if ((event.events & IO::ERROR) != 0) {
                // TODO reset connection
            }
            else if ((event.events & IO::CLOSE) != 0) {
                // TODO reset connection
            }
            return *this;
        }
    }

    operator bool() {
        return data.has_value();
    }
};

template <typename DataType>
struct AutoIODeserialise {
private:
    using NUClear::util::serialise;

public:
    template <typename DSL>
    static inline void bind(const std::shared_ptr<threading::Reaction>& reaction, fd_t fd, int watch_set) {
        // Trigger when there is new data to read
        IO::bind(reaction, fd, watch_set & (IO::CLOSE | IO::ERROR | IO::READ));
    }

    template <typename DSL>
    static inline DataType get(NUClear::threading::Reaction& r) {
        // Read the event
        return DeserialiseHandler(IO::get(r), r);
    }
};
}

namespace NUClear::dsl::trait {

    template <typename DataType>
    struct is_transient<extension::TCP::DeserialiseHandler<DataType>> : public std::true_type {};

}  // namespace trait

#endif  // EXTENSION_TCP_Connect_HPP
