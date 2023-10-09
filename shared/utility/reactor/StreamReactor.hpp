#ifndef UTILITY_REACTOR_STREAMREACTOR_HPP
#define UTILITY_REACTOR_STREAMREACTOR_HPP

#include <fmt/format.h>
#include <memory>
#include <mutex>
#include <nuclear>
#include <variant>

#include "utility/file/FileDescriptor.hpp"
#include "utility/io/uart.hpp"
#include "utility/network/connect.hpp"
#include "utility/strutil/strutil.hpp"
#include "utility/support/si_unit.hpp"

namespace utility::reactor {

    /**
     * This reactor implements the common pattern of connecting to a stream device such as a serial port or a TCP
     * connection. It then will process bytes one at a time as they come in by calling the parser function provided
     * by the CRTP T type.
     *
     * @tparam T The CRTP type (the type that inherits from this class)
     * @tparam Parser The parser function that will be called to process the bytes as they come in
     *                The type is expected to have operator()(uint8_t) so either a functor or a lambda can be used
     * @tparam TimeoutTicks The number of units to wait before considering the device to be disconnected or
     *                      non-responsive
     * @tparam TimeoutPeriod The units that TimeoutTicks is measured in
     */
    template <typename T, typename Parser, int TimeoutTicks = 1, typename TimeoutPeriod = std::chrono::seconds>
    class StreamReactor : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the StreamReactor.
        explicit StreamReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<ConnectTCP>, Single, Sync<T>>().then("Initiate a TCP Connection", [this](const ConnectTCP& c) {
                // Disconnect if a connection is already established
                if (!std::holds_alternative<std::monostate>(connection)) {
                    emit<Scope::DIRECT>(std::make_unique<Disconnect>());
                }

                // Allow pre-connection settings to be applied
                emit<Scope::DIRECT>(std::make_unique<PreConnect>());

                try {
                    // Connect the tcp socket
                    connection = TCPConnection{c.host, c.port};
                    auto& tcp  = std::get<TCPConnection>(connection);
                    tcp.fd     = utility::network::connect(tcp.host, tcp.port);

                    // Recreate the IO reaction
                    auto flags = IO::READ | IO::CLOSE | IO::ERROR;
                    io_handle  = on<IO>(tcp.fd.get(), flags).then("Read Stream", [this](const IO::Event& event) {
                        if ((event.events & IO::READ) != 0) {
                            process(event.fd);
                        }
                        if ((event.events & IO::ERROR) != 0) {
                            emit(std::make_unique<Reconnect>("An invalid state occurred"));
                        }
                        if ((event.events & IO::CLOSE) != 0) {
                            emit(std::make_unique<Reconnect>("The device hung up"));
                        }
                        if ((event.events & ~(IO::READ | IO::ERROR | IO::CLOSE)) != 0) {
                            auto& tcp = std::get<TCPConnection>(connection);
                            log<NUClear::ERROR>(utility::strutil::dedent(fmt::format(R"(
                                Unknown IO event: {}
                                    FD Valid.......? {}
                                    Bytes Available: {}
                                )",
                                                                                     event.events,
                                                                                     tcp.fd.valid(),
                                                                                     tcp.fd.available())));
                        }
                    });
                    log<NUClear::INFO>(fmt::format("Successfully connected to the device at {}", device_description()));
                }
                catch (const std::runtime_error& ex) {
                    log<NUClear::WARN>(fmt::format("Failed to reconnect to the device at {}\nError: {}",
                                                   device_description(),
                                                   ex.what()));
                }

                // Allow post-connection settings to be applied
                emit(std::make_unique<PostConnect>(++connection_attempt));
            });

            on<Trigger<ConnectSerial>, Single, Sync<T>>().then(
                "Initiate a Serial Connection",
                [this](const ConnectSerial& c) {
                    // Disconnect if a connection is already established
                    if (!std::holds_alternative<std::monostate>(connection)) {
                        emit<Scope::DIRECT>(std::make_unique<Disconnect>());
                    }

                    // Allow pre-connection settings to be applied
                    emit<Scope::DIRECT>(std::make_unique<PreConnect>());

                    try {
                        // Connect the serial device
                        connection   = SerialConnection{c.device, c.baud_rate};
                        auto& serial = std::get<SerialConnection>(connection);
                        serial.uart  = utility::io::uart(serial.device, serial.baud_rate);

                        // Recreate the IO reaction
                        auto flags = IO::READ | IO::CLOSE | IO::ERROR;
                        io_handle  = on<IO>(serial.uart.native_handle(), flags)
                                        .then("Read Stream", [this](const IO::Event& event) {
                                            if ((event.events & IO::READ) != 0) {
                                                process(event.fd);
                                            }
                                            if ((event.events & IO::ERROR) != 0) {
                                                emit(std::make_unique<Reconnect>("An invalid state occurred"));
                                            }
                                            if ((event.events & IO::CLOSE) != 0) {
                                                emit(std::make_unique<Reconnect>("The device hung up"));
                                            }
                                            if ((event.events & ~(IO::READ | IO::ERROR | IO::CLOSE)) != 0) {
                                                auto& serial = std::get<SerialConnection>(connection);
                                                log<NUClear::ERROR>(
                                                    utility::strutil::dedent(fmt::format(R"(
                                                        Unknown IO event: {}
                                                            FD Valid.......? {}
                                                            Bytes Available: {}
                                                        )",
                                                                                         event.events,
                                                                                         serial.uart.connected(),
                                                                                         serial.uart.available())));
                                            }
                                        });
                        log<NUClear::INFO>(
                            fmt::format("Successfully connected to the device at {}", device_description()));
                    }
                    catch (const std::runtime_error& ex) {
                        log<NUClear::WARN>(fmt::format("Failed to reconnect to the device at {}\nError: {}",
                                                       device_description(),
                                                       ex.what()));
                    }

                    // Allow post-connection settings to be applied
                    emit(std::make_unique<PostConnect>(++connection_attempt));
                });

            on<Trigger<Disconnect>, Single, Sync<T>>().then("Teardown Connection", [this] {
                // Allow pre-disconnection settings to be applied
                emit<Scope::DIRECT>(std::make_unique<PreDisconnect>());

                // Unbind the reaction handle if it is bound
                if (io_handle) {
                    io_handle.unbind();
                }

                // Reset the connection part of the variant
                if (std::holds_alternative<SerialConnection>(connection)) {
                    auto& serial = std::get<SerialConnection>(connection);
                    serial.uart  = utility::io::uart{};
                }
                else if (std::holds_alternative<TCPConnection>(connection)) {
                    auto& tcp = std::get<TCPConnection>(connection);
                    tcp.fd    = utility::file::FileDescriptor{};
                }

                // Allow post-disconnection settings to be applied
                emit<Scope::DIRECT>(std::make_unique<PostDisconnect>());
            });

            on<Trigger<Reconnect>, Single, Sync<T>>().then("Reconnecting", [this](const Reconnect& rc) {
                // If there is not already a connection established, then calling reconnect is an error
                if (std::holds_alternative<std::monostate>(connection)) {
                    throw std::runtime_error("Can't reconnect if there is no established connection");
                }

                // Log the reason we are reconnecting if it's not an empty string.
                // empty string means we are just starting up
                if (!rc.reason.empty()) {
                    log<NUClear::WARN>(rc.reason);
                }

                if (std::holds_alternative<TCPConnection>(connection)) {
                    auto& tcp = std::get<TCPConnection>(connection);
                    emit(std::make_unique<ConnectTCP>(tcp.host, tcp.port));
                }
                else if (std::holds_alternative<SerialConnection>(connection)) {
                    auto& serial = std::get<SerialConnection>(connection);
                    emit(std::make_unique<ConnectSerial>(serial.device, serial.baud_rate));
                }
            });

            // Watchdog to ensure we are getting data from the device
            on<Watchdog<StreamReactor<T, Parser>, TimeoutTicks, TimeoutPeriod>, Single>().then([this] {
                auto s = double(TimeoutTicks) * double(TimeoutPeriod::period::num) / double(TimeoutPeriod::period::den);
                auto [si, unit] = utility::support::si_time(s);
                emit(std::make_unique<Reconnect>(fmt::format("No activity from the Device at {} in {}{}. Reconnecting",
                                                             device_description(),
                                                             si,
                                                             unit)));
            });

            on<Trigger<TransmitData>, Single, Sync<T>>().then("Transmit data to device", [this](const TransmitData& t) {
                // If there is not already a connection established, then calling reconnect is an error
                if (std::holds_alternative<std::monostate>(connection)) {
                    throw std::runtime_error("Can't transmit to no one");
                }

                // Find the file descriptor to write to
                int fd = -1;
                if (std::holds_alternative<TCPConnection>(connection)) {
                    auto& tcp = std::get<TCPConnection>(connection);
                    fd        = tcp.fd.get();
                }
                if (std::holds_alternative<SerialConnection>(connection)) {
                    auto& serial = std::get<SerialConnection>(connection);
                    fd           = serial.uart.native_handle();
                }

                // Write data to the device
                size_t bytes_written = 0;
                while (bytes_written < t.data.size()) {
                    ssize_t written = ::write(fd, t.data.data() + bytes_written, t.data.size() - bytes_written);
                    if (written < 0) {
                        throw std::system_error(errno,
                                                std::system_category(),
                                                fmt::format("Failed to transmit data to device at {} with error {}",
                                                            device_description(),
                                                            errno));
                    }
                    bytes_written += written;
                }
            });
        }

    protected:
        /**
         * A message struct that is emitted immediately before a connection is established
         */
        struct PreConnect {};

        /**
         * A message struct that is emitted immediately after a connection is established
         */
        struct PostConnect {
            PostConnect(const int& attempt) : attempt(attempt) {}
            /// The attempt number of the connection
            int attempt;
        };

        /**
         * A message struct that is emitted immediately before a connection is torn down
         */
        struct PreDisconnect {};

        /**
         * A message struct that is emitted immediately after a connection is torn down
         */
        struct PostDisconnect {};

        /**
         * A message to tell the system to establish a connection to a TCP server
         */
        struct ConnectTCP {
            ConnectTCP(std::string host, const uint16_t& port) : host(std::move(host)), port(port) {}
            std::string host{};
            uint16_t port{0};
        };

        /**
         * A message to tell the system to establish a connection to a serial device
         */
        struct ConnectSerial {
            ConnectSerial(std::string device, const int& baud_rate) : device(std::move(device)), baud_rate(baud_rate) {}
            std::string device{};
            int baud_rate{0};
        };

        /**
         * A message to tell the system to disconnect from a device
         */
        struct Disconnect {};

        /**
         * A message struct to tell the system to reconnect to the device
         */
        struct Reconnect {
            Reconnect() = default;

            /**
             * @brief Construct a new Reconnect object
             *
             * @param reason_ The reason the reconnection is happening when reconnecting or empty for the initial
             * connection
             */
            Reconnect(std::string reason_) : reason(std::move(reason_)) {}

            /// The reason the reconnection is happening when reconnecting or empty for the initial connection
            std::string reason;
        };

        /**
         * A message struct to tell the system to transmit data to the device
         */
        struct TransmitData {
            template <typename SourceType>
            explicit TransmitData(const SourceType& source)
                : data(NUClear::util::serialise::Serialise<SourceType>::serialise(source)) {}
            std::vector<uint8_t> data{};
        };

        [[nodiscard]] inline bool connection_valid() const {
            if (std::holds_alternative<SerialConnection>(connection)) {
                auto& serial = std::get<SerialConnection>(connection);
                return serial.uart.connected();
            }
            if (std::holds_alternative<TCPConnection>(connection)) {
                auto& tcp = std::get<TCPConnection>(connection);
                return tcp.fd.valid();
            }
            return false;
        }

        /// The current connection attempt to remove double ups in reactions
        int connection_attempt = 0;

    private:
        /**
         * Holds the information about a serial connection
         */
        struct SerialConnection {
            /// The path to the serial device
            std::string device{};
            /// The baud rate to connect at
            int baud_rate{};
            /// The uart object that is connected to the serial device
            utility::io::uart uart{};
        };

        /**
         * Holds the information about a TCP connection
         */
        struct TCPConnection {
            /// The host to connect to
            std::string host{};
            /// The port to connect to
            uint16_t port{};
            /// The file descriptor connected to the TCP socket
            utility::file::FileDescriptor fd{};
        };

        /**
         * Get the description of the device that we are connected to (network or serial)
         *
         * @return the description of the device
         */
        std::string device_description() {
            if (std::holds_alternative<SerialConnection>(connection)) {
                auto& serial = std::get<SerialConnection>(connection);
                return fmt::format("Serial Device: {} @ {}baud", serial.device, serial.baud_rate);
            }

            if (std::holds_alternative<TCPConnection>(connection)) {
                auto& tcp = std::get<TCPConnection>(connection);
                return fmt::format("{}:{}", tcp.host, tcp.port);
            }

            return "Unknown";
        }

        /**
         * @brief Process the data received from the device
         *
         * @details This function is called when the IO reaction is triggered. It reads the data from the device and
         * passes it to the parser. Once the parser finds a packet it emits that packet.
         */
        void process(const int& fd) {
            // Read bytes one at a time
            uint8_t byte = 0;
            while (::read(fd, &byte, 1) == 1) {
                try {
                    // Pass the byte to the parser and emit the packet if it is complete
                    auto packet = parser(byte);
                    if (packet) {
                        // Service the watchdog and emit the packet
                        emit<Scope::WATCHDOG>(ServiceWatchdog<StreamReactor<T, Parser>>());
                        emit(std::move(packet));
                    }
                }
                catch (const std::exception& ex) {
                    log<NUClear::WARN>(ex.what());
                }
            }
        }

        /// The parser for this reactor
        Parser parser;
        /// Holds the information about the connection for the stream reactor
        std::variant<std::monostate, SerialConnection, TCPConnection> connection{};
        /// The handle attached to the IO reaction
        ReactionHandle io_handle{};
    };

}  // namespace utility::reactor

#endif  // UTILITY_REACTOR_STREAMREACTOR_HPP
