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

            on<Trigger<Reconnect>, Single, Sync<Reconnect>>().then("Connecting", [this](const Reconnect& rc) {
                // Log the reason we are reconnecting if it's not an empty string.
                // empty string means we are just starting up
                if (!rc.reason.empty()) {
                    log<NUClear::WARN>(rc.reason);
                }
                reconnect();
            });

            on<Trigger<ConnectSerial>, Sync<Reconnect>>().then("Serial", [this](const ConnectSerial& cs) {
                connect_serial(cs.device, cs.baud_rate);
            });

            on<Trigger<ConnectTCP>, Sync<Reconnect>>().then("TCP", [this](const ConnectTCP& ct) {
                connect_tcp(ct.host, ct.port);
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
        }

    protected:
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
         * A message struct to tell the system to apply the configuration to the device
         */
        struct ApplyConfiguration {
            ApplyConfiguration(const int& attempt_) : attempt(attempt_) {}
            /// The attempt number of the connection
            int attempt;
        };

        struct ConnectSerial {
            ConnectSerial(const std::string& device_, const int& baud_rate_) : device(device_), baud_rate(baud_rate_) {}
            /// The path to the serial device to connect to
            std::string device;
            /// The baud rate to connect at
            int baud_rate;
        };

        struct ConnectTCP {
            ConnectTCP(const std::string& host_, const uint16_t& port_) : host(host_), port(port_) {}
            /// The host to connect to
            std::string host;
            /// The port to connect to
            uint16_t port;
        };

        /// The current file descriptor connected to by the StreamReactor
        int fd = -1;
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
         * Connect the StreamReactor to the given serial device
         *
         * @param device    the path to the serial device to connect to
         * @param baud_rate the baud rate to connect at
         */
        void connect_serial(const std::string& device, const int& baud_rate) {
            connection = SerialConnection{device, baud_rate};
            reconnect();
        }

        /**
         * Connect the StreamReactor to the given TCP host and port
         *
         * @param host the host to connect to
         * @param port the port to connect to
         */
        void connect_tcp(const std::string& host, const uint16_t& port) {
            connection = TCPConnection{host, port};
            reconnect();
        }

        /**
         * @brief Disconnect from the device
         *
         * @details Closes the connection to the device
         */
        void disconnect() {
            // Unbind the reaction handle if it is bound
            if (io_handle) {
                io_handle.unbind();
            }

            fd = -1;

            // Reset the connection part of the variant
            if (std::holds_alternative<SerialConnection>(connection)) {
                auto& serial = std::get<SerialConnection>(connection);
                serial.uart  = utility::io::uart{};
            }
            else if (std::holds_alternative<TCPConnection>(connection)) {
                auto& tcp = std::get<TCPConnection>(connection);
                tcp.fd    = utility::file::FileDescriptor{};
            }
        }

        /**
         *  Get the description of the device that we are connected to (network or serial)
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
         * @brief Reconnect to the device
         *
         * @details Closes the connection to the device and then re-opens the connection. The reaction to handle
         *   listening to new data from the newly opened file descriptor is also set up here
         */
        void reconnect() {
            try {
                // Disconnect from the Device
                disconnect();

                // Work out what kind of connection this is (serial device or tcp stream)
                if (std::holds_alternative<SerialConnection>(connection)) {
                    // Connect the serial device
                    auto& serial = std::get<SerialConnection>(connection);
                    serial.uart  = utility::io::uart(serial.device, serial.baud_rate);
                    fd           = serial.uart.native_handle();
                }
                else if (std::holds_alternative<TCPConnection>(connection)) {
                    // Connect the tcp socket
                    auto& tcp = std::get<TCPConnection>(connection);
                    tcp.fd    = utility::network::connect(tcp.host, tcp.port);
                    fd        = tcp.fd.get();
                }
                else {
                    throw std::runtime_error("Unknown connection type");
                }

                // Recreate the IO reaction
                auto flags = IO::READ | IO::CLOSE | IO::ERROR;
                io_handle  = on<IO>(fd, flags).then("Read Stream", [this](const IO::Event& event) {
                    if ((event.events & IO::READ) != 0) {
                        process(event.fd);
                    }
                    else if ((event.events & IO::ERROR) != 0) {
                        emit(std::make_unique<Reconnect>("An invalid state occurred"));
                    }
                    else if ((event.events & IO::CLOSE) != 0) {
                        emit(std::make_unique<Reconnect>("The device hung up"));
                    }
                });
                log<NUClear::INFO>(fmt::format("Successfully connected to the device at {}", device_description()));

                emit(std::make_unique<ApplyConfiguration>(++connection_attempt));
            }
            catch (const std::runtime_error& ex) {
                log<NUClear::WARN>(
                    fmt::format("Failed to reconnect to the device at {}\nError: {}", device_description(), ex.what()));
            }
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
