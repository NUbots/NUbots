#ifndef UTILITY_REACTOR_STREAMREACTOR_HPP
#define UTILITY_REACTOR_STREAMREACTOR_HPP

#include <cctype>
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

            /**************************
             *   USER TRIGGER HOOKS   *
             **************************/
            on<ConnectTCP>().then("Initiate a TCP Connection", [this](const ConnectTCP& c) {
                // Setup the connection details
                remote = c;

                emit(std::make_unique<Reconnect>());
            });

            on<ConnectSerial>().then("Initiate a Serial Connection", [this](const ConnectSerial& c) {
                // Setup the connection details
                remote = c;

                emit(std::make_unique<Reconnect>());
            });

            on<Disconnect>().then("Disconnecting", [this] {
                // Allow pre-disconnection settings to be applied
                emit(std::make_unique<PreDisconnect>(false));
                emit(std::make_unique<Do<PreDisconnect>>(false));
            });

            on<Reconnect>().then("Reconnecting", [this](const Reconnect& rc) {
                // Log the reason we are reconnecting if it's not an empty string.
                // empty string means we are just starting up
                if (!rc.reason.empty()) {
                    log<NUClear::WARN>(rc.reason);
                }

                if (connection_valid()) {
                    // Initiate the reconnection
                    emit(std::make_unique<PreDisconnect>(true));
                    emit(std::make_unique<Do<PreDisconnect>>(true));
                }
                else {
                    // Initiate the connection
                    emit(std::make_unique<PreConnect>());
                    emit(std::make_unique<Do<PreConnect>>());
                }
            });

            on<TransmitData>().then("Transmit data to device", [this](const TransmitData& t) {
                // If there is not already a connection established, then calling reconnect is an error
                if (std::holds_alternative<std::monostate>(connection) || fd < 0) {
                    throw std::runtime_error("Can't transmit to no one.");
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
                if (log_level <= NUClear::TRACE) {
                    std::stringstream debug_string;
                    for (const auto& byte : t.data) {
                        debug_string << std::isprint(byte) ? std::string(byte) : fmt::format("\\{:02X} ", byte);
                    }
                    log<NUClear::TRACE>("Wrote:", debug_string.str());
                }
            });


            /**************************
             * INTERNAL TRIGGER CHAIN *
             **************************/
            on<Do<PreConnect>>().then([this] { emit(std::make_unique<DoConnect>()); });

            on<DoConnect>().then([this] {
                // Increase the connection attempt number to stop any existing configuration loops
                ++connection_attempt;

                try {
                    // Initiate the connection and get the file descriptor for the IO handle
                    if (std::holds_alternative<ConnectTCP>(remote)) {
                        const auto& r = std::get<ConnectTCP>(remote);
                        connection    = TCPConnection(r.host, r.port);

                        auto& tcp = std::get<TCPConnection>(connection);
                        tcp.fd    = utility::network::connect(tcp.host, tcp.port);
                        fd        = tcp.fd.get();
                    }
                    else if (std::holds_alternative<ConnectSerial>(remote)) {
                        const auto& r = std::get<ConnectSerial>(remote);
                        connection    = SerialConnection(r.device, r.baud_rate);

                        auto& serial = std::get<SerialConnection>(connection);
                        serial.uart  = utility::io::uart(serial.device, serial.baud_rate);
                        fd           = serial.uart.native_handle();
                    }
                    else {
                        remote     = std::monostate();
                        connection = std::monostate();
                        fd         = -1;
                        throw std::runtime_error("No valid connection settings found.");
                    }

                    // Something must have gone wrong and we are not connected
                    if (!connection_valid()) {
                        throw std::runtime_error("Failed to connect to device");
                    }

                    // Recreate the IO reaction
                    auto flags = IO::READ | IO::CLOSE | IO::ERROR;
                    io_handle  = on<IO>(fd, flags).then("Read Stream", [this](const IO::Event& event) {
                        if ((event.events & IO::READ) != 0) {
                            process(event.fd);
                        }
                        if ((event.events & IO::ERROR) != 0) {
                            // Unbind the handle, so it does not read stream in later iterations
                            io_handle.unbind();
                            // Start a new connection
                            emit(std::make_unique<Reconnect>("An invalid state occurred"));
                        }
                        if ((event.events & IO::CLOSE) != 0) {
                            // Unbind the handle, so it does not read stream in later iterations
                            io_handle.unbind();
                            // Start a new connection
                            emit(std::make_unique<Reconnect>("The device hung up"));
                        }
                        if ((event.events & ~(IO::READ | IO::ERROR | IO::CLOSE)) != 0) {
                            log<NUClear::ERROR>(
                                utility::strutil::dedent(fmt::format("Unknown IO event on {}valid connection: {}",
                                                                     connection_valid() ? "" : "in",
                                                                     event.events)));
                        }
                    });

                    // Allow post-connection settings to be applied
                    emit(std::make_unique<PostConnect>(connection_attempt));

                    log<NUClear::INFO>(fmt::format("Successfully connected to the device at {}", device_description()));
                }
                catch (const std::runtime_error& ex) {
                    log<NUClear::WARN>(fmt::format("Failed to connect to the device at {}\nError: {}",
                                                   device_description(),
                                                   ex.what()));
                }
            });

            on<Do<PreDisconnect>>().then([this](const Do<PreDisconnect>& pd) {
                // Allow post-disconnection settings to be applied
                emit(std::make_unique<DoDisconnect>(pd.reconnect));
            });

            on<DoDisconnect>().then("Disconnect from device", [this](const DoDisconnect& d) {
                // Unbind the reaction handle if it is bound
                if (io_handle) {
                    io_handle.unbind();
                }

                // Reset the connection part of the variant
                if (!std::holds_alternative<std::monostate>(connection)) {
                    connection = std::monostate();
                }

                // Clear old file descriptor
                fd = -1;

                // Allow post-disconnection settings to be applied
                emit(std::make_unique<PostDisconnect>(d.reconnect));
                emit(std::make_unique<Do<PostDisconnect>>(d.reconnect));
            });

            on<Do<PostDisconnect>>().then([this](const Do<PostDisconnect>& pd) {
                // If a reconnect has been initiated then kick off the connection attempt now
                if (pd.reconnect) {
                    emit(std::make_unique<PreConnect>());
                    emit(std::make_unique<Do<PreConnect>>());
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
        }

    protected:
        /**
         * @brief Common inheritance for all StreamReactor steps
         */
        template <typename U>
        struct NUClearArgs
            : public Trigger<U>
            , Sync<T> {};

        /**
         * A message struct that is emitted immediately before a connection is established
         */
        struct PreConnect : public NUClearArgs<PreConnect> {};

        /**
         * A message struct that is emitted immediately after a connection is established
         */
        struct PostConnect : public NUClearArgs<PostConnect> {
            PostConnect(const int& attempt) : attempt(attempt) {}
            /// The attempt number of the connection
            int attempt;
        };

        /**
         * A message struct that is emitted immediately before a connection is torn down
         */
        struct PreDisconnect : public NUClearArgs<PreDisconnect> {
            explicit PreDisconnect(const bool& reconnect) : reconnect(reconnect) {}

            /// Set to true if a new connection should be established after disconnection is complete
            bool reconnect{false};
        };

        /**
         * A message struct that is emitted immediately after a connection is torn down
         */
        struct PostDisconnect : public NUClearArgs<PostDisconnect> {
            explicit PostDisconnect(const bool& reconnect) : reconnect(reconnect) {}

            /// Set to true if a new connection should be established after disconnection is complete
            bool reconnect{false};
        };

        /**
         * A message to tell the system to establish a connection to a TCP server
         */
        struct ConnectTCP : public NUClearArgs<ConnectTCP> {
            ConnectTCP(std::string host, const uint16_t& port) : host(std::move(host)), port(port) {}
            std::string host{};
            uint16_t port{0};
        };

        /**
         * A message to tell the system to establish a connection to a serial device
         */
        struct ConnectSerial : public NUClearArgs<ConnectSerial> {
            ConnectSerial(std::string device, const int& baud_rate) : device(std::move(device)), baud_rate(baud_rate) {}
            std::string device{};
            int baud_rate{0};
        };

        /**
         * A message to tell the system to disconnect from a device
         */
        struct Disconnect : public NUClearArgs<Disconnect> {};

        /**
         * A message struct to tell the system to reconnect to the device
         */
        struct Reconnect : public NUClearArgs<Reconnect> {
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
        struct TransmitData : public NUClearArgs<TransmitData> {
            explicit TransmitData(const std::vector<uint8_t>& source) : data(source) {}
            explicit TransmitData(std::vector<uint8_t>&& source) noexcept : data(std::move(source)) {}
            template <typename SourceType>
            explicit TransmitData(const SourceType& source)
                : data(NUClear::util::serialise::Serialise<SourceType>::serialise(source)) {}

            TransmitData& operator=(const std::vector<uint8_t>& rhs) {
                data = rhs;
                return *this;
            }
            TransmitData& operator=(std::vector<uint8_t>&& rhs) noexcept {
                data = std::move(rhs);
                return *this;
            }

            /// The data to be transferred
            std::vector<uint8_t> data{};
        };

        /**
         * @brief Returns true if there is a currently established connection, false otherwise
         */
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
         * @brief Internal message used to perform connection logic
         */
        struct DoConnect : public NUClearArgs<DoConnect> {};

        /**
         * @brief Internal message used to perform disconnection logic
         */
        struct DoDisconnect : public NUClearArgs<DoDisconnect> {
            explicit DoDisconnect(const bool& reconnect) : reconnect(reconnect) {}

            /// Set to true if a new connection should be established after disconnection is complete
            bool reconnect{false};
        };

        /**
         * @brief Aliasing type for sanity and brevity purposes
         */
        template <typename Step>
        struct Do
            : public Step
            , NUClearArgs<Do<Step>> {
            // Inherit constructors from Step
            using Step::Step;

            // Disambiguate inherited NUClear fusion functions
            using NUClearArgs<Do<Step>>::bind;   // For the triggering
            using NUClearArgs<Do<Step>>::get;    // For the caching
            using NUClearArgs<Do<Step>>::group;  // For the syncing
        };

        /**
         * Holds the information about a serial connection
         */
        struct SerialConnection {
            explicit SerialConnection(std::string device, const int& baud_rate)
                : device(std::move(device)), baud_rate(baud_rate) {}
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
            explicit TCPConnection(std::string host, const uint16_t& port) : host(std::move(host)), port(port) {}
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
        [[nodiscard]] std::string device_description() {
            if (std::holds_alternative<ConnectSerial>(remote)) {
                auto& serial = std::get<ConnectSerial>(remote);
                return fmt::format("Serial Device: {} @ {}baud", serial.device, serial.baud_rate);
            }

            if (std::holds_alternative<ConnectTCP>(remote)) {
                auto& tcp = std::get<ConnectTCP>(remote);
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
            std::stringstream debug_string;
            // Read bytes one at a time
            uint8_t byte = 0;
            while (::read(fd, &byte, 1) == 1) {

                if (log_level <= NUClear::TRACE) {
                    debug_string << std::isprint(byte) ? std::string(byte) : fmt::format("\\{:02X} ", byte);
                }

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
            if (log_level <= NUClear::TRACE) {
                log<NUClear::TRACE>("Read:", debug_string.str());
            }
        }

        /// The parser for this reactor
        Parser parser;
        /// Holds the information about the connection for the stream reactor
        std::variant<std::monostate, SerialConnection, TCPConnection> connection{};
        /// Holds details about the remote device
        std::variant<std::monostate, ConnectSerial, ConnectTCP> remote{};
        /// The handle attached to the IO reaction
        ReactionHandle io_handle{};
        /// The file descriptor for the currently active connection
        int fd = -1;
    };

}  // namespace utility::reactor

#endif  // UTILITY_REACTOR_STREAMREACTOR_HPP
