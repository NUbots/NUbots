#include "NUSense.hpp"

#include "extension/Configuration.hpp"

#include "message/reflection.hpp"


namespace module::platform::NUsense {

    /**
     * Message reflector class that can be used to emit messages provided as NBSFrames to the rest of the system
     *
     * @tparam T The type of the message to emit
     */
    template <typename T>
    struct EmitReflector;

    /// Virtual base class for the emit reflector
    template <>
    struct EmitReflector<void> {  // NOLINT(cppcoreguidelines-special-member-functions)
        virtual uint64_t emit(const NBSFrame& frame) = 0;
        virtual ~EmitReflector()                     = default;
    };
    template <typename T>
    struct EmitReflector : public EmitReflector<void> {
        template <typename U = T>
        void emit(NUClear::PowerPlant& powerplant, const NBSFrame& frame) {
            // Deserialise and emit
            powerplant.emit(std::make_unique<T>(NUClear::util::serialise::Serialise<T>::deserialise(frame.payload)));
        }
    };


    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : utility::reactor::StreamReactor<HardwareIO, NBSParser, 5>(std::move(environment)) {

        on<Configuration>("NUSense.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Get the IP address and port from the config file
            auto device = config["connection"]["device"].as<std::string>();
            auto baud   = config["connection"]["baud"].as<int>();

            // Tell the stream reactor to connect to the device
            emit(std::make_unique<ConnectSerial>(device, baud));
        });

        on<PostConnect>().then("Apply Configuration", [this](const PostConnect& apply) {
            // TODO: Apply any configuration to the device in this lambda
        });

        // Send servo targets to the device
        on<Trigger<ServoTargets>>().then("ServoTargets", [this](const ServoTargets& packet) { send_packet(packet); });

        // Emit any messages sent by the device to the rest of the system
        on<Trigger<NUSenseFrame>>().then("From NUSense", [this](const NUSenseFrame& packet) {
            message::reflection::from_hash<IdReflector>(packet.hash)->emit(powerplant, packet);
        });
    }

}  // namespace module::platform::NUsense
