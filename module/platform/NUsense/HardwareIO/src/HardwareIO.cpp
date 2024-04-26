#include "HardwareIO.hpp"

#include <fmt/format.h>
#include <string>

#include "NUSenseParser.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/platform/NUSenseData.hpp"
#include "message/reflection.hpp"

namespace module::platform::NUsense {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::NUSense;

    /**
     * Message reflector class that can be used to emit messages provided as NUSenseFrames to the rest of the system
     *
     * @tparam T The type of the message to emit
     */
    template <typename T>
    struct EmitReflector;

    /// Virtual base class for the emit reflector
    template <>
    struct EmitReflector<void> {  // NOLINT(cppcoreguidelines-special-member-functions)
        virtual void emit(NUClear::PowerPlant& powerplant, const NUSenseFrame& frame) = 0;
        virtual ~EmitReflector()                                                      = default;
    };
    template <typename T>
    struct EmitReflector : public EmitReflector<void> {
        void emit(NUClear::PowerPlant& powerplant, const NUSenseFrame& frame) override {
            // Deserialise and emit
            powerplant.emit(std::make_unique<T>(NUClear::util::serialise::Serialise<T>::deserialise(frame.payload)));
        }
    };


    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : utility::reactor::StreamReactor<HardwareIO, NUSenseParser, 5>(std::move(environment)) {

        on<Configuration>("NUSense.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Get the port from the config file
            auto device = config["connection"]["path"].as<std::string>();
            auto baud   = config["connection"]["baud"].as<int>();

            // Tell the stream reactor to connect to the device
            emit(std::make_unique<ConnectSerial>(device, baud));
        });

        on<PostConnect>().then("Apply Configuration", [this](const PostConnect& apply) {
            // Apply any configuration to the device in this lambda
            std::string status = apply.attempt == 1 ? "successful" : "unsucessful";
            log<NUClear::INFO>(fmt::format("StreamReactor PostConnect status: {}", status));
        });

        // Send servo targets to the device
        on<Trigger<ServoTargets>>().then("ServoTargets", [this](const ServoTargets& packet) { send_packet(packet); });

        // Emit any messages sent by the device to the rest of the system
        on<Trigger<NUSenseFrame>>().then("From NUSense", [this](const NUSenseFrame& packet) {
            message::reflection::from_hash<EmitReflector>(packet.hash)->emit(powerplant, packet);
        });

        // TODO (JohanneMontano) Figure out why we only receive 17 servo targets on NUSense with frequencies above 50 Hz
        on<Every<50, Per<std::chrono::seconds>>>().then([this] {
            auto servo_targets = std::make_unique<ServoTargets>();

            for (int i = 0; i < 20; ++i) {
                auto servo_target      = std::make_unique<ServoTarget>();
                servo_target->time     = NUClear::clock::now();
                servo_target->id       = i + 1;
                servo_target->position = 100.0;
                servo_target->gain     = 100.0;
                servo_target->torque   = 100.0;

                servo_targets->targets.push_back(*servo_target);
            }

            emit(servo_targets);
        });

        on<Trigger<NUSense>>().then([this](const NUSense& nusense) {
            log<NUClear::DEBUG>(
                fmt::format("\nIMU Data\n"
                            "\tAccel(xyz): {} - {} - {}\n"
                            "\t Gyro(xyz): {} - {} - {}\n ",
                            nusense.imu.accel.x,
                            nusense.imu.accel.y,
                            nusense.imu.accel.z,
                            nusense.imu.gyro.x,
                            nusense.imu.gyro.y,
                            nusense.imu.gyro.z));

            log<NUClear::DEBUG>("Logging servo states...");

            for (const auto& [key, val] : nusense.servo_map) {
                log<NUClear::DEBUG>(fmt::format("      key: {}", key));

                log<NUClear::DEBUG>(fmt::format("       id: {}", val.id));
                log<NUClear::DEBUG>(fmt::format("   hw_err: {}", val.hardware_error));
                log<NUClear::DEBUG>(fmt::format("torque_en: {}", val.torque_enabled));
                log<NUClear::DEBUG>(fmt::format("     ppwm: {}", val.present_pwm));
                log<NUClear::DEBUG>(fmt::format("    pcurr: {}", val.present_current));
                log<NUClear::DEBUG>(fmt::format("    pvelo: {}", val.present_velocity));
                log<NUClear::DEBUG>(fmt::format("     ppos: {}", val.present_position));
                log<NUClear::DEBUG>(fmt::format("     gpwm: {}", val.goal_pwm));
                log<NUClear::DEBUG>(fmt::format("    gcurr: {}", val.goal_current));
                log<NUClear::DEBUG>(fmt::format("    gvelo: {}", val.goal_velocity));
                log<NUClear::DEBUG>(fmt::format("     gpos: {}", val.goal_position));
                log<NUClear::DEBUG>(fmt::format("  voltage: {}", val.voltage));
                log<NUClear::DEBUG>(fmt::format("     temp: {}", val.temperature));
            }
        });
    }

}  // namespace module::platform::NUsense
