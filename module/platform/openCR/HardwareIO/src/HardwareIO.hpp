#ifndef MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP
#define MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP

#include <Eigen/Core>
#include <map>
#include <nuclear>

#include "NUgus.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/platform/StatusReturn.hpp"

#include "utility/io/uart.hpp"

namespace module::platform::openCR {

    class HardwareIO : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        // How often we read the servos
        static constexpr int UPDATE_FREQUENCY = 90;

        utility::io::uart opencr;
        NUgus nugus;

        uint32_t byte_wait;
        uint32_t packet_wait;

        // Maps device IDs to expected packet data
        enum class PacketTypes : uint8_t { MODEL_INFORMATION, OPENCR_DATA, SERVO_DATA };
        std::map<uint8_t, std::vector<PacketTypes>> packet_queue;

        struct OpenCRState {
            bool dirty = false;

            message::platform::RawSensors::LEDPanel ledPanel = {false, false, false};
            //  0x00, 0xRR, 0xGG, 0xBB
            message::platform::RawSensors::HeadLED headLED = {0x0000FF00};
            message::platform::RawSensors::EyeLED eyeLED   = {0x000000FF};

            // Left, middle, right
            message::platform::RawSensors::Buttons buttons = {false, false, false};

            // X, Y, Z
            Eigen::Vector3f acc  = Eigen::Vector3f::Zero();
            Eigen::Vector3f gyro = Eigen::Vector3f::Zero();

            // Buzzer
            uint16_t buzzer = 0;

            // Error status
            uint8_t errorFlags = 0;
        };

        struct Battery {
            float chargedVoltage = 0.0f;
            float nominalVoltage = 0.0f;
            float flatVoltage    = 0.0f;
            float currentVoltage = 0.0f;
            float percentage     = 0.0f;
            bool dirty           = false;
        };

        struct ServoState {
            // True if we need to write new values to the hardware
            bool dirty = false;

            // Current error state of the servo
            uint8_t errorFlags = 0;

            // True if we simulate where we think the servos should be
            // Note that we still write the commands to hardware
            bool simulated = false;

            bool torqueEnabled = true;

            // Cached values that are never read
            float velocityPGain       = 0.0f;
            float velocityIGain       = 0.0f;
            float velocityDGain       = 0.0f;
            float positionPGain       = 0.0f;
            float positionIGain       = 0.0f;
            float feedforward1stGain  = 0.0f;
            float feedforward2ndGain  = 0.0f;
            float goalPWM             = 0.0f;
            float goalCurrent         = 0.0f;
            float goalVelocity        = 0.0f;
            float goalPosition        = 0.0f;
            float profileAcceleration = 0.0f;
            float profileVelocity     = 0.0f;

            // Values that are either simulated or read
            float presentPWM      = 0.0f;
            float presentCurrent  = 0.0f;
            float presentVelocity = 0.0f;
            float presentPosition = 0.0f;
            float voltage         = 0.0f;
            float temperature     = 0.0f;
        };

        /// @brief Our state for our OpenCR for variables we send to it
        OpenCRState opencrState;

        /// @brief Our state for our servos for variables we send to it
        std::array<ServoState, 20> servoState;

        /// @brief Our state for our battery
        Battery batteryState;

        void processModelInformation(const message::platform::StatusReturn& packet);
        void processOpenCRData(const message::platform::StatusReturn& packet);
        void processServoData(const message::platform::StatusReturn& packet);
    };

}  // namespace module::platform::openCR

#endif  // MODULE_PLATFORM_NUGUS_HARDWAREIO_H
