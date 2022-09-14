#ifndef MODULE_PLATFORM_NUGUS_HARDWAREIO_H
#define MODULE_PLATFORM_NUGUS_HARDWAREIO_H

#include <map>
#include <nuclear>

#include "NUgus.h"

#include "message/platform/nugus/Sensors.h"
#include "message/platform/nugus/StatusReturn.h"

#include "utility/io/uart.h"

namespace module {
namespace platform {
    namespace nugus {

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

                message::platform::nugus::Sensors::LEDPanel ledPanel = {false, false, false};
                //  0x00, 0xRR, 0xGG, 0xBB
                message::platform::nugus::Sensors::HeadLED headLED = {0x0000FF00};
                message::platform::nugus::Sensors::EyeLED eyeLED   = {0x000000FF};

                // Left, middle, right
                message::platform::nugus::Sensors::Buttons buttons = {false, false, false};

                // X, Y, Z
                message::platform::nugus::Sensors::Accelerometer acc = {0.0f, 0.0f, 0.0f};
                message::platform::nugus::Sensors::Gyroscope gyro    = {0.0f, 0.0f, 0.0f};

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

            void processModelInformation(const message::platform::nugus::StatusReturn& packet);
            void processOpenCRData(const message::platform::nugus::StatusReturn& packet);
            void processServoData(const message::platform::nugus::StatusReturn& packet);
        };

    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_HARDWAREIO_H
