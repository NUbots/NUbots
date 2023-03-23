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
        enum class PacketTypes : uint8_t { MODEL_INFORMATION, OPENCR_DATA, SERVO_DATA, FSR_DATA };
        std::map<uint8_t, std::vector<PacketTypes>> packet_queue;

        /// @see opencrState
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
            bool alertFlag      = 0;
            uint8_t errorNumber = 0;
        };

        /// @see batteryState
        struct Battery {
            float chargedVoltage = 0.0f;
            float nominalVoltage = 0.0f;
            float flatVoltage    = 0.0f;
            float currentVoltage = 0.0f;
            float percentage     = 0.0f;
            bool dirty           = false;
        };

        /// @see servoStates
        struct ServoState {
            /// @brief True if we need to write new values to the hardware
            bool dirty = false;

            /// @brief Current error state of the servo
            /// @warning different to the dynamixel packet error status
            uint8_t errorFlags = 0;

            /// @brief True if we simulate where we think the servos should be
            /// @note that we still write the commands to hardware
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
            /// @brief replaces "Moving speed" from v1 protocol, basically just the set speed.
            float profileVelocity = 0.0f;

            // Values that are either simulated or read
            float presentPWM      = 0.0f;
            float presentCurrent  = 0.0f;
            float presentVelocity = 0.0f;
            float presentPosition = 0.0f;
            float voltage         = 0.0f;
            float temperature     = 0.0f;
        };

        /// @see fsrStates
        // struct FSRState {
        //     uint16_t fsr1;
        //     uint16_t fsr2;
        //     uint16_t fsr3;
        //     uint16_t fsr4;
        //     uint8_t centreX;
        //     uint8_t centreY;
        // };

        /**
         * @brief Our state for our OpenCR for variables we send to it
         * Written to by processOpenCRData() and Read by constructSensors()
         */
        OpenCRState opencrState;

        /**
         * @brief Our state for our servos for variables we send to it
         * Written to by processServoData() and Read by constructSensors()
         */
        std::array<ServoState, 20> servoStates;

        /**
         * @brief Our state for our battery
         * Written to by processOpenCRData() and Read by constructSensors()
         */
        Battery batteryState;

        /**
         * @brief Our state for the 2 force sensitive resistors, stored as Right Left
         * Written to by processFSRData and Read by constructSensors()
         */
        // std::array<FSRState, 2> fsrStates;

        /**
         * @brief Reads information from an OpenCR packet and logs the model and firmware version.
         * @param packet a preprocessed OpenCR packet
         */
        void processModelInformation(const message::platform::StatusReturn& packet);

        /**
         * @brief Reads information from an OpenCR packet and populates opencrState and batteryState
         * @param packet a preprocessed OpenCR packet
         */
        void processOpenCRData(const message::platform::StatusReturn& packet);

        /**
         * @brief Reads information from an OpenCR packet and populates servoStates
         * @param packet a preprocessed OpenCR packet
         * @note Although we do a Sync Write to all servos, data is returned one by one
         */
        void processServoData(const message::platform::StatusReturn& packet);

        /**
         * @brief Reads information from an OpenCR packet and populares fsrStates
         * @param packet a preprocessed OpenCR packet
         * @note we sync write to both FSRs but data is returned one by one
         */
        // void processFSRData(const message::platform::StatusReturn& packet);

        /**
         * @brief Reads info from the state variables and processes it into a RawSensors message
         * @return A RawSensors message created from the current state variables
         */
        message::platform::RawSensors constructSensors();
    };

}  // namespace module::platform::openCR

#endif  // MODULE_PLATFORM_OPENCR_HARDWAREIO_H
