#ifndef MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP
#define MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP

#include <Eigen/Core>
#include <map>
#include <nuclear>

#include "NUgus.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "message/platform/RawSensors.hpp"
#include "message/platform/ServoLED.hpp"
#include "message/platform/StatusReturn.hpp"

#include "utility/io/uart.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::platform::OpenCR {

    class HardwareIO : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the HardwareIO reactor.
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Manages the connection with OpenCR
        utility::io::uart opencr{};

        /// @brief How frequently the servos are read
        static constexpr int UPDATE_FREQUENCY = 90;

        /// @brief Contains device information specific to the NUgus robot
        NUgus nugus{};

        /// @brief How long we expect to wait per byte of data being sent
        uint32_t byte_wait = 0;

        /// @brief How long we expect to wait for a packet
        uint32_t packet_wait = 0;

        /// @brief Maps device IDs to expected packet data
        enum class PacketTypes : uint8_t { MODEL_INFORMATION, OPENCR_DATA, SERVO_DATA, FSR_DATA };

        /// @brief The packets we are currently waiting to receive
        std::map<NUgus::ID, std::vector<PacketTypes>> packet_queue;

        /// @see opencr_state
        struct OpenCRState {
            /// @brief Whether any of the variables in this struct have changed
            bool dirty = false;

            /// @brief State of the LED panel, whether the lights are on or off
            message::platform::RawSensors::LEDPanel led_panel = {false, false, false};

            /// @brief Colour of the head LED
            /// @details In the form of 0x00, 0xRR, 0xGG, 0xBB
            message::platform::RawSensors::HeadLED head_led = {0x0000FF00};

            /// @brief Colour of the eye LED
            /// @details In the form of 0x00, 0xRR, 0xGG, 0xBB
            message::platform::RawSensors::EyeLED eye_led = {0x000000FF};

            /// @brief Pushed state of the buttons - left, middle, right
            message::platform::RawSensors::Buttons buttons = {false, false, false};

            /// @brief Accelerometer value of the OpenCR IMU
            Eigen::Vector3f acc = Eigen::Vector3f::Zero();

            /// @brief Gyroscope value of the OpenCR IMU
            Eigen::Vector3f gyro = Eigen::Vector3f::Zero();

            /// @brief Buzzer sound level of the OpenCR device
            uint16_t buzzer = 0;

            /// @brief Most recent packet error from the OpenCR device
            uint8_t error_flags = 0;
        };

        /// @see battery_state
        struct Battery {
            /// @brief The voltage of the battery at full charge
            float charged_voltage = 0.0f;
            /// @brief The manufacturer's nominal voltage of the battery
            float nominal_voltage = 0.0f;
            /// @brief The voltage of the battery when it is flat
            float flat_voltage = 0.0f;
            /// @brief The current voltage of the battery
            float current_voltage = 0.0f;
            /// @brief The percentage of the battery charge that is remaining
            float percentage = 0.0f;
            /// @brief Whether any battery values have changed
            bool dirty = false;
        };

        /// @see servo_states
        struct ServoState {
            /// @brief True if we need to write new values to the hardware
            bool dirty = false;

            /// @brief Current error state of the servo
            /// @note different to the dynamixel packet error status
            uint8_t hardware_error = 0;

            /// @brief Most recent packet error recieved, to cache before RawSensors is emitted
            uint8_t packet_error = 0;

            /// @brief True if we simulate where we think the servos should be
            /// @note that we still write the commands to hardware
            bool simulated = false;

            /// @brief Our internal system torque target, this is never sent to the servo
            int torque = 0;
            /// @brief Whether the servo's torque is enabled, allowing it to move
            bool torque_enabled = true;

            // Cached values that are never read and are sent to the servos
            /// @brief The integral gain of the velocity
            float velocity_i_gain = 1920.0f / 65536.0f;  // ROBOTIS default
            /// @brief The proportional gain of the velocity
            float velocity_p_gain = 100.0f / 128.0f;  // ROBOTIS default
            /// @brief The derivative gain of the position
            float position_d_gain = 0.0f;
            /// @brief The integral gain of the position
            float position_i_gain = 0.0f;
            /// @brief The proportional gain of the position
            float position_p_gain = 850.0f / 128.0f;  // ROBOTIS default
            /// @brief The first feedforward torque coefficient
            float feedforward_1st_gain = 0.0f;
            /// @brief The second feedforward torque coefficient
            float feedforward_2nd_gain = 0.0f;
            /// @brief The target pulse width modulation of the servo
            float goal_pwm = 885.0f;  // ROBOTIS default
            /// @brief The target current of the servo
            float goal_current = 6.52176f / 0.00336f;  // ROBOTIS default
            /// @brief The target velocity of the servo, not used by the servos
            float goal_velocity = 1.08775f;  // ROBOTIS default
            /// @brief The target position of the servo
            float goal_position = 0.0f;
            /// @brief The target acceleration of the servo
            float profile_acceleration = 0.0f;
            /// @brief The target velocity of the servo, replacing moving speed in v1 protocol
            float profile_velocity = 0.0f;

            // Values that are either simulated or read from the servos
            /// @brief The last read pulse width modulation of the servo
            float present_pwm = 0.0f;
            /// @brief The last read current of the servo
            float present_current = 0.0f;
            /// @brief The last read velocity of the servo
            float present_velocity = 0.0f;
            /// @brief The last read position of the servo
            float present_position = 0.0f;
            /// @brief The last read voltage of the servo
            float voltage = 0.0f;
            /// @brief The last read temperature of the servo
            float temperature = 0.0f;

            /// @brief Whether we have initialised this servo yet
            bool initialised = false;
        };

        /// @brief Contains the current state of the OpenCR device
        /// The state is both read from the device (eg reading IMU and buttons) and set (eg setting LEDs)
        OpenCRState opencr_state{};

        /// @brief The state of the servos, used to store read values and to store values to be written
        std::array<ServoState, 20> servo_states{};

        /// @brief The state of the battery
        Battery battery_state{};

        /// @brief Reads information from an OpenCR packet and logs the model and firmware version
        /// @param packet a preprocessed OpenCR packet
        void process_model_information(const message::platform::StatusReturn& packet);

        /// @brief Reads information from an OpenCR packet and populates opencr_state and battery_state
        /// @param packet a preprocessed OpenCR packet
        void process_opencr_data(const message::platform::StatusReturn& packet);

        /// @brief Reads information from an OpenCR packet and populates servo_states
        /// @param packet a preprocessed OpenCR packet
        /// @note Although we do a Sync Write to all servos, data is returned one by one
        void process_servo_data(const message::platform::StatusReturn& packet);

        /// @brief Reads info from the state variables and processes it into a RawSensors message
        /// @return A RawSensors message created from the current state variables
        message::platform::RawSensors construct_sensors();

        /// @brief Runs the setup for the devices
        void startup();

        /// @brief handles a response packet from the device
        void handle_response();

        /// @brief handle sending a request to the OpenCR device
        void send_opencr_request();

        /// @brief handle sending a request to the servo devices
        void send_servo_request();

        /// @brief Check if we're currently waiting on any servo packets
        /// @returns ID of FIRST servo we're waiting on, or 0 if none
        NUgus::ID servo_waiting();

        /// @brief Check if we're currently waiting on any OpenCR packets
        /// @returns True if waiting on packets, false if not
        bool opencr_waiting();

        /// @brief Check if we're currently waiting on any packets
        /// @returns ID of FIRST device we're waiting on, or 0 if none
        NUgus::ID queue_item_waiting();

        /// @brief clear all packet queues
        /// @returns the number of packets cleared
        int queue_clear_all();

        struct PacketWatchdog {};
        struct ModelWatchdog {};

        /// @brief Handle for the watchdog timer for the model information
        ReactionHandle model_watchdog;
        /// @brief Handle for our watchdog timer for packet handling
        ReactionHandle packet_watchdog;
    };

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFORM_OPENCR_HARDWAREIO_H
