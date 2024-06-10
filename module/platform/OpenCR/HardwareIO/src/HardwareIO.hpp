/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP
#define MODULE_PLATFORM_OPENCR_HARDWAREIO_HPP

#include <Eigen/Core>
#include <map>
#include <nuclear>

#include "NUgus.hpp"
#include "dynamixel/v2/Dynamixel.hpp"

#include "message/actuation/Servos.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/platform/ServoLED.hpp"
#include "message/platform/StatusReturn.hpp"

#include "utility/io/uart.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::platform::OpenCR {

    using message::actuation::Servo;

    using message::actuation::ServoID;

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
            uint8_t packet_error = 0;
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

        /// @brief Contains the current state of the OpenCR device
        /// The state is both read from the device (eg reading IMU and buttons) and set (eg setting LEDs)
        OpenCRState opencr_state{};

        /// @brief The state of the servos, used to store read values and to store values to be written
        std::map<ServoID, Servo> servos;

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

        /// @brief Stores configuration values
        struct Config {
            struct {
                struct {
                    /// @brief Container for the max tolerable temp for all servos
                    float level = 0.0;
                    /// @brief Container for the buzzer frequency, used if a Buzzer message is emitted
                    float buzzer_frequency = 0.0;
                } temperature;
            } alarms;
        } cfg;
    };

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFORM_OPENCR_HARDWAREIO_H
