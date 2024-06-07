/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

#ifndef MODULE_PLATFORM_WEBOTS_HPP
#define MODULE_PLATFORM_WEBOTS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <nuclear>
#include <string>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>
#include <vector>

#include "message/input/Image.hpp"
#include "message/platform/webots/messages.hpp"

namespace module::platform {

    class Webots : public NUClear::Reactor {
    private:
        /// @brief How often we read the servos
        static constexpr int UPDATE_FREQUENCY = 500;

        /// @brief Handle for incoming data reaction. This will be bound/unbound during (re)connection
        ReactionHandle read_io;
        /// @brief Handle for outgoing data reaction. This will be bound/unbound during (re)connection
        ReactionHandle send_io;
        /// @brief Handle for error checking on the TCP connection. This will be bound/unbound during (re)connection
        ReactionHandle error_io;

        /// @brief server_name The name or IP address to connect to. If it's an IP, it should be in "X.X.X.X" form
        std::string server_address;
        /// @brief server_port The port number to connect to on the server
        std::string server_port;

        /// @brief Establish a TCP connection to the specified server/port
        /// @return If the connection was successful, a file descriptor. Else, -1 is returned
        [[nodiscard]] int tcpip_connect();

        /// @brief Establishes the connection with webots, then binds the reaction handles with the resulting fd
        void setup_connection();

        /// @brief Translate sensor measurement messages Webots sends us, emmitting readings as our message types
        /// @param sensor_measurements Message from Webots with information from the sensors
        void translate_and_emit_sensor(const message::platform::webots::SensorMeasurements& sensor_measurements);

        /// @brief The current file descriptor used for the connection. It should be kept -1 if no active connection
        int fd = -1;

        /// @brief The time the connection was opened.
        NUClear::clock::time_point connect_time{};

        /// @brief The number of time ticks which have passed since the last IO::READ trigger
        uint32_t sim_delta = 0;
        /// @brief The number of milliseconds which have passed since the last IO::READ trigger
        uint64_t real_delta = 0;
        /// @brief The current simulation time in ticks
        uint32_t current_sim_time = 0;
        /// @brief The current real time in milliseconds (unix time)
        uint64_t current_real_time = 0;
        /// @brief Interpolation factor to smooth clock. 0.0 is no smoothing (raw updates from Webots), 1.0 takes no
        /// updates from Webots
        double clock_smoothing = 0.0;
        /// @brief Real time factor of the simulation clock
        double rtf = 1.0;

        /// @brief The time between two measurements, expressed in milliseconds
        int time_step;
        /// @brief The minimum allowed time between two camera measurements
        int min_camera_time_step;
        /// @brief The minimum allowed time between two sensor measurements, not including the camera
        int min_sensor_time_step;
        /// @brief The maximum velocity allowed by the NUgus motors in webots
        double max_velocity_mx64;
        double max_velocity_mx106;

        /// @brief Current state of a servo
        struct ServoState {
            /// @brief True if we need to write new values to the simulator
            bool dirty = false;

            /// @brief ID of the servo
            int id;

            /// @brief Name of the servo
            std::string name;

            double p_gain = 32.0 / 255.0;
            // `i` and `d` gains are always 0
            static constexpr double i_gain = 0.0;
            static constexpr double d_gain = 0.0;

            double moving_speed   = 0.0;
            double goal_position  = 0.0;
            double goal_torque    = 0.0;
            double torque_enabled = 0.0;  // 0.0 to 1.0

            /// Values that are read from the simulator
            double present_position = 0.0;
            double present_speed    = 0.0;
        };

        /// @brief Our current servo states
        std::array<ServoState, 20> servo_state{};

        /// @brief Buffer for storing received messages
        std::vector<uint8_t> buffer{};

        /// @brief Atomic variable indicating that a reconnect is currently in progress
        std::atomic_bool active_reconnect{false};
        bool connection_active = false;

        std::mutex sensors_mutex;
        std::vector<std::pair<NUClear::clock::time_point, Eigen::Isometry3d>> Hwps;

        struct CameraContext {
            std::string name;
            uint32_t id = 0;
            message::input::Image::Lens lens;
            // Homogenous transform from camera (c) to platform (p) where platform is the rigid body the camera is
            // attached to
            Eigen::Isometry3d Hpc;
        };
        std::map<std::string, CameraContext> camera_context;
        uint32_t num_cameras = 0;

        /// @brief When set, the next ActuatorRequests to webots will set the reset world command to end the simulation.
        bool reset_simulation_world = false;

        /// @brief When set, the next ActuatorRequests to webots will set the reset time command to end the simulation.
        bool reset_simulation_time = false;

        /// @brief When set, the next ActuatorRequests to webots will set the terminate command to end the simulation.
        bool terminate_simulation = false;

        /// @brief Max FSR sensor value
        float max_fsr_value = 0;

    public:
        /// @brief Called by the powerplant to build and setup the webots reactor
        explicit Webots(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_WEBOTS_HPP
