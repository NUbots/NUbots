/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include <fcntl.h>
#include <fmt/format.h>
#include <chrono>
#include <nuclear>

#include "PS3Walk.h"

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/JoystickEvent.h"
#include "message/motion/HeadCommand.h"
#include "message/motion/KickCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace behaviour {
    namespace strategy {

        using extension::Configuration;
        using extension::ExecuteScriptByName;

        using message::behaviour::MotionCommand;
        using message::input::JoystickEvent;
        using message::motion::HeadCommand;
        using message::motion::KickScriptCommand;

        using utility::behaviour::ActionPriorites;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform2D;

        PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), id(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("PS3Walk.yaml").then([this](const Configuration& config) {
                // Only reconnect if we are changing the path to the device
                if ((joystick_path.compare(config["controller_path"]) != 0)
                    || (joystick_acc_path.compare(config["accelerometer_path"]) != 0)) {
                    joystick_path     = config["controller_path"];
                    joystick_acc_path = config["accelerometer_path"];
                    connect_joystick();
                }

                actions.clear();
                for (const auto& action : config["action_scripts"].as<std::vector<std::string>>()) {
                    actions.push_back(action);
                }
            });

            on<Shutdown>().then([this] { disconnect_joystick(); });

            // Trigger when the joystick has an event to read
            on<IO>(joystick_fd, IO::READ).then([this] {
                // JoystickEvent has
                // Timestamp: uint32_t
                // Value:     int16_t
                // Type:      uint8_t
                // Number:    uint8_t
                constexpr size_t JOYSTICK_EVENT_SIZE =
                    sizeof(uint32_t) + sizeof(int16_t) + sizeof(uint8_t) + sizeof(uint8_t);

                // Try to read a JoystickEvent worth of data
                std::array<uint8_t, JOYSTICK_EVENT_SIZE> buffer;
                auto num_bytes = read(joystick_fd, buffer.data(), buffer.size());
                if (num_bytes > -1) {
                    for (const uint8_t& byte : buffer) {
                        event_buffer.push_back(byte);
                    }
                }

                while (event_buffer.size() >= JOYSTICK_EVENT_SIZE) {
                    // Construct and emit a JoystickEvent message
                    std::unique_ptr<JoystickEvent> event = std::make_unique<JoystickEvent>();

                    // Extract timestamp from event buffer in milliseconds and convert it to a
                    // NUClear::clock::time_point
                    auto timestamp   = std::chrono::milliseconds(*reinterpret_cast<uint32_t*>(event_buffer.data()));
                    event->timestamp = std::chrono::time_point<NUClear::clock, std::chrono::milliseconds>(timestamp);
                    // Erase timestamp from buffer
                    event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint32_t)));

                    // Extract value from event buffer a signed 16-bit value
                    event->value = *reinterpret_cast<int16_t*>(event_buffer.data());
                    // Erase value from buffer
                    event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(int16_t)));

                    // Extract type from event buffer an unsigned 8-bit value
                    bool is_axis = false, is_button = false, is_init = false, is_other = false;
                    switch (event_buffer.front()) {
                        case JoystickEvent::EventType::BUTTON:
                            event->type = JoystickEvent::EventType::BUTTON;
                            is_button   = true;
                            break;
                        case JoystickEvent::EventType::AXIS:
                            event->type = JoystickEvent::EventType::AXIS;
                            is_axis     = true;
                            break;
                        case JoystickEvent::EventType::INIT:
                            event->type = JoystickEvent::EventType::INIT;
                            is_init     = true;
                            break;
                        default:
                            event->type = JoystickEvent::EventType::UNKNOWN;
                            is_other    = true;
                            break;
                    }
                    // Erase type from buffer
                    event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint8_t)));

                    // Extract number from event buffer an unsigned 8-bit value
                    if (is_button) {
                        switch (event_buffer.front()) {
                            case JoystickEvent::Button::CROSS: event->button = JoystickEvent::Button::CROSS; break;
                            case JoystickEvent::Button::CIRCLE: event->button = JoystickEvent::Button::CIRCLE; break;
                            case JoystickEvent::Button::TRIANGLE:
                                event->button = JoystickEvent::Button::TRIANGLE;
                                break;
                            case JoystickEvent::Button::SQUARE: event->button = JoystickEvent::Button::SQUARE; break;
                            case JoystickEvent::Button::L1: event->button = JoystickEvent::Button::L1; break;
                            case JoystickEvent::Button::R1: event->button = JoystickEvent::Button::R1; break;
                            case JoystickEvent::Button::SELECT: event->button = JoystickEvent::Button::SELECT; break;
                            case JoystickEvent::Button::START: event->button = JoystickEvent::Button::START; break;
                            case JoystickEvent::Button::PS: event->button = JoystickEvent::Button::PS; break;
                            case JoystickEvent::Button::LEFT_JOYSTICK:
                                event->button = JoystickEvent::Button::LEFT_JOYSTICK;
                                break;
                            case JoystickEvent::Button::RIGHT_JOYSTICK:
                                event->button = JoystickEvent::Button::RIGHT_JOYSTICK;
                                break;
                            case JoystickEvent::Button::DPAD_UP: event->button = JoystickEvent::Button::DPAD_UP; break;
                            case JoystickEvent::Button::DPAD_DOWN:
                                event->button = JoystickEvent::Button::DPAD_DOWN;
                                break;
                            case JoystickEvent::Button::DPAD_LEFT:
                                event->button = JoystickEvent::Button::DPAD_LEFT;
                                break;
                            case JoystickEvent::Button::DPAD_RIGHT:
                                event->button = JoystickEvent::Button::DPAD_RIGHT;
                                break;
                        }
                    }
                    if (is_axis) {
                        switch (event_buffer.front()) {
                            case JoystickEvent::Axis::LEFT_JOYSTICK_HORIZONTAL:
                                event->axis = JoystickEvent::Axis::LEFT_JOYSTICK_HORIZONTAL;
                                break;
                            case JoystickEvent::Axis::LEFT_JOYSTICK_VERTICAL:
                                event->axis = JoystickEvent::Axis::LEFT_JOYSTICK_VERTICAL;
                                break;
                            case JoystickEvent::Axis::L2: event->axis = JoystickEvent::Axis::L2; break;
                            case JoystickEvent::Axis::RIGHT_JOYSTICK_HORIZONTAL:
                                event->axis = JoystickEvent::Axis::RIGHT_JOYSTICK_HORIZONTAL;
                                break;
                            case JoystickEvent::Axis::RIGHT_JOYSTICK_VERTICAL:
                                event->axis = JoystickEvent::Axis::RIGHT_JOYSTICK_VERTICAL;
                                break;
                            case JoystickEvent::Axis::R2: event->axis = JoystickEvent::Axis::R2; break;
                        }
                    }

                    if (is_init) {
                        // TODO: Do we need to handle this?
                    }

                    if (is_other) {
                        log<NUClear::WARN>("Unknown event on joystick. Ignoring");
                    }

                    // Erase axis/button from buffer
                    event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint8_t)));

                    if (!is_other) {
                        emit(std::move(event));
                    }
                }
            });

            // Trigger when the joystick accelerometer has an event to read
            on<IO>(joystick_acc_fd, IO::READ).then([this] {
                // Accelerometer comes in here .... if you know which axis is which
            });

            on<Trigger<JoystickEvent>>().then([this](const JoystickEvent& event) {
                switch (event.type.value) {
                    case JoystickEvent::EventType::BUTTON:
                        switch (event.button.value) {
                            case JoystickEvent::Button::TRIANGLE:
                                if (event.value > 0) {  // button down
                                    if (moving) {
                                        NUClear::log("Stop walking");
                                        emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                                    }
                                    else {
                                        NUClear::log("Start walking");
                                    }
                                    moving = !moving;
                                }
                                break;
                            case JoystickEvent::Button::SQUARE:
                                if (event.value > 0) {  // button down
                                    if (headLocked) {
                                        NUClear::log("Head unlocked");
                                    }
                                    else {
                                        NUClear::log("Head locked");
                                    }
                                    headLocked = !headLocked;
                                }
                                break;
                            case JoystickEvent::Button::CROSS:
                                if (event.value > 0) {  // button down
                                    NUClear::log("Triggering actions");
                                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                                    emit(std::make_unique<ExecuteScriptByName>(id, actions));
                                }
                                break;
                            case JoystickEvent::Button::CIRCLE:
                                if (event.value > 0) {  // button down
                                    NUClear::log("Standing");
                                    emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {90}}));
                                    emit(std::make_unique<ExecuteScriptByName>(id, "Stand.yaml"));
                                }
                                break;
                            case JoystickEvent::Button::L1:
                                if (event.value > 0) {  // button down
                                    NUClear::log("Requesting Left Front Kick");
                                    emit(std::make_unique<KickScriptCommand>(KickScriptCommand{
                                        Eigen::Vector3d(1, 0, 0),  // vector pointing forward relative to robot
                                        LimbID::LEFT_LEG}));
                                }
                                break;
                            case JoystickEvent::Button::R1:
                                if (event.value > 0) {  // button down
                                    NUClear::log("Requesting Right Front Kick");
                                    emit(std::make_unique<KickScriptCommand>(KickScriptCommand(
                                        Eigen::Vector3d(1, 0, 0),  // vector pointing forward relative to robot
                                        LimbID::RIGHT_LEG)));
                                }
                                break;
                            default: break;
                        }
                    case JoystickEvent::EventType::AXIS:
                        switch (event.axis.value) {
                            case JoystickEvent::Axis::LEFT_JOYSTICK_HORIZONTAL:
                                // y is left relative to robot
                                // strafe[1] = -event.value;
                                rotationalSpeed = -event.value;
                                break;
                            case JoystickEvent::Axis::LEFT_JOYSTICK_VERTICAL:
                                // x is forward relative to robot
                                strafe[0] = -event.value;
                                break;
                            case JoystickEvent::Axis::RIGHT_JOYSTICK_HORIZONTAL: headYaw = -event.value; break;
                            case JoystickEvent::Axis::RIGHT_JOYSTICK_VERTICAL: headPitch = -event.value; break;

                            case JoystickEvent::Axis::L2:
                            case JoystickEvent::Axis::R2:
                            default: break;
                        }
                    case JoystickEvent::EventType::UNKNOWN:
                    case JoystickEvent::EventType::INIT:
                    default: break;
                }
            });

            // Keep an eye on the joystick devices
            on<Every<1, std::chrono::seconds>, Single>().then([this] {
                // If it's closed then we should try to reconnect
                if ((joystick_fd > -1) && (joystick_acc_fd > -1)) {
                    bool joystick_valid     = !(fcntl(joystick_fd, F_GETFL) < 0 && errno == EBADF);
                    bool joystick_acc_valid = !(fcntl(joystick_acc_fd, F_GETFL) < 0 && errno == EBADF);
                    if (!joystick_valid || !joystick_acc_valid) {
                        log<NUClear::WARN>("Joystick is not valid. Reconnecting.");
                        connect_joystick();
                    }
                }
            });

            // output walk command based on updated strafe and rotation speed from joystick
            // TODO: potential performance gain: ignore if value hasn't changed since last emit?
            on<Every<20, Per<std::chrono::seconds>>>().then([this] {
                if (!headLocked) {
                    auto headCommand        = std::make_unique<HeadCommand>();
                    headCommand->yaw        = headYaw / std::numeric_limits<short>::max() * 1.5;
                    headCommand->pitch      = headPitch / std::numeric_limits<short>::max();
                    headCommand->robotSpace = true;
                    emit(std::move(headCommand));
                }

                if (moving) {
                    // TODO: hacked to not allow backwards movement for stability
                    // arma::vec s = { std::max(strafe[0], 0.0), strafe[1] };
                    arma::vec s          = strafe;
                    arma::vec strafeNorm = s / std::numeric_limits<short>::max();

                    auto rotationalSpeedNorm = rotationalSpeed / std::numeric_limits<short>::max();
                    auto transform           = Transform2D(strafeNorm, rotationalSpeedNorm);
                    emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(transform)));
                }
            });

            emit<Scope::DIRECT>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "PS3Walk",
                {std::pair<float, std::set<LimbID>>(
                    0, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<ServoID>&) {
                    emit(std::make_unique<ActionPriorites>(ActionPriorites{id, {0}}));
                }}));
        }

        void PS3Walk::connect_joystick() {
            // Make sure joystick file descriptors are closed.
            disconnect_joystick();

            NUClear::log<NUClear::INFO>(fmt::format("Connecting to {} and {}", joystick_path, joystick_acc_path));
            joystick_fd     = open(joystick_path.c_str(), O_RDONLY | O_NONBLOCK);
            joystick_acc_fd = open(joystick_acc_path.c_str(), O_RDONLY | O_NONBLOCK);
        }

        void PS3Walk::disconnect_joystick() {
            if (joystick_fd != -1) {
                ::close(joystick_fd);
            }
            if (joystick_acc_fd != -1) {
                ::close(joystick_acc_fd);
            }
        }
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
