#include "PS3Controller.h"

#include <fcntl.h>
#include <fmt/format.h>
#include <array>
#include <chrono>
#include <limits>

#include "extension/Configuration.h"

#include "message/input/PS3Controller.h"

namespace module {
namespace input {

    enum class EventType : uint8_t { BUTTON = 0x01, AXIS = 0x02, INIT = 0x80 };
    enum class Button : uint8_t {
        CROSS          = 0,
        CIRCLE         = 1,
        TRIANGLE       = 2,
        SQUARE         = 3,
        L1             = 4,
        R1             = 5,
        SELECT         = 8,
        START          = 9,
        PS             = 10,
        JOYSTICK_LEFT  = 11,
        JOYSTICK_RIGHT = 12,
        DPAD_UP        = 13,
        DPAD_DOWN      = 14,
        DPAD_LEFT      = 15,
        DPAD_RIGHT     = 16
    };
    enum class Axis : uint8_t {
        JOYSTICK_LEFT_HORIZONTAL = 0,
        JOYSTICK_LEFT_VERTICAL,
        L2,
        JOYSTICK_RIGHT_HORIZONTAL,
        JOYSTICK_RIGHT_VERTICAL,
        R2,
        ACCELEROMETER_X = 23,
        ACCELEROMETER_Y,
        ACCELEROMETER_Z
    };

    using extension::Configuration;

    using message::input::PS3Controller::Accelerometer;
    using message::input::PS3Controller::CircleButton;
    using message::input::PS3Controller::CrossButton;
    using message::input::PS3Controller::DPadButton;
    using message::input::PS3Controller::L1Button;
    using message::input::PS3Controller::L2Trigger;
    using message::input::PS3Controller::LeftJoystick;
    using message::input::PS3Controller::LeftJoystickButton;
    using message::input::PS3Controller::PSButton;
    using message::input::PS3Controller::R1Button;
    using message::input::PS3Controller::R2Trigger;
    using message::input::PS3Controller::RightJoystick;
    using message::input::PS3Controller::RightJoystickButton;
    using message::input::PS3Controller::SelectButton;
    using message::input::PS3Controller::SquareButton;
    using message::input::PS3Controller::StartButton;
    using message::input::PS3Controller::TriangleButton;

    PS3Controller::PS3Controller(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("PS3Controller.yaml").then([this](const Configuration& config) {
            // Only reconnect if we are changing the path to the device
            if ((controller_path.compare(config["controller_path"]) != 0)
                || (accelerometer_path.compare(config["accelerometer_path"]) != 0)) {
                controller_path    = config["controller_path"];
                accelerometer_path = config["accelerometer_path"];
                connect();
            }
        });

        on<Shutdown>().then([this] { disconnect(); });

        // Details on Linux Joystick API
        // https://www.kernel.org/doc/Documentation/input/joystick-api.txt
        // Trigger when the joystick has an data to read
        on<IO>(controller_fd, IO::READ).then([this] {
            // JoystickEvent has
            // Timestamp: uint32_t
            // Value:     int16_t
            // Type:      uint8_t
            // Number:    uint8_t
            constexpr size_t JOYSTICK_EVENT_SIZE =
                sizeof(uint32_t) + sizeof(int16_t) + sizeof(uint8_t) + sizeof(uint8_t);

            // Try to read a JoystickEvent worth of data
            std::array<uint8_t, JOYSTICK_EVENT_SIZE> buffer;
            auto num_bytes = read(controller_fd, buffer.data(), buffer.size());
            if (num_bytes > -1) {
                for (const uint8_t& byte : buffer) {
                    event_buffer.push_back(byte);
                }
            }

            while (event_buffer.size() >= JOYSTICK_EVENT_SIZE) {
                // Extract timestamp from event buffer in milliseconds and convert it to a NUClear::clock::time_point
                auto timestamp = std::chrono::time_point<NUClear::clock, std::chrono::milliseconds>(
                    std::chrono::milliseconds(*reinterpret_cast<uint32_t*>(event_buffer.data())));
                // Erase timestamp from buffer
                event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint32_t)));

                // Extract value from event buffer as a signed 16-bit value
                int16_t value = *reinterpret_cast<int16_t*>(event_buffer.data());
                // Erase value from buffer
                event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(int16_t)));

                // Extract type from event buffer an unsigned 8-bit value
                bool is_axis = false, is_button = false, is_init = false;
                uint8_t type = event_buffer.front();
                if (type & static_cast<uint8_t>(EventType::INIT)) {
                    type &= ~static_cast<uint8_t>(EventType::INIT);
                    is_init = true;
                }
                switch (static_cast<EventType>(type)) {
                    case EventType::BUTTON: is_button = true; break;
                    case EventType::AXIS: is_axis = true; break;
                    case EventType::INIT: /* This is handled above. */ break;
                }
                // Erase type from buffer
                event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint8_t)));

                // Extract number from event buffer an unsigned 8-bit value
                if (is_button) {
                    switch (static_cast<Button>(event_buffer.front())) {
                        case Button::CROSS: emit(std::make_unique<CrossButton>(timestamp, value > 0, is_init)); break;
                        case Button::CIRCLE: emit(std::make_unique<CircleButton>(timestamp, value > 0, is_init)); break;
                        case Button::TRIANGLE:
                            emit(std::make_unique<TriangleButton>(timestamp, value > 0, is_init));
                            break;
                        case Button::SQUARE: emit(std::make_unique<SquareButton>(timestamp, value > 0, is_init)); break;
                        case Button::L1: emit(std::make_unique<L1Button>(timestamp, value > 0, is_init)); break;
                        case Button::R1: emit(std::make_unique<R1Button>(timestamp, value > 0, is_init)); break;
                        case Button::SELECT: emit(std::make_unique<SelectButton>(timestamp, value > 0, is_init)); break;
                        case Button::START: emit(std::make_unique<StartButton>(timestamp, value > 0, is_init)); break;
                        case Button::PS: emit(std::make_unique<PSButton>(timestamp, value > 0, is_init)); break;
                        case Button::JOYSTICK_LEFT:
                            emit(std::make_unique<LeftJoystickButton>(timestamp, value > 0, is_init));
                            break;
                        case Button::JOYSTICK_RIGHT:
                            emit(std::make_unique<RightJoystickButton>(timestamp, value > 0, is_init));
                            break;
                        case Button::DPAD_UP:
                            emit(
                                std::make_unique<DPadButton>(timestamp, DPadButton::Direction::UP, value > 0, is_init));
                            break;
                        case Button::DPAD_DOWN:
                            emit(std::make_unique<DPadButton>(
                                timestamp, DPadButton::Direction::DOWN, value > 0, is_init));
                            break;
                        case Button::DPAD_LEFT:
                            emit(std::make_unique<DPadButton>(
                                timestamp, DPadButton::Direction::LEFT, value > 0, is_init));
                            break;
                        case Button::DPAD_RIGHT:
                            emit(std::make_unique<DPadButton>(
                                timestamp, DPadButton::Direction::RIGHT, value > 0, is_init));
                            break;
                    }
                }
                else if (is_axis) {
                    float normalised =
                        static_cast<float>(value) / static_cast<float>(std::numeric_limits<int16_t>::max());
                    switch (static_cast<Axis>(event_buffer.front())) {
                        case Axis::JOYSTICK_LEFT_HORIZONTAL:
                            emit(std::make_unique<LeftJoystick>(
                                timestamp, LeftJoystick::Direction::HORIZONTAL, normalised, is_init));
                            break;
                        case Axis::JOYSTICK_LEFT_VERTICAL:
                            emit(std::make_unique<LeftJoystick>(
                                timestamp, LeftJoystick::Direction::VERTICAL, normalised, is_init));
                            break;
                        case Axis::L2: emit(std::make_unique<L2Trigger>(timestamp, normalised, is_init)); break;
                        case Axis::JOYSTICK_RIGHT_HORIZONTAL:
                            emit(std::make_unique<RightJoystick>(
                                timestamp, RightJoystick::Direction::HORIZONTAL, normalised, is_init));
                            break;
                        case Axis::JOYSTICK_RIGHT_VERTICAL:
                            emit(std::make_unique<RightJoystick>(
                                timestamp, RightJoystick::Direction::VERTICAL, normalised, is_init));
                            break;
                        case Axis::R2: emit(std::make_unique<R2Trigger>(timestamp, normalised, is_init)); break;
                        case Axis::ACCELEROMETER_X: /* TODO: Handle this. */ break;
                        case Axis::ACCELEROMETER_Y: /* TODO: Handle this. */ break;
                        case Axis::ACCELEROMETER_Z: /* TODO: Handle this. */ break;
                    }
                }
                else {
                    log<NUClear::WARN>("Unknown event on joystick. Ignoring.");
                    log<NUClear::WARN>(fmt::format("Timestamp: {}", timestamp.time_since_epoch().count()));
                    log<NUClear::WARN>(fmt::format("Value....: {}", value));
                    log<NUClear::WARN>(
                        fmt::format("Type.....: {}", is_init ? type | static_cast<uint8_t>(EventType::INIT) : type));
                    log<NUClear::WARN>(fmt::format("Number...: {}", static_cast<int>(event_buffer.front())));
                }

                // Erase axis/button from buffer
                event_buffer.erase(event_buffer.begin(), std::next(event_buffer.begin(), sizeof(uint8_t)));
            }
        });

        // Trigger when the joystick accelerometer has an event to read
        on<IO>(accelerometer_fd, IO::READ).then([this] {
            // Accelerometer comes in here .... if you know which axis is which
        });

        // Keep an eye on the joystick devices
        on<Every<1, std::chrono::seconds>, Single>().then([this] {
            // If it's closed then we should try to reconnect
            if ((controller_fd > -1) && (accelerometer_fd > -1)) {
                bool controller_valid    = !(fcntl(controller_fd, F_GETFL) < 0 && errno == EBADF);
                bool accelerometer_valid = !(fcntl(accelerometer_fd, F_GETFL) < 0 && errno == EBADF);
                if (!controller_valid || !accelerometer_valid) {
                    log<NUClear::WARN>("Joystick is not valid. Reconnecting.");
                    connect();
                }
            }
        });
    }

    void PS3Controller::connect() {
        // Make sure joystick file descriptors are closed.
        disconnect();

        NUClear::log<NUClear::INFO>(fmt::format("Connecting to {} and {}", controller_path, accelerometer_path));
        controller_fd    = open(controller_path.c_str(), O_RDONLY | O_NONBLOCK);
        accelerometer_fd = open(accelerometer_path.c_str(), O_RDONLY | O_NONBLOCK);
    }

    void PS3Controller::disconnect() {
        if (controller_fd != -1) {
            ::close(controller_fd);
        }
        if (accelerometer_fd != -1) {
            ::close(accelerometer_fd);
        }
    }
}  // namespace input
}  // namespace module
