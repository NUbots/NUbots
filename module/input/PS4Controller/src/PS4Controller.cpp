#include "PS4Controller.h"

#include <fcntl.h>
#include <fmt/format.h>
#include <array>
#include <chrono>
#include <limits>

#include "extension/Configuration.h"

#include "message/input/PSController.h"

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
        SHARE          = 8,
        OPTIONS        = 9,
        PS             = 10,
        JOYSTICK_LEFT  = 11,
        JOYSTICK_RIGHT = 12,
    };
    enum class Axis : uint8_t {
        JOYSTICK_LEFT_HORIZONTAL = 0,
        JOYSTICK_LEFT_VERTICAL,
        L2,
        JOYSTICK_RIGHT_HORIZONTAL,
        JOYSTICK_RIGHT_VERTICAL,
        R2,
        DPAD_HORIZONTAL,
        DPAD_VERTICAL,
        ACCELEROMETER_X = 23,
        ACCELEROMETER_Y,
        ACCELEROMETER_Z,
        TOUCHPAD
    };

#pragma pack(push, 1)
    struct JoystickEvent {
        uint32_t timestamp;  // event timestamp in milliseconds
        int16_t value;       // value
        uint8_t type;        // event type
        uint8_t number;      // axis/button number
    };
    // Check that this struct is not cache alligned
    static_assert(sizeof(JoystickEvent) == 8, "The compiler is adding padding to this struct, Bad compiler!");
#pragma pack(pop)

    using extension::Configuration;

    using message::input::Accelerometer;
    using message::input::CircleButton;
    using message::input::CrossButton;
    using message::input::DPadButton;
    using message::input::L1Button;
    using message::input::L2Trigger;
    using message::input::LeftJoystick;
    using message::input::LeftJoystickButton;
    using message::input::OptionsButton;
    using message::input::PSButton;
    using message::input::R1Button;
    using message::input::R2Trigger;
    using message::input::RightJoystick;
    using message::input::RightJoystickButton;
    using message::input::ShareButton;
    using message::input::SquareButton;
    using message::input::TouchPad;
    using message::input::TriangleButton;

    PS4Controller::PS4Controller(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), controller_fd(-1), accelerometer_fd(-1) {

        on<Configuration>("PS4Controller.yaml").then([this](const Configuration& config) {
            // Only reconnect if we are changing the path to the device
            if ((controller_path.compare(config["controller_path"]) != 0)
                || (accelerometer_path.compare(config["accelerometer_path"]) != 0)) {
                controller_path    = config["controller_path"];
                accelerometer_path = config["accelerometer_path"];
                connect();
            }
        });

        on<Shutdown>().then([this] { disconnect(); });

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
            if ((controller_fd < 0) || (accelerometer_fd < 0)) {
                log<NUClear::WARN>("Joystick is not valid. Reconnecting.");
                connect();
            }
        });
    }

    void PS4Controller::connect() {
        // Make sure joystick file descriptors are closed.
        disconnect();

        NUClear::log<NUClear::INFO>(fmt::format("Connecting to {} and {}", controller_path, accelerometer_path));
        controller_fd    = open(controller_path.c_str(), O_RDONLY | O_NONBLOCK);
        accelerometer_fd = open(accelerometer_path.c_str(), O_RDONLY | O_NONBLOCK);

        if (controller_fd > -1) {
            // Details on Linux Joystick API
            // https://www.kernel.org/doc/Documentation/input/joystick-api.txt
            // Trigger when the joystick has an data to read
            controller_reaction     = on<IO>(controller_fd, IO::READ).then([this] {
                // Try to read a JoystickEvent worth of data
                for (JoystickEvent event;
                     read(controller_fd, &event, sizeof(JoystickEvent)) == sizeof(JoystickEvent);) {
                    // Extract type from event buffer an unsigned 8-bit value
                    bool is_axis = false, is_button = false, is_init = false;
                    if (event.type & static_cast<uint8_t>(EventType::INIT)) {
                        event.type &= ~static_cast<uint8_t>(EventType::INIT);
                        is_init = true;
                    }
                    switch (static_cast<EventType>(event.type)) {
                        case EventType::BUTTON: is_button = true; break;
                        case EventType::AXIS: is_axis = true; break;
                        case EventType::INIT: /* This is handled above. */ break;
                    }

                    // Extract number from event buffer an unsigned 8-bit value
                    if (is_button) {
                        switch (static_cast<Button>(event.number)) {
                            case Button::CROSS:
                                emit(std::make_unique<CrossButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::CIRCLE:
                                emit(std::make_unique<CircleButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::TRIANGLE:
                                emit(std::make_unique<TriangleButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::SQUARE:
                                emit(std::make_unique<SquareButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::L1:
                                emit(std::make_unique<L1Button>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::R1:
                                emit(std::make_unique<R1Button>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::SHARE:
                                emit(std::make_unique<ShareButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::OPTIONS:
                                emit(std::make_unique<OptionsButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::PS:
                                emit(std::make_unique<PSButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::JOYSTICK_LEFT:
                                emit(std::make_unique<LeftJoystickButton>(event.timestamp, event.value > 0, is_init));
                                break;
                            case Button::JOYSTICK_RIGHT:
                                emit(std::make_unique<RightJoystickButton>(event.timestamp, event.value > 0, is_init));
                                break;
                        }
                    }
                    else if (is_axis) {
                        float normalised =
                            static_cast<float>(event.value) / static_cast<float>(std::numeric_limits<int16_t>::max());
                        switch (static_cast<Axis>(event.number)) {
                            case Axis::JOYSTICK_LEFT_HORIZONTAL:
                                emit(std::make_unique<LeftJoystick>(
                                    event.timestamp, LeftJoystick::Direction::HORIZONTAL, normalised, is_init));
                                break;
                            case Axis::JOYSTICK_LEFT_VERTICAL:
                                emit(std::make_unique<LeftJoystick>(
                                    event.timestamp, LeftJoystick::Direction::VERTICAL, normalised, is_init));
                                break;
                            case Axis::L2:
                                emit(std::make_unique<L2Trigger>(event.timestamp, normalised, is_init));
                                break;
                            case Axis::JOYSTICK_RIGHT_HORIZONTAL:
                                emit(std::make_unique<RightJoystick>(
                                    event.timestamp, RightJoystick::Direction::HORIZONTAL, normalised, is_init));
                                break;
                            case Axis::JOYSTICK_RIGHT_VERTICAL:
                                emit(std::make_unique<RightJoystick>(
                                    event.timestamp, RightJoystick::Direction::VERTICAL, normalised, is_init));
                                break;
                            case Axis::R2:
                                emit(std::make_unique<R2Trigger>(event.timestamp, normalised, is_init));
                                break;
                            case Axis::DPAD_HORIZONTAL:
                                if (event.value < 0) {
                                    emit(std::make_unique<DPadButton>(
                                        event.timestamp, DPadButton::Direction::LEFT, true, is_init));
                                    dpad_left_pressed = true;
                                }
                                else if (event.value > 0) {
                                    emit(std::make_unique<DPadButton>(
                                        event.timestamp, DPadButton::Direction::RIGHT, true, is_init));
                                    dpad_right_pressed = true;
                                }
                                else {
                                    if (dpad_left_pressed) {
                                        emit(std::make_unique<DPadButton>(
                                            event.timestamp, DPadButton::Direction::LEFT, false, is_init));
                                        dpad_left_pressed = false;
                                    }
                                    if (dpad_right_pressed) {
                                        emit(std::make_unique<DPadButton>(
                                            event.timestamp, DPadButton::Direction::RIGHT, false, is_init));
                                        dpad_right_pressed = false;
                                    }
                                }
                                break;
                            case Axis::DPAD_VERTICAL:
                                if (event.value < 0) {
                                    emit(std::make_unique<DPadButton>(
                                        event.timestamp, DPadButton::Direction::UP, true, is_init));
                                    dpad_up_pressed = true;
                                }
                                else if (event.value > 0) {
                                    emit(std::make_unique<DPadButton>(
                                        event.timestamp, DPadButton::Direction::DOWN, true, is_init));
                                    dpad_down_pressed = true;
                                }
                                else {
                                    if (dpad_up_pressed) {
                                        emit(std::make_unique<DPadButton>(
                                            event.timestamp, DPadButton::Direction::UP, false, is_init));
                                        dpad_up_pressed = false;
                                    }
                                    if (dpad_down_pressed) {
                                        emit(std::make_unique<DPadButton>(
                                            event.timestamp, DPadButton::Direction::DOWN, false, is_init));
                                        dpad_down_pressed = false;
                                    }
                                }
                                break;
                            case Axis::ACCELEROMETER_X: break;
                            case Axis::ACCELEROMETER_Y: break;
                            case Axis::ACCELEROMETER_Z: break;
                            case Axis::TOUCHPAD: break;
                        }
                    }
                    else {
                        log<NUClear::WARN>("Unknown event on joystick. Ignoring.");
                        log<NUClear::WARN>(fmt::format("Timestamp: {}", event.timestamp));
                        log<NUClear::WARN>(fmt::format("Value....: {}", event.value));
                        log<NUClear::WARN>(
                            fmt::format("Type.....: {}",
                                        is_init ? event.type | static_cast<uint8_t>(EventType::INIT) : event.type));
                        log<NUClear::WARN>(fmt::format("Number...: {}", static_cast<int>(event.number)));
                    }
                }
            });
            controller_err_reaction = on<IO>(controller_fd, IO::ERROR).then([this] { disconnect(); });
        }

        if (accelerometer_fd > -1) {
            // Trigger when the joystick accelerometer has an event to read
            accelerometer_reaction     = on<IO>(accelerometer_fd, IO::READ).then([this] {
                // Accelerometer comes in here .... if you know which axis is which
            });
            accelerometer_err_reaction = on<IO>(accelerometer_fd, IO::ERROR).then([this] { disconnect(); });
        }
    }

    void PS4Controller::disconnect() {
        controller_reaction.unbind();
        controller_err_reaction.unbind();
        accelerometer_reaction.unbind();
        accelerometer_err_reaction.unbind();
        if (controller_fd != -1) {
            ::close(controller_fd);
            controller_fd = -1;
        }
        if (accelerometer_fd != -1) {
            ::close(accelerometer_fd);
            accelerometer_fd = -1;
        }
    }
}  // namespace input
}  // namespace module
