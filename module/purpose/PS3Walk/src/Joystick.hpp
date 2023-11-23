/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#ifndef MODULE_PURPOSE_PS3WALK_JOYSTICK_HPP
#define MODULE_PURPOSE_PS3WALK_JOYSTICK_HPP

#include <string>

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent {
private:
    static constexpr unsigned char JS_EVENT_BUTTON{0x01};  // button pressed/released
    static constexpr unsigned char JS_EVENT_AXIS{0x02};    // joystick moved
    static constexpr unsigned char JS_EVENT_INIT{0x80};    // initial state of device

public:
    JoystickEvent() = default;

    /**
     * The timestamp of the event, in milliseconds.
     */
    uint32_t time = 0;

    /**
     * The value associated with this joystick event.
     * For buttons this will be either 1 (down) or 0 (up).
     * For axes, this will range between -32768 and 32767.
     */
    int16_t value = 0;

    /**
     * The event type.
     */
    uint8_t type = 0;

    /**
     * The axis/button number.
     */
    uint8_t number = 0;

    /**
     * Returns true if this event is the result of a button press.
     */
    [[nodiscard]] bool isButton() const {
        return (type & JS_EVENT_BUTTON) != 0;
    }

    /**
     * Returns true if this event is the result of an axis movement.
     */
    [[nodiscard]] bool isAxis() const {
        return (type & JS_EVENT_AXIS) != 0;
    }

    /**
     * Returns true if this event is part of the initial state obtained when
     * the joystick is first connected to.
     */
    [[nodiscard]] bool isInitialState() const {
        return (type & JS_EVENT_INIT) != 0;
    }
};

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick {
private:
    void openPath(const std::string& devicePath);

    int fd = -1;
    std::string path;

public:
    /**
     * Closes the fd and destroys the object
     */
    ~Joystick();

    /**
     * Initialises an instance for the first joystick: /dev/input/js0
     */
    Joystick();

    // Delete the move and copy constructors and operators to maintain file descriptor sanitation
    Joystick(Joystick& other)             = delete;
    Joystick(Joystick&& other)            = delete;
    Joystick& operator=(Joystick& other)  = delete;
    Joystick& operator=(Joystick&& other) = delete;

    /**
     * Initialises an instance for the joystick with the specified,
     * zero-indexed number.
     */
    Joystick(int joystickNumber);

    /**
     * Initialises an instance for the joystick device specified.
     */
    Joystick(const std::string& devicePath);

    /**
     * Returns true if the joystick was found and may be used, otherwise false.
     */
    [[nodiscard]] bool found() const;

    /**
     * Returns true if the joystick file descriptor is valid, otherwise false
     */
    [[nodiscard]] bool valid() const;

    /**
     * Reconnect to the joystick
     */
    void reconnect();

    /**
     * Attempts to populate the provided JoystickEvent instance with data
     * from the joystick. Returns true if data is available, otherwise false.
     */
    bool sample(JoystickEvent* event) const;
};

#endif  // MODULE_PURPOSE_PS3WALK_JOYSTICK_HPP
