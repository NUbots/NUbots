// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>

#ifndef __JOYSTICK_HPP__
#define __JOYSTICK_HPP__

#include <string>

#define JS_EVENT_BUTTON 0x01  // button pressed/released
#define JS_EVENT_AXIS   0x02  // joystick moved
#define JS_EVENT_INIT   0x80  // initial state of device

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent {
public:
    JoystickEvent() {}

    /**
     * The timestamp of the event, in milliseconds.
     */
    unsigned int time{0};

    /**
     * The value associated with this joystick event.
     * For buttons this will be either 1 (down) or 0 (up).
     * For axes, this will range between -32768 and 32767.
     */
    short value{0};

    /**
     * The event type.
     */
    unsigned char type{0};

    /**
     * The axis/button number.
     */
    unsigned char number{0};

    /**
     * Returns true if this event is the result of a button press.
     */
    bool isButton() const {
        return (type & JS_EVENT_BUTTON) != 0;
    }

    /**
     * Returns true if this event is the result of an axis movement.
     */
    bool isAxis() const {
        return (type & JS_EVENT_AXIS) != 0;
    }

    /**
     * Returns true if this event is part of the initial state obtained when
     * the joystick is first connected to.
     */
    bool isInitialState() const {
        return (type & JS_EVENT_INIT) != 0;
    }
};

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick {
private:
    void openPath(const std::string& devicePath);

    int _fd{-1};
    std::string path;

public:
    ~Joystick();

    /**
     * Initialises an instance for the first joystick: /dev/input/js0
     */
    Joystick();

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
    bool found() const;

    /**
     * Returns true if the joystick file descriptor is valid, otherwise false
     */
    bool valid() const;

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

#endif  // __JOYSTICK_HPP__
