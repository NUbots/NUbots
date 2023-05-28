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

#include "Joystick.hpp"

#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include "unistd.h"

Joystick::Joystick() : path("/dev/input/js0") {
    openPath(path);
}

Joystick::Joystick(const std::string& devicePath) : path(devicePath) {
    openPath(devicePath);
}

void Joystick::openPath(const std::string& devicePath) {
    std::cout << "Connecting to " << devicePath << std::endl;
    fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event) const {
    const int bytes = read(fd, event, sizeof(*event));

    if (bytes == -1) {
        return false;
    }

    // NOTE if this condition is not met, we're probably out of sync and this
    // Joystick instance is likely unusable
    return bytes == sizeof(*event);
}

bool Joystick::found() const {
    return fd >= 0;
}

bool Joystick::valid() const {
    // Check if we can get the stats
    return !(fcntl(fd, F_GETFL) < 0 && errno == EBADF);
}

void Joystick::reconnect() {
    close(fd);
    openPath(path);
}

Joystick::~Joystick() {
    close(fd);
}
