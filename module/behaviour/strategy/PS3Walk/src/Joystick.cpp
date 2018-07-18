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

#include "Joystick.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <sstream>
#include <string>
#include "unistd.h"

Joystick::Joystick() : _fd({-1, -1}), path({"/dev/input/js0", "/dev/input/js1"}) {}

Joystick::Joystick(const std::string& device, const std::string& acc) : _fd({-1, -1}), path({device, acc}) {
    reconnect();
}

void Joystick::connect(const std::string& device, const std::string& acc) {
    path[0] = device;
    path[1] = acc;
    reconnect();
}

bool Joystick::sample(JoystickEvent* event) {
    int bytes = read(_fd[0], event, sizeof(*event));

    // NOTE if this condition is not met, we're probably out of sync and this
    // Joystick instance is likely unusable
    return (bytes == sizeof(*event));
}

bool Joystick::sample_acc(JoystickEvent* event) {
    int bytes = read(_fd[1], event, sizeof(*event));

    // NOTE if this condition is not met, we're probably out of sync and this
    // Joystick instance is likely unusable
    return (bytes == sizeof(*event));
}

bool Joystick::found() {
    return (_fd[0] >= 0);
}

bool Joystick::valid() {
    // Check if we can get the stats
    return !(fcntl(_fd[0], F_GETFL) < 0 && errno == EBADF);
}

void Joystick::reconnect() {
    close();
    std::cout << "Connecting to " << path[0] << " and " << path[1] << std::endl;
    _fd[0] = open(path[0].c_str(), O_RDONLY | O_NONBLOCK);
    _fd[1] = open(path[1].c_str(), O_RDONLY | O_NONBLOCK);
}

void Joystick::close() {
    ::close(_fd[0]);
    ::close(_fd[1]);
}

Joystick::~Joystick() {
    close();
}
