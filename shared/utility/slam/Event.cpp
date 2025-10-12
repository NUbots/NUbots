/*
* MIT License
*
* Copyright (c) 2025 NUbots
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

#include <print>
#include <string>
#include "system/SystemBase.hpp"
#include "Event.hpp"

namespace utility::slam {

    Event::Event(double time)
    : time_(time)
    , verbosity_(1)
    {}

    Event::Event(double time, int verbosity)
        : time_(time)
        , verbosity_(verbosity)
    {}

    Event::~Event() = default;

    void Event::process(SystemBase & system)
    {
        if (verbosity_ > 0)
        {
            std::print("[t={:07.3f}s] {}", time_, getProcessString());
        }

        // Time update
        system.predict(time_);

        // Event-specific implementation
        update(system);

        if (verbosity_ > 0)
        {
            std::println(" done");
        }
    }

    std::string Event::getProcessString() const
    {
        return "Processing event:";
    }

}  // namespace utility::slam
