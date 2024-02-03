/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef MODULES_SUPPORT_SIGNALCATCHER_HPP
#define MODULES_SUPPORT_SIGNALCATCHER_HPP

#include <exception>
#include <nuclear>

namespace module::support {

    /**
     * @brief Handles OS interrupt signals.
     *
     * @details
     *  This module catches SIGINT and SIGSEGV and changes how they are handled. Only one SignalCatcher should ever
     * be installed in the program and this module will be incompatible with any code that uses signal(SIGINT, ...)
     *  or signal(SIGSEGV, ...) anywhere else. Additionally if there are multiple NUClear::PowerPlant instances this
     *  module may cause a race condition and incorrectly shut down the wrong power plant.
     *
     * @author Trent Houliston
     */
    class SignalCatcher : public NUClear::Reactor {
    public:
        explicit SignalCatcher(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::support

#endif  // MODULES_SUPPORT_SIGNALCATCHER_HPP
