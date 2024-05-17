/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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

#include "NUsightHarness.hpp"

#include "utility/nusight/NUhelpers.hpp"

namespace module::support {

    using std::chrono::milliseconds;

    using utility::nusight::graph;

    NUsightHarness::NUsightHarness(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Every<50, milliseconds>>().then([this] {
            double period  = 10;
            double freq    = 1.0 / period;
            double t       = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            double sine    = std::sin(2.0 * M_PI * freq * t);
            double cosine  = std::cos(2.0 * M_PI * freq * t);
            double dsine   = 2.0 * sine;
            double dcosine = 4.0 * cosine;

            emit(graph("Debug Waves", sine, cosine, dsine, dcosine));
        });
    }
}  // namespace module::support
