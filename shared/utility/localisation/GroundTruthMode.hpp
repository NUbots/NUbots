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
#ifndef UTILITY_LOCALISATION_GROUNDTRUTHMODE_HPP
#define UTILITY_LOCALISATION_GROUNDTRUTHMODE_HPP

namespace utility::localisation {

    /// @brief Emitted when ground truth perception is switched on or off (see input::NUSimGroundTruth). While it is
    /// enabled, Field and Ball come from the simulator and the vision-based localisation modules stay quiet.
    struct GroundTruthMode {
        bool enabled = false;
    };

}  // namespace utility::localisation

#endif  // UTILITY_LOCALISATION_GROUNDTRUTHMODE_HPP
