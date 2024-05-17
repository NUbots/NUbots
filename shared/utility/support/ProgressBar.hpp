/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef UTILITY_SUPPORT_PROGRESSBAR_HPP
#define UTILITY_SUPPORT_PROGRESSBAR_HPP

#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace utility::support {

    struct ProgressBarState {};

    /**
     * @brief This class displays a progress on a single line in the terminal showing rates and a bar to show progress
     */
    class ProgressBar {
    public:
        /**
         * @brief Construct a new Progress Bar object using the provided unit
         *
         * @param unit the unit to display
         */
        ProgressBar();

        ProgressBar(const ProgressBar&)            = delete;
        ProgressBar& operator=(const ProgressBar&) = delete;

        ProgressBar(ProgressBar&& other) noexcept;
        ProgressBar& operator=(ProgressBar&& other) noexcept;

        /**
         * Destroy the Progress Bar object, will call close() if the progress bar is still valid.
         */
        ~ProgressBar();

        /**
         * @brief Updates the progress bar using the new values for current and total.
         * Calls to this method after calling close() will throw an error.
         *
         * @param current   the current number of units that have been completed
         * @param total     the total number of units to be completed
         * @param unit      the units that the measurement is displayed int
         * @param status    a string to display besides the progress bar
         */
        void update(const double& current,
                    const double& total,
                    const std::string& unit   = "",
                    const std::string& status = "");

        /**
         * @brief Updates the progress bar using the new values for current and total with types that are castable to
         * double. Calls to this method after calling close() will throw an error.
         *
         * @tparam T    a type that is castable to double to be used with update(double, double)
         * @tparam U    a type that is castable to double to be used with update(double, double)
         *
         * @param current   the current number of units that have been completed
         * @param total     the total number of units to be completed
         * @param unit      the units that the measurement is displayed int
         * @param status    a string to display beside the progress bar
         */
        template <typename T, typename U>
        void update(const T& current, const U& total, const std::string& unit = "", const std::string& status = "") {
            update(double(current), double(total), unit, status);
        }

        /**
         * @brief Closes the current progress bar and clears it from the screen.
         * After calling this method this progress bar is no longer valid and calls to update() will throw an error.
         * Calling close() on an invalid progress bar performs no action.
         */
        void close();

    private:
        /// Mutex controlling access to index_list, max_index, and printing to the screen
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        static std::mutex pbar_mutex;

        /// A vector holding the state of every currently active progress bar
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        static std::set<int> index_list;

        /// The next index to use when a new progress bar is created
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
        static int max_index;

        /// Our index into the set of progress bars (index_list), or -1 if we are an invalid progress bar
        int index{-1};

        /// The previous value that was passed in for current
        double previous{0.0};
        /// The current calculated rate in units per second
        double rate{std::numeric_limits<double>::quiet_NaN()};
        /// The start time for this progress bar to use when calculating rates
        std::chrono::steady_clock::time_point start_time{std::chrono::steady_clock::now()};
        /// The last time the update function was called so we can rate limit updates when they are too frequent
        std::chrono::steady_clock::time_point last_update;
        /// The number of updates we have done in the last second so we can handle burst updates
        double updates_per_second{0};

        /// The characters used to make up the progress bar
        static constexpr std::array<const char*, 9> bars = {" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};
    };

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_PROGRESSBAR_HPP
