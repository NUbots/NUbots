/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#include "ProgressBar.hpp"

#include <algorithm>
#include <cstdio>
#include <fmt/format.h>
#include <sys/ioctl.h>
#include <tuple>
#include <unistd.h>

#include "si_unit.hpp"

namespace utility::support {

    /*
     * This code takes advantage of ANSI character codes in order to manipulate the terminal and choose where we are
     * writing and what text we are writing. Each of these codes starts by printing the ANSI escape sequence `\033[`
     * followed by a command.
     *
     * The commands that we use in this are the following
     * `\033[A`:  This command moves the cursor up a single line
     * `\033[2K`: This command clears the current line that the cursor is on from beginning to end
     * `\r`:      We also use \r (carriage return). This moves the cursor to the beginning of the current line
     */

    /// Move up one line
    constexpr const char* MOVE_UP = "\033[A";
    /// Move down one line
    constexpr const char* MOVE_DOWN = "\n";
    /// Move to the start of the line
    constexpr const char* LINE_START = "\r";
    /// Clear the line
    constexpr const char* CLEAR_LINE = "\033[2K";

    // Start with no active progress bars
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    std::set<int> ProgressBar::index_list = {};
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    int ProgressBar::max_index = 0;

    // Make sure the mutex is constructed
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    std::mutex ProgressBar::pbar_mutex = {};

    /**
     * @brief Used to format the duration for the progress bar in HH:mm::ss format
     *
     * @tparam T the type of the duration to format (should be an std::chrono::duration instance)
     *
     * @param duration the duration to format
     *
     * @return the duration in HH::mm::ss format
     */
    template <typename T>
    std::string duration_format(const T& duration) {
        std::chrono::hours h   = std::chrono::duration_cast<std::chrono::hours>(duration);
        std::chrono::minutes m = std::chrono::duration_cast<std::chrono::minutes>(duration) - h;
        std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(duration) - h - m;

        return fmt::format("{:02d}:{:02d}:{:02d}", h.count(), m.count(), s.count());
    }

    ProgressBar::ProgressBar() {

        /* mutex scope */ {
            std::lock_guard<std::mutex> lock(pbar_mutex);

            // Our index value is always 1 larger than the current largest index value
            index = max_index++;
            index_list.insert(index);

            // Compute the distance between the start of the progress bar list and our current index
            // This will be the nesting level of our progress bar
            const auto nesting = std::distance(index_list.begin(), index_list.find(index));

            // Make sure there is enough space for the new progress bar
            for (int i = 0; i < nesting; ++i) {
                printf("%s%s", MOVE_DOWN, MOVE_DOWN);
            }

            // The progress bar itself just needs one new line so there isn't a blank line below the bar
            printf("%s%s", MOVE_DOWN, MOVE_UP);

            // Go back up to the start of the first progress bar
            for (int i = 0; i < nesting; ++i) {
                printf("%s%s", MOVE_UP, MOVE_UP);
            }
        }
    }

    ProgressBar::ProgressBar(ProgressBar&& other) noexcept {

        /* mutex scope */ {
            std::lock_guard<std::mutex> lock(pbar_mutex);

            // Steal the other progress bars index and invalidate the other progress bar
            index = std::exchange(other.index, -1);
        }
    }

    ProgressBar& ProgressBar::operator=(ProgressBar&& other) noexcept {
        /* mutex scope */ {
            std::lock_guard<std::mutex> lock(pbar_mutex);

            // Steal the other progress bars index and invalidate the other progress bar
            index = std::exchange(other.index, -1);
        }

        return *this;
    }

    ProgressBar::~ProgressBar() {
        // If we still have a valid index then make sure we clean up
        if (index != -1) {
            close();
        }
    }

    void ProgressBar::update(const double& current,
                             const double& total,
                             const std::string& unit,
                             const std::string& status) {
        using namespace std::chrono;  // NOLINT(google-build-using-namespace) fine in function scope

        // Only perform an update if we have a valid index
        if (index == -1) {
            throw std::runtime_error("ProgressBar::update() called on a ProgressBar with an invalid index");
        }

        // Updating too fast is a bad idea, we aim for updating the bar at about 25Hz
        auto now = steady_clock::now();

        // Only update if we changed
        if (previous - current != 0) {

            const bool rate_limit = (updates_per_second < 25 || now - last_update > milliseconds(40));

            // Only update the progress bar if we haven't printed in a while, or if the current update would complete
            // the progress bar
            if (rate_limit || current >= total) {

                double time_delta = duration_cast<duration<double>>(now - last_update).count();

                // Update our transfer rate and if it is NaN or inf (which will also happen at startup)
                // Jump directly to the current instantaneous rate
                rate     = !std::isfinite(rate) ? ((current - previous) / time_delta)
                                                : 0.01 * ((current - previous) / time_delta) + 0.99 * rate;
                previous = current;

                // Update our last update time and our burst counter
                updates_per_second = std::max(0.0, 1 + updates_per_second - std::abs(25 * time_delta));
                last_update        = now;

                // Get the current window size so we can size things appropriately
                struct winsize w {};
                ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

                // Work out the SI units
                auto si_current = si_unit(current);
                auto si_total   = si_unit(total);
                auto si_rate    = si_unit(rate);

                // Get our rate statistics
                std::string time_stats = fmt::format("{:.2f}{}{}/{:.2f}{}{} [{}<{}, {:.2f}{}{}]",
                                                     si_current.first,
                                                     si_current.second,
                                                     unit,
                                                     si_total.first,
                                                     si_total.second,
                                                     unit,
                                                     duration_format(now - start_time),
                                                     duration_format(duration<double>((total - current) / rate)),
                                                     si_rate.first,
                                                     si_rate.second,
                                                     unit.empty() ? "Hz" : fmt::format("{}/s", unit));

                /* mutex scope */ {
                    std::lock_guard<std::mutex> lock(pbar_mutex);

                    // Compute the distance between the start of the progress bar list and our current index
                    // This will be the nesting level of our progress bar
                    const auto nesting = std::distance(index_list.begin(), index_list.find(index));

                    // Jump down the right number of lines based on our nesting
                    for (int i = 0; i < nesting; ++i) {
                        printf("%s%s", MOVE_DOWN, MOVE_DOWN);
                    }

                    // Erase the line, then print our label, space, and the stats
                    printf("%s%s", LINE_START, CLEAR_LINE);
                    printf("%s", status.c_str());
                    for (int i = 0; i < w.ws_col - int(status.size()) - int(time_stats.size()); ++i) {
                        printf(" ");
                    }
                    printf("%s", time_stats.c_str());

                    // Build the progress bar
                    int usable_width = w.ws_col - 2;

                    double full_frac = usable_width * current / total;
                    double remainder = full_frac - std::floor(full_frac);

                    // Full elements
                    int full = std::max(0, std::min(usable_width, int(std::floor(full_frac))));
                    // Empty elements are the remainder except for the partial bar
                    int empty = usable_width - full - 1;

                    // Erase the line and then print the progress bar
                    printf("%s%s%s", MOVE_DOWN, LINE_START, CLEAR_LINE);
                    printf("|");
                    for (int i = 0; i < full; ++i) {
                        printf("%s", bars.back());
                    }
                    if (full < usable_width) {
                        printf("%s", bars[int(bars.size() * remainder)]);
                    }
                    for (int i = 0; i < empty; ++i) {
                        printf("%s", bars.front());
                    }
                    printf("|");

                    // Move back and jump up one (to the start of the progress bar)
                    printf("%s%s", LINE_START, MOVE_UP);

                    // Jump back up the right number of lines based on our nesting
                    for (int i = 0; i < nesting; ++i) {
                        printf("%s%s", MOVE_UP, MOVE_UP);
                    }

                    // Force it to display
                    std::ignore = fflush(stdout);
                }
            }
        }
    }

    void ProgressBar::close() {
        // Only perform a close if we have a valid index
        if (index == -1) {
            return;
        }

        /* mutex scope */ {
            std::lock_guard<std::mutex> lock(pbar_mutex);

            // Clear last progress bar on the screen
            const size_t nesting = index_list.size();
            for (size_t i = 0; i < nesting - 1; ++i) {
                printf("%s%s", MOVE_DOWN, MOVE_DOWN);
            }
            printf("%s%s%s%s%s%s", LINE_START, CLEAR_LINE, MOVE_DOWN, LINE_START, CLEAR_LINE, MOVE_UP);
            for (size_t i = 0; i < nesting - 1; ++i) {
                printf("%s%s", MOVE_UP, MOVE_UP);
            }
            std::ignore = fflush(stdout);

            // Remove the progress bar from the list
            // All progress bars below this one in the list should now shift up 1 position
            index_list.erase(index);

            // Reset our state vector index
            index = -1;
        }
    }

}  // namespace utility::support
