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

#include <cstdio>
#include <fmt/format.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace utility::support {

    /**
     * @brief Converts the provided value into an SI representation, returning the SI prefix and a value reduced to the
     * SI unit where it is >=1 <=1000
     *
     * @param v the value to convert into an SI prefixed representation
     *
     * @return a pair containing the reduced value, and the SI prefix for the unit
     */
    std::pair<double, std::string> si_unit(const double& v) {

        // Work out our SI index between -8 and 8
        int si_index = std::max(-8L, std::min(8L, int64_t(std::log10(std::abs(v)) / 3)));

        switch (si_index) {
            case 8: return std::make_pair(v * 1e-24, "Y");  // yotta
            case 7: return std::make_pair(v * 1e-21, "Z");  // zetta
            case 6: return std::make_pair(v * 1e-18, "E");  // exa
            case 5: return std::make_pair(v * 1e-15, "P");  // peta
            case 4: return std::make_pair(v * 1e-12, "T");  // tera
            case 3: return std::make_pair(v * 1e-9, "G");   // giga
            case 2: return std::make_pair(v * 1e-6, "M");   // mega
            case 1: return std::make_pair(v * 1e-3, "k");   // kilo
            case 0: return std::make_pair(v, "");           //
            case -1: return std::make_pair(v * 1e3, "m");   // milli
            case -2: return std::make_pair(v * 1e6, "μ");   // micro
            case -3: return std::make_pair(v * 1e9, "n");   // nano
            case -4: return std::make_pair(v * 1e12, "p");  // pico
            case -5: return std::make_pair(v * 1e15, "f");  // femto
            case -6: return std::make_pair(v * 1e18, "a");  // atto
            case -7: return std::make_pair(v * 1e21, "z");  // zepto
            case -8: return std::make_pair(v * 1e24, "y");  // yocto
            default: return std::make_pair(v, "");
        }
    }

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

    ProgressBar::ProgressBar(std::string unit)
        : unit(std::move(unit))
        , start_time(std::chrono::steady_clock::now())
        , last_update(std::chrono::seconds(0))
        , bars({" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"}) {
        printf("Progress\nbar!");
    }

    void ProgressBar::update(const double& current, const double& total, const std::string& status) {
        using namespace std::chrono;  // NOLINT(google-build-using-namespace) fine in function scope

        // Updating too fast is a bad idea, we aim for updating the bar at about 25Hz
        auto now = steady_clock::now();

        // Only update if we changed
        if (previous - current != 0) {
            // NOLINTNEXTLINE(modernize-use-nullptr) there are no pointers here TODO: updates to clang for fix.
            if (updates_per_second < 25 || now - last_update > milliseconds(40)) {

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

                // Erase our line and the line above
                printf("\r\033[2K\033[A\r\033[2K");

                // Work out the SI units
                auto si_current = si_unit(current);
                auto si_total   = si_unit(total);
                auto si_rate    = si_unit(rate);

                // Get our rate statistics
                std::string time_stats = fmt::format("{:.2f}{}{}/{:.2f}{}{} [{}<{}, {:.2f}{}{}]\n",
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

                // Print our label, space, and the stats
                printf("%s", status.c_str());
                for (int i = 0; i < w.ws_col - int(status.size()) - int(time_stats.size()) + 1; ++i) {
                    printf(" ");
                }
                printf("%s", time_stats.c_str());

                // Build the progress bar
                double usable_width      = w.ws_col - 2;
                double frac              = current / total;
                double screen_full_frac  = frac * usable_width;
                double screen_empty_frac = usable_width - screen_full_frac;

                // Print the progress bar
                printf("|");
                for (int i = 0; i < int(screen_full_frac); ++i) {
                    printf("%s", bars.back().c_str());
                }
                printf("%s", bars[int(bars.size() * (screen_full_frac - std::floor(screen_full_frac)))].c_str());
                for (int i = 0; i < int(screen_empty_frac); ++i) {
                    printf("%s", bars.front().c_str());
                }
                printf("|");

                // Force it to display
                fflush(stdout);
            }
        }
    }

}  // namespace utility::support
