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

/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_SPLINECONTAINER_HPP
#define UTILITY_MOTION_SPLINES_SPLINECONTAINER_HPP

#include <algorithm>
#include <fmt/format.h>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "Spline.hpp"

namespace utility::motion::splines {

    /**
     * SplineContainer
     *
     * Wrapper for map of generic splines types indexed by string name
     * Implementation of implort/export from files
     */
    template <typename T, typename U, typename Scalar>
    class SplineContainer {
    public:
        using Map = std::map<U, T>;

        /**
         * Return the number of contained splines
         */
        [[nodiscard]] constexpr size_t size() const {
            return container.size();
        }

        /**
         * Add an empty spline with given name.
         * Variadic arguments allow to pass parameters to spline constructor.
         */
        template <typename... Args>
        inline void add(const U& name, Args... args) {
            if (container.count(name) != 0) {
                throw std::logic_error("SplineContainer spline already added");
            }
            container[name] = T(args...);
        }

        /**
         * Reset spline to be empty
         */
        inline void reset() {
            auto it = container.begin();

            for (size_t i = 0; i < size(); i++) {
                container[it->first].reset();
                it++;
            }
        }

        /**
         * Return true if given spline name is contained
         */
        [[nodiscard]] constexpr bool exist(const U& name) const {
            return container.count(name) > 0;
        }

        /**
         * Access to given named spline
         */
        [[nodiscard]] constexpr const T& get(const U& name) const {
            if (container.count(name) == 0) {
                throw std::logic_error(fmt::format("SplineContainer invalid name: {}", std::string(name)));
            }
            return container.at(name);
        }

        [[nodiscard]] constexpr T& get(const U& name) {
            if (container.count(name) == 0) {
                throw std::logic_error(fmt::format("SplineContainer invalid name: {}", std::string(name)));
            }
            return container.at(name);
        }

        /**
         * Access to internal map container
         */
        [[nodiscard]] constexpr const Map& get() const {
            return container;
        }

        [[nodiscard]] Map& get() {
            return container;
        }

        /**
         * Returns all time points where a point in any spline exists.
         */
        [[nodiscard]] std::vector<Scalar> getTimes() {
            std::set<Scalar> times;
            std::vector<Scalar> times_sorted;
            // go through all splines
            for (const auto& sp : container) {
                // go trough all points of the spline
                for (typename SmoothSpline<Scalar>::Point point : sp.second.points()) {
                    times.insert(point.time);
                }
            }
            // insert set into vector
            times_sorted.insert(times_sorted.end(), times.begin(), times.end());
            std::sort(times_sorted.begin(), times_sorted.end());
            return times_sorted;
        }

        /**
         * Return minimum and maximum abscisse values of all registered splines parts
         */
        [[nodiscard]] constexpr Scalar min() const {
            if (container.size() == 0) {
                return 0.0;
            }
            bool isFirst = true;
            Scalar m     = 0.0;
            for (const auto& sp : container) {
                if (isFirst || m > sp.second.min()) {
                    m       = sp.second.min();
                    isFirst = false;
                }
            }
            return m;
        }

        [[nodiscard]] constexpr Scalar max() const {
            if (container.size() == 0) {
                return 0.0;
            }
            bool isFirst = true;
            Scalar m     = 0.0;
            for (const auto& sp : container) {
                if (isFirst || m < sp.second.max()) {
                    m       = sp.second.max();
                    isFirst = false;
                }
            }
            return m;
        }

        /**
         * Export to and Import from given file name in "spline" CSV format prefixed with spline name
         */
        void exportData(const std::string& file_name) const {
            if (container.size() == 0) {
                throw std::logic_error("SplineContainer empty");
            }

            std::ofstream file(file_name);
            if (!file.is_open()) {
                throw std::runtime_error(fmt::format("SplineContainer unable to write file: ", file_name));
            }

            for (const auto& sp : container) {
                file << "'" << sp.first << "' ";
                sp.second.exportData(file);
            }

            file.close();
        }

        void importData(const std::string& file_name) const {
            std::ifstream file(file_name);
            if (!file.is_open()) {
                throw std::runtime_error(fmt::format("SplineContainer unable to read file: ", file_name));
            }

            bool isParseError = true;
            while (file.good()) {
                isParseError = true;
                // Skip name delimitor
                if (file.peek() != '\'') {
                    break;
                }
                file.ignore();
                if (!file.good()) {
                    break;
                }
                // Parse spline name
                char name[256];
                file.getline(name, 256, '\'');
                // Import founded spline
                add(U(name));
                if (!file.good()) {
                    break;
                }
                container.at(U(name)).importData(file);
                isParseError = false;
                // Skip end line
                while (file.peek() == ' ' || file.peek() == '\n') {
                    if (!file.good()) {
                        break;
                    }
                    file.ignore();
                }
            }
            if (isParseError) {
                throw std::logic_error("SplineContainer invalid input format");
            }

            file.close();
        }

    private:
        /**
         * Spline container indexed by their name
         */
        Map container{};
    };

}  // namespace utility::motion::splines
#endif
