/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef MODULE_MOTION_QUINTICWALK_SPLINECONTAINER_HPP
#define MODULE_MOTION_QUINTICWALK_SPLINECONTAINER_HPP

#include <fmt/format.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "Spline.hpp"

namespace module {
namespace motion {
    namespace engine {

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
            inline size_t size() const {
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
             * Return true if given spline name is contained
             */
            inline bool exist(const U& name) const {
                return container.count(name) > 0;
            }

            /**
             * Access to given named spline
             */
            inline const T& get(const U& name) const {
                if (container.count(name) == 0) {
                    throw std::logic_error(fmt::format("SplineContainer invalid name: {}", std::string(name)));
                }
                return container.at(name);
            }

            inline T& get(const U& name) {
                if (container.count(name) == 0) {
                    throw std::logic_error(fmt::format("SplineContainer invalid name: {}", std::string(name)));
                }
                return container.at(name);
            }

            /**
             * Access to internal map container
             */
            const Map& get() const {
                return container;
            }

            Map& get() {
                return container;
            }

            /**
             * Returns all time points where a point in any spline exists.
             */
            std::vector<Scalar> getTimes() {
                std::set<Scalar> times;
                std::vector<Scalar> times_sorted;
                // go trough all splines
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
            Scalar min() const {
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

            Scalar max() const {
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

            void importData(const std::string& file_name) {
                std::ifstream file(file_name);
                if (!file.is_open()) {
                    throw std::runtime_error(fmt::format("SplineContainer unable to read file: ", file_name));
                }

                bool isParseError;
                while (file.good()) {
                    isParseError = true;
                    // Skip name delimitor
                    if (file.peek() != '\'') break;
                    file.ignore();
                    if (!file.good()) break;
                    // Parse spline name
                    char name[256];
                    file.getline(name, 256, '\'');
                    // Import founded spline
                    add(U(name));
                    if (!file.good()) break;
                    container.at(U(name)).importData(file);
                    isParseError = false;
                    // Skip end line
                    while (file.peek() == ' ' || file.peek() == '\n') {
                        if (!file.good()) break;
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
            Map container;
        };

    }  // namespace engine
}  // namespace motion
}  // namespace module

#endif  // SPLINECONTAINER_HPP
