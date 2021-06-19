#ifndef UTILITY_SUPPORT_PROGRESSBAR_HPP
#define UTILITY_SUPPORT_PROGRESSBAR_HPP

#include <chrono>
#include <limits>
#include <string>
#include <vector>

namespace utility::support {

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
        ProgressBar(std::string unit = "iteration");

        /**
         * @brief Updates the progress bar using the new values for current and total
         *
         * @param current   the current number of units that have been completed
         * @param total     the total number of units to be completed
         * @param status    a string to display besides the progress bar
         */
        void update(const double& current, const double& total, const std::string& status = "");

        /**
         * @brief Updates the progress bar using the new values for current and total with types that are castable to
         * double
         *
         * @tparam T    a type that is castable to double to be used with update(double, double)
         * @tparam U    a type that is castable to double to be used with update(double, double)
         *
         * @param current   the current number of units that have been completed
         * @param total     the total number of units to be completed
         * @param status    a string to display beside the progress bar
         */
        template <typename T, typename U>
        void update(const T& current, const U& total, const std::string& status = "") {
            update(double(current), double(total), status);
        }

    private:
        /// The previous value that was passed in for current
        double previous{0};
        /// The current calculated rate in units per second
        double rate{std::numeric_limits<double>::quiet_NaN()};
        /// The unit to display rates and progress in
        std::string unit;
        /// The start time for this progress bar to use when calculating rates
        std::chrono::steady_clock::time_point start_time;
        /// The last time the update function was called so we can rate limit updates when they are too frequent
        std::chrono::steady_clock::time_point last_update;
        /// The number of updates we have done in the last second so we can handle burst updates
        double updates_per_second{0};
        /// The characters used to make up the progress bar
        std::vector<std::string> bars;
    };

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_PROGRESSBAR_HPP
