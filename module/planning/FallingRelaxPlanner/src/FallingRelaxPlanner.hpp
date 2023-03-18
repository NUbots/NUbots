#ifndef MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
#define MODULE_PLANNING_FALLINGRELAXPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class FallingRelaxPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            struct Levels {
                /// @brief The mean value of this sensor to subtract
                double mean;
                /// @brief The threshold for this sensor to be considered unstable
                double unstable;
                /// @brief The threshold for this sensor to be considered falling
                double falling;
                /// @brief The smoothing factor for this sensor
                double smoothing;
            };

            /// @brief The configuration for the gyroscope magnitude check
            Levels gyro_mag;
            /// @brief The configuration for the accelerometer magnitude check
            Levels acc_mag;
            /// @brief The configuration for the accelerometer angle check
            Levels acc_angle;
            /// @brief The script to run when falling
            std::string fall_script;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FallingRelaxPlanner reactor.
        explicit FallingRelaxPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief the current smoothed value of the gyroscope magnitude
        double gyro_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer magnitude
        double acc_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer angle from upright
        double acc_angle = 0.0;
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
