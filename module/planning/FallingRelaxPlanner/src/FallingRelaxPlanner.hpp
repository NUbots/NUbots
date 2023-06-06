#ifndef MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
#define MODULE_PLANNING_FALLINGRELAXPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    template <typename Scalar>
    Scalar smooth(Scalar value, Scalar new_value, Scalar alpha) {
        return alpha * value + (1.0 - alpha) * new_value;
    }

    class FallingRelaxPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            struct Levels {
                /// @brief The mean value of this sensor to subtract
                float mean = 0.0f;
                /// @brief The threshold for this sensor to be considered unstable
                float unstable = 0.0f;
                /// @brief The threshold for this sensor to be considered falling
                float falling = 0.0f;
                /// @brief The smoothing factor for this sensor
                float smoothing = 0.0f;
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
        float gyro_mag = 0.0f;
        /// @brief the current smoothed value of the accelerometer magnitude
        float acc_mag = 0.0f;
        /// @brief the current smoothed value of the accelerometer angle from upright
        float acc_angle = 0.0f;
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
