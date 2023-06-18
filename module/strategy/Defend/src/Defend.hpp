#ifndef MODULE_STRATEGY_DEFEND_HPP
#define MODULE_STRATEGY_DEFEND_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class Defend : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Region to defend the ball
            Eigen::Vector4f defending_region = Eigen::Vector4f::Zero();

        } cfg;

        // Robot - Defending position on field
        Eigen::Vector3f rDFf = Eigen::Vector3f::Zero();

        // Thge distance of the robot from the ball
        double robot_distance_to_ball = 0.0;

    public:
        /// @brief Called by the powerplant to build and setup the Defend reactor.
        explicit Defend(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_DEFEND_HPP
