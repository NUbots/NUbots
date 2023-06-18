#ifndef MODULE_STRATEGY_WALKINSIDEBOUNDEDBOX_HPP
#define MODULE_STRATEGY_WALKINSIDEBOUNDEDBOX_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkInsideBoundedBox : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Bounded box of region to defend the ball - x min, x max, y max, y min
            Eigen::Vector4f bounded_region        = Eigen::Vector4f::Zero();
            Eigen::Vector2d bounded_region_xy_min = Eigen::Vector2d::Zero();
            Eigen::Vector2d bounded_region_xy_max = Eigen::Vector2d::Zero();

        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Defend reactor.
        explicit WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKINSIDEBOUNDED_HPP
