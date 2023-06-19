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
            float bounded_region_x_min = 0.0;
            float bounded_region_x_max = 0.0;
            float bounded_region_y_min = 0.0;
            float bounded_region_y_max = 0.0;

        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Defend reactor.
        explicit WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKINSIDEBOUNDED_HPP
