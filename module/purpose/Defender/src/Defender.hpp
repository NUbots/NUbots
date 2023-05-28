#ifndef MODULE_PURPOSE_DEFENDER_HPP
#define MODULE_PURPOSE_DEFENDER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

class Defender : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3f ready_position = Eigen::Vector3f::Zero();

        } cfg;

        enum field_position {left_wing, right_wing, goal_box, centre_cirlce};

        /// @brief Calls Tasks to play soccer normally for a defender
        void play();

    public:
        /// @brief Called by the powerplant to build and setup the Defender reactor.
        explicit Defender(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFENDER_HPP
