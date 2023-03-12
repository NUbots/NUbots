#ifndef MODULE_PURPOSE_STRIKER_HPP
#define MODULE_PURPOSE_STRIKER_HPP


#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Striker : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The position of the robot to walk to in ready, relative to the field
            Eigen::Vector3d rRFf = Eigen::Vector3d::Zero();
        } cfg;

        /// @brief Calls Tasks to play soccer normally for a striker
        void play();

    public:
        /// @brief Called by the powerplant to build and setup the Striker reactor.
        explicit Striker(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_STRIKER_HPP
