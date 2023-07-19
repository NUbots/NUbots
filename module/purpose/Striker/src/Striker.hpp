#ifndef MODULE_PURPOSE_STRIKER_HPP
#define MODULE_PURPOSE_STRIKER_HPP


#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Striker : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Calls Tasks to play soccer normally for a striker
        void play();

        /// @brief Stores configuration values
        struct Config {
            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3f ready_position = Eigen::Vector3f::Zero();
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the Striker reactor.
        explicit Striker(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_STRIKER_HPP
