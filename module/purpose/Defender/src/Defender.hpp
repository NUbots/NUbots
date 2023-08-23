#ifndef MODULE_PURPOSE_DEFENDER_HPP
#define MODULE_PURPOSE_DEFENDER_HPP


#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Defender : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Calls Tasks to play soccer normally for a defender
        void play();

        /// @brief Stores configuration values
        struct Config {
            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3d ready_position = Eigen::Vector3d::Zero();
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the defender reactor.
        explicit Defender(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFENDER_HPP
