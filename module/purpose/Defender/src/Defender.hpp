#ifndef MODULE_PURPOSE_DEFENDER_HPP
#define MODULE_PURPOSE_DEFENDER_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Defender : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3f left_ready_position  = Eigen::Vector3f::Zero();
            Eigen::Vector3f right_ready_position = Eigen::Vector3f::Zero();

            std::vector<Eigen::Vector3f> left_defender_bounding_box = {Eigen::Vector3f::Zero(),
                                                                       Eigen::Vector3f::Zero(),
                                                                       Eigen::Vector3f::Zero(),
                                                                       Eigen::Vector3f::Zero()};

            std::vector<Eigen::Vector3f> right_defender_bounding_box = {Eigen::Vector3f::Zero(),
                                                                        Eigen::Vector3f::Zero(),
                                                                        Eigen::Vector3f::Zero(),
                                                                        Eigen::Vector3f::Zero()};

        } cfg;

        bool is_left_defender;

        /// @brief Calls Tasks to play soccer normally for a defender
        void play();

    public:
        /// @brief Called by the powerplant to build and setup the Defender reactor.
        explicit Defender(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFENDER_HPP
