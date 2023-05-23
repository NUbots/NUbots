#ifndef MODULE_SKILL_GETUP_HPP
#define MODULE_SKILL_GETUP_HPP

#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::skill {

    class GetUp : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Script sequence to run when getting from lying on the front to standing
            std::vector<std::string> getup_front;
            /// @brief Script sequence to run when getting from lying on the back to standing
            std::vector<std::string> getup_back;
            /// @brief Script sequence to run when getting from lying on the right to standing
            std::vector<std::string> getup_right;
            /// @brief Script sequence to run when getting from lying on the left to standing
            std::vector<std::string> getup_left;
            /// @brief Script sequence to run when already upright
            std::vector<std::string> getup_upright;
            /// @brief Script sequence to run when told to get up while upside down
            std::vector<std::string> getup_upside_down;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the GetUp reactor.
        explicit GetUp(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_GETUP_HPP
