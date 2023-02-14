#ifndef MODULE_PLANLOOK_HPP
#define MODULE_PLANLOOK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module {

    class PlanLook : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            float ball_search_timeout  = 0.0;
            float goal_search_timeout  = 0.0;
            float field_search_timeout = 0.0;
            float fixation_time        = 0.0f;
            std::vector<Eigen::Vector2d> search_positions{};
        } cfg;

        /// @brief Time since last doing a full search
        NUClear::clock::time_point last_field_search = NUClear::clock::now();

        /// @brief  Time since last search position transition
        NUClear::clock::time_point search_last_moved = NUClear::clock::now();

        /// @brief Current index in the list of search positions
        int search_idx = 0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanLook reactor.
        explicit PlanLook(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module

#endif  // MODULE_PLANLOOK_HPP
