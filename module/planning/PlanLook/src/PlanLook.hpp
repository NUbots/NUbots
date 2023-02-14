#ifndef MODULE_PLANNING_PLANLOOK_HPP
#define MODULE_PLANNING_PLANLOOK_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanLook : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            NUClear::clock::duration ball_search_timeout{};
            NUClear::clock::duration goal_search_timeout{};
            NUClear::clock::duration field_search_timeout{};

            float search_fixation_time = 0.0f;
            std::vector<Eigen::Vector2d> search_positions{};
        } cfg;

        /// @brief Time since last doing a full search
        NUClear::clock::time_point last_field_search = NUClear::clock::now();

        /// @brief  Time since last search position transition
        NUClear::clock::time_point search_last_moved = NUClear::clock::now();

        /// @brief Current index in the list of search positions
        long unsigned int search_idx = 0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanLook reactor.
        explicit PlanLook(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANLOOK_HPP
