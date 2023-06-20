#ifndef MODULE_SKILL_SPLINEKICK_HPP
#define MODULE_SKILL_SPLINEKICK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/skill/Kick.hpp"

namespace module::skill {

    class SplineKick : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 200;

        struct Config {
            /// @brief Stores the gains for each servo
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states{};

            /// @brief Desired arm positions while walking
            std::vector<std::pair<utility::input::ServoID, double>> arm_positions{};
        } cfg;

        /// @brief Last time we updated the walk engine
        NUClear::clock::time_point last_update_time{};

        /// @brief Walk engine, generates swing foot and torso trajectories for walk velocity target

        utility::skill::KickOptions<double> kick_options;
        utility::skill::KickGenerator<double> kick_generator{};

    public:
        /// @brief Called by the powerplant to build and setup the SplineKick reactor.
        explicit SplineKick(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SPLINEKICK_HPP
