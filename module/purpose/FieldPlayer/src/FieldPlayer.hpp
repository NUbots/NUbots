#ifndef MODULE_FIELDPLAYER_HPP
#define MODULE_FIELDPLAYER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class FieldPlayer : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The distance from the ball to consider it in possession
            double ball_threshold = 0.0;
            /// @brief The distance from a robot to consider it equidistant, to consider error
            double equidistant_threshold = 0.0;
            /// @brief The distance from the ball to consider it moved from the center position and in play for kickoff
            double ball_off_center_threshold = 0.0;
            /// @brief The offset to the center circle for the ready position.
            /// Avoids being in the center circle during another team's kickoff
            double center_circle_offset = 0.0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FieldPlayer reactor.
        explicit FieldPlayer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_FIELDPLAYER_HPP
