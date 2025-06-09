#ifndef MODULE_PURPOSE_READYATTACK_HPP
#define MODULE_PURPOSE_READYATTACK_HPP

#include <nuclear>

namespace module::purpose {

    class ReadyAttack : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The offset to the center circle for the ready position.
            /// Avoids being in the center circle during another team's kickoff
            double center_circle_offset = 0.0;
            /// @brief The distance away from the ball when defending during a penalty state
            double penalty_defend_distance = 0.0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the ReadyAttack reactor.
        explicit ReadyAttack(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_READYATTACK_HPP
