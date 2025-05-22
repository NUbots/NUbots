#ifndef MODULE_SOCCERNEW_HPP
#define MODULE_SOCCERNEW_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module {

    struct EnableIdle {};
    struct DisableIdle {};

    class SoccerNew : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the SoccerNew reactor.
        explicit SoccerNew(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module

#endif  // MODULE_SOCCERNEW_HPP
