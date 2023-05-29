#ifndef MODULE_STRATEGY_LOCALISE_HPP
#define MODULE_STRATEGY_LOCALISE_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class Localise : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The confidence value field localisation needs to be below to not request standing still.
            float confidence_threshold = 0;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Localise reactor.
        explicit Localise(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_LOCALISE_HPP
