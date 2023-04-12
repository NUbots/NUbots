#ifndef MODULE_PURPOSE_DEFENDER_HPP
#define MODULE_PURPOSE_DEFENDER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

class Defender : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Defender reactor.
        explicit Defender(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFENDER_HPP
