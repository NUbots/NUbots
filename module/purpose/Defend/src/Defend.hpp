#ifndef MODULE_PURPOSE_DEFEND_HPP
#define MODULE_PURPOSE_DEFEND_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

class Defend : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Defend reactor.
    explicit Defend(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DEFEND_HPP
