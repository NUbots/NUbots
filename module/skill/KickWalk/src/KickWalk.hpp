#ifndef MODULE_SKILL_KICKWALK_HPP
#define MODULE_SKILL_KICKWALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::skill {

class KickWalk : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the KickWalk reactor.
    explicit KickWalk(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::skill

#endif  // MODULE_SKILL_KICKWALK_HPP
