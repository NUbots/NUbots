#ifndef MODULE_MOTION_KICKSCRIPT_HPP
#define MODULE_MOTION_KICKSCRIPT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::motion {

class KickScript : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the KickScript reactor.
    explicit KickScript(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::motion

#endif  // MODULE_MOTION_KICKSCRIPT_HPP
