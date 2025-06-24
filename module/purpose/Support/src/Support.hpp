#ifndef MODULE_PURPOSE_SUPPORT_HPP
#define MODULE_PURPOSE_SUPPORT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

class Support : public ::extension::behaviour::BehaviourReactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Support reactor.
    explicit Support(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SUPPORT_HPP
