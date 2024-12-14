#ifndef MODULE_SKILL_SEMIDYNAMICGETUP_HPP
#define MODULE_SKILL_SEMIDYNAMICGETUP_HPP

#include <nuclear>

namespace module::skill {

class SemiDynamicGetup : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the SemiDynamicGetup reactor.
    explicit SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::skill

#endif  // MODULE_SKILL_SEMIDYNAMICGETUP_HPP
