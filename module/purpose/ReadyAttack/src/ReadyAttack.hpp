#ifndef MODULE_PURPOSE_READYATTACK_HPP
#define MODULE_PURPOSE_READYATTACK_HPP

#include <nuclear>

namespace module::purpose {

class ReadyAttack : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the ReadyAttack reactor.
    explicit ReadyAttack(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_READYATTACK_HPP
