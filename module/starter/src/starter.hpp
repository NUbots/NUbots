#ifndef MODULE_STARTER_HPP
#define MODULE_STARTER_HPP

#include <nuclear>

namespace module {

class starter : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the starter reactor.
    explicit starter(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_STARTER_HPP
