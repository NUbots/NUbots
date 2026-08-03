#ifndef MODULE_TESTMODULE_HPP
#define MODULE_TESTMODULE_HPP

#include <nuclear>

namespace module {

class TestModule : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
        /// @brief How much to increment the count by each time a new Ping or Pong message is emitted
        int increment = 0;
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the TestModule reactor.
    explicit TestModule(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_TESTMODULE_HPP
