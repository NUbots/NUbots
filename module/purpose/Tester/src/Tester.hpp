#ifndef MODULE_PURPOSE_TESTER_HPP
#define MODULE_PURPOSE_TESTER_HPP

#include <nuclear>

namespace module::purpose {

class Tester : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Tester reactor.
    explicit Tester(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TESTER_HPP
