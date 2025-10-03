#ifndef MODULE_CHECKER_HPP
#define MODULE_CHECKER_HPP

#include <nuclear>

namespace module {

class checker : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the checker reactor.
    explicit checker(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_CHECKER_HPP
