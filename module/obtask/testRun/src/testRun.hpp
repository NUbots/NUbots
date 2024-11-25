#ifndef MODULE_OBTASK_TESTRUN_HPP
#define MODULE_OBTASK_TESTRUN_HPP

#include <nuclear>

namespace module::obtask {

class testRun : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the testRun reactor.
    explicit testRun(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::obtask

#endif  // MODULE_OBTASK_TESTRUN_HPP
