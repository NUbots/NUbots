#ifndef MODULE_OBTASK_TESTNOD_HPP
#define MODULE_OBTASK_TESTNOD_HPP

#include <nuclear>

namespace module::obtask {

class testNod : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the testNod reactor.
    explicit testNod(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::obtask

#endif  // MODULE_OBTASK_TESTNOD_HPP
