#ifndef MODULE_PURPOSE_PENALTYSHOOTOUT_HPP
#define MODULE_PURPOSE_PENALTYSHOOTOUT_HPP

#include <nuclear>

namespace module::purpose {

class PenaltyShootout : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the PenaltyShootout reactor.
    explicit PenaltyShootout(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_PENALTYSHOOTOUT_HPP
