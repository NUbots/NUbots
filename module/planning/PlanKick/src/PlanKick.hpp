#ifndef MODULE_PLANNING_PLANKICK_HPP
#define MODULE_PLANNING_PLANKICK_HPP

#include <nuclear>

namespace module::planning {

class PlanKick : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the PlanKick reactor.
    explicit PlanKick(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANKICK_HPP
