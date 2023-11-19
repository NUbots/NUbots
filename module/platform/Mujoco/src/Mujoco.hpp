#ifndef MODULE_PLATFORM_MUJOCO_HPP
#define MODULE_PLATFORM_MUJOCO_HPP

#include <nuclear>

namespace module::platform {

class Mujoco : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Mujoco reactor.
    explicit Mujoco(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MUJOCO_HPP
