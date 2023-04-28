#ifndef MODULE_SUPPORT_VORONOI_HPP
#define MODULE_SUPPORT_VORONOI_HPP

#include <nuclear>

namespace module::support {

class Voronoi : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Voronoi reactor.
    explicit Voronoi(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::support

#endif  // MODULE_SUPPORT_VORONOI_HPP
