#ifndef MODULE_VORONOI_HPP
#define MODULE_VORONOI_HPP

#include <nuclear>

namespace module {

class Voronoi : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Voronoi reactor.
    explicit Voronoi(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module

#endif  // MODULE_VORONOI_HPP
