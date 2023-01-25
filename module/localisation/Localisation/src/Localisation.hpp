#ifndef MODULE_LOCALISATION_LOCALISATION_HPP
#define MODULE_LOCALISATION_LOCALISATION_HPP

#include <nuclear>

namespace module::localisation {

class Localisation : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the Localisation reactor.
    explicit Localisation(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_LOCALISATION_HPP
