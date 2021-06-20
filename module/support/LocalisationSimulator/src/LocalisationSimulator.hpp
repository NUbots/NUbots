#ifndef MODULE_SUPPORT_LOCALISATIONSIMULATOR_HPP
#define MODULE_SUPPORT_LOCALISATIONSIMULATOR_HPP

#include <nuclear>

namespace module::support {

    class LocalisationSimulator : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the LocalisationSimulator reactor.
        explicit LocalisationSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support

#endif  // MODULE_SUPPORT_LOCALISATIONSIMULATOR_HPP
