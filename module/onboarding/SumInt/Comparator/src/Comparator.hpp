#ifndef MODULE_ONBOARDING_COMPARATOR_HPP
#define MODULE_ONBOARDING_COMPARATOR_HPP

#include <nuclear>

namespace module::onboarding::SumInt {

    class Comparator : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        int n;
        int k;

    public:
        /// @brief Called by the powerplant to build and setup the SumInt reactor.
        explicit Comparator(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::onboarding::SumInt

#endif  // MODULE_ONBOARDING_COMPARATOR_HPP
