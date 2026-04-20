#ifndef MODULE_ONBOARDING_PONG_HPP
#define MODULE_ONBOARDING_PONG_HPP

#include <nuclear>

namespace module::onboarding {

    class Pong : public NUClear::Reactor {
    private:
        unsigned int counter = 0;  // keep track of how many pongs sent

    public:
        explicit Pong(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::onboarding

#endif  // MODULE_ONBOARDING_PONG_HPP
