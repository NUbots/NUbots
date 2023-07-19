#include "FallRecovery.hpp"

#include "extension/Behaviour.hpp"

#include "message/planning/GetUpWhenFallen.hpp"
#include "message/planning/RelaxWhenFalling.hpp"
#include "message/strategy/FallRecovery.hpp"

namespace module::strategy {

    using message::planning::GetUpWhenFallen;
    using message::planning::RelaxWhenFalling;
    using FallRecoveryTask = message::strategy::FallRecovery;

    FallRecovery::FallRecovery(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Provide<FallRecoveryTask>>().then([this] {
            // Plan to relax when falling and get up when on the ground
            // We set the get up priority higher than falling so that relax won't take over while we are getting up
            // emit<Task>(std::make_unique<RelaxWhenFalling>(), 1); // commented out until it stops causing issues
            emit<Task>(std::make_unique<GetUpWhenFallen>(), 2);
        });
    }

}  // namespace module::strategy
