#include "K1Servos.hpp"

#include "message/actuation/K1Servos.hpp"
#include "message/booster/BoosterLowCmd.hpp"

namespace module::actuation {

    using message::booster::BoosterLowCmd;
    using K1ServosTask = message::actuation::K1Servos;

    K1Servos::K1Servos(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Provide<K1ServosTask>, Priority::HIGH>().then([this](const K1ServosTask& servos) {
            emit(std::make_unique<BoosterLowCmd>(servos.command));
        });
    }

}  // namespace module::actuation
