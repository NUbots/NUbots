#include "Servos.hpp"

#include "message/actuation/Servos.hpp"
#include "message/booster/BoosterLowCmd.hpp"

namespace module::actuation {

    using message::booster::BoosterLowCmd;
    using ServosTask = message::actuation::Servos;

    Servos::Servos(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Provide<ServosTask>, Priority::HIGH>().then(
            [this](const ServosTask& servos) { emit(std::make_unique<BoosterLowCmd>(servos.command)); });
    }

}  // namespace module::actuation
