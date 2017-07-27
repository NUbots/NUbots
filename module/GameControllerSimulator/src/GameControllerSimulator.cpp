#include "GameControllerSimulator.h"

#include "extension/Configuration.h"
#include "message/input/GameState.h"

namespace module {

using extension::Configuration;
using message::input::GameState;

GameControllerSimulator::GameControllerSimulator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

    on<Configuration>("GameControllerSimulator.yaml").then([this](const Configuration& config) {
        // Use configuration here from file GameControllerSimulator.yaml

        Phase phase                               = 1;
        Mode mode                                 = 2;
        bool first_half                           = 3;
        bool kicked_out_by_us                     = 4;
        google.protobuf.Timestamp kicked_out_time = 5;
        bool our_kick_off                         = 6;
        google.protobuf.Timestamp primary_time    = 7;
        google.protobuf.Timestamp secondary_time  = 8;
        Team team                                 = 9;
        Team opponent                             = 10;
        Robot self                                = 11;
    });
}
}
