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

        // Phase message->phase                               = config["phase"].as<
        // Mode message->mode                                 = config["mode"].as<
        // bool message->first_half                           = config["first_half"].as<
        // bool message->kicked_out_by_us                     = config["kicked_out_by_us"].as<
        // google.protobuf.Timestamp message->kicked_out_time = config["kicked_out_time"].as<
        // bool message->our_kick_off                         = config["our_kick_off"].as<
        // google.protobuf.Timestamp message->primary_time    = config["primary_time"].as<
        // google.protobuf.Timestamp message->secondary_time  = config["secondary_time"].as<
        // Team message->team                                 = config["team"].as<
        // Team message->opponent                              = config["opponent"].as<
        // Robot message->self                                 = config["self"].as<
    });
}
}  // namespace module
