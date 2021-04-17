#include "LocalisationSimulator.hpp"

#include "extension/Configuration.hpp"

#include "message/localisation/Field.hpp"

namespace module::support {

using extension::Configuration;
using message::localisation::Field;

LocalisationSimulator::LocalisationSimulator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)), config{} {

    on<Configuration>("LocalisationSimulator.yaml").then([this](const Configuration& cfg) {
        // Use configuration here from file LocalisationSimulator.yaml

        // clang-format off
        auto lvl = cfg["log_level"].as<std::string>();
        if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
        else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
        else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
        else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
        else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
        else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
        // clang-format on
    });

    on<Every<3, Per<std::chrono::seconds>>, With<Field>>().then("Heart Beat",
                                                                [this](const Field& f) { log(f.position); });
}

}  // namespace module::support
