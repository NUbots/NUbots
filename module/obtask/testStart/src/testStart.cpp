#include "testStart.hpp"

#include "extension/Configuration.hpp"

#include "message/obtask/testRun.proto"
#include "message/obtask/testStart.proto"

namespace module::obtask {

    using extension::Configuration;

    testStart::testStart(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::obtask::testRun;
        using message::obtask::testStart;

        on<Configuration>("testStart.yaml").then([this](const Configuration& config) {
            // Use configuration here from file testStart.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            auto startup = std::make_unique<testStart>();
            log<NUClear::INFO>("startup");
            emit(startup);
        });

        on<Trigger<testRun>>().then([this](const testStart& testStart_msg) {
            auto start = std::make_unique<testStart>();
            if (cfg.num == 10) {
                log<NUClear::INFO>("stop");
                log<NUClear::INFO>(cfg.total);
            }
            else {
                cfg.num++;
                testStart->num = cfg.num;
                cfg.total *= cfg.num;
                log<NUClear::INFO>(cfg.total);
            }
            emit(start);
        });
    }

}  // namespace module::obtask
