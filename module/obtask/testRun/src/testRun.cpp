#include "testRun.hpp"

#include "extension/Configuration.hpp"

#include "message/obtask/testRun.hpp"
#include "message/obtask/testStart.hpp"

namespace module::obtask {

    using extension::Configuration;

    testRun::testRun(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::obtask::testRun;
        using message::obtask::testStart;

        on<Configuration>("testRun.yaml").then([this](const Configuration& config) {
            // Use configuration here from file testRun.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Vibe
        });

        on<Trigger<testStart>>().then([this](const testStart& start_msg) {
            auto run = std::make_unique<testRun>();
            if (start_msg.num == 10) {
                // send nod msg to other module
                run->num = start_msg.num;
                log<NUClear::INFO>("nod");
                emit(run);
            }
            else {
                // log<NUClear::INFO>(start_msg.num);
                emit(run);
            }
        });
    }

}  // namespace module::obtask
