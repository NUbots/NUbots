#include "testNod.hpp"

#include "extension/Configuration.hpp"

#include "message/obtask/testRun.hpp"

namespace module::obtask {

    using extension::Configuration;

    testNod::testNod(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {


        using message::obtask::testRun;

        on<Configuration>("testNod.yaml").then([this](const Configuration& config) {
            // Use configuration here from file testNod.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Vibe
        });

        on<Trigger<testRun>>().then([this](const testRun& run_msg) {
            auto nod = std::make_unique<testRun>();
            if (run_msg.num == 10) {
                // send nod msg to other module
                log<NUClear::INFO>("nod");
                emit(nod);
            }
            else {
                // emit(run);
            }
        });
    }

}  // namespace module::obtask
