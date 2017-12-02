#include <random>

#include "RandomHead.h"

#include "extension/Configuration.h"

#include "message/motion/HeadCommand.h"

namespace module {
namespace behaviour {
    namespace planning {

        using extension::Configuration;

        using message::motion::HeadCommand;

        RandomHead::RandomHead(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), rd(), gen(rd()) {

            on<Configuration>("RandomHead.yaml").then([this](const Configuration& config) {
                freq        = config["frequency"].as<float>();
                pitch_limit = config["pitch_limit"].as<float>();
                yaw_limit   = config["yaw_limit"].as<float>();
            });

            on<Every<2, Per<std::chrono::seconds>>>().then([this] {
                std::uniform_real_distribution<> dis(0.0, 1.0);
                std::uniform_real_distribution<float> yaw(-yaw_limit, yaw_limit);
                std::uniform_real_distribution<float> pitch(-pitch_limit, pitch_limit);

                if (dis(gen) < freq) {
                    emit(std::make_unique<HeadCommand>(yaw(gen), pitch(gen), true));
                }
            });
        }
    }  // namespace planning
}  // namespace behaviour
}  // namespace module
