#include "Dive.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"
#include "extension/behaviour/Script.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/skill/Dive.hpp"


namespace module::skill {

    using extension::Configuration;
    using extension::behaviour::Script;
    using FilteredBall = message::localisation::FilteredBall;
    using message::actuation::BodySequence;
    using message::skill::Dive;

    Dive::Dive(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Dive.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Dive.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.diving_distance_threshold = config["diving_distance_threshold"].as<float>();

            cfg.dive_left.clear();
            for (const auto& s : config["scripts"]["dive_left"].as<std::vector<std::string>>()) {
                cfg.dive_left.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.dive_right.clear();
            for (const auto& s : config["scripts"]["dive_right"].as<std::vector<std::string>>()) {
                cfg.dive_right.push_back(extension::behaviour::ScriptRequest(s));
            }
        });

        // on<Startup>().then({emit<Task>(std::make_unique<Dive>)});

        on<Provide<Dive>, With<FilteredBall>>().then([this](const FilteredBall ball) {
            float yaw_angle        = std::atan2(ball->rBTt.y(), ball->rBTt.x());
            float distance_to_ball = ball->rBTt.head(2).norm();
            if (ball && yaw_angle < 0 && distance_to_ball < cfg.diving_distance_threshold) {
                emit<Script>(std::make_unique<BodySequence>(), cfg.dive_right);
            }
            else {
                emit<Script>(std::make_unique<BodySequence>(), cfg.dive_left);
            }
        });
    }

}  // namespace module::skill
