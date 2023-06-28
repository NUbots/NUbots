#include "Tester.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/KickTo.hpp"
#include "message/planning/LookAround.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::strategy::AlignBallToGoal;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;

    using message::planning::KickTo;
    using message::planning::LookAround;

    using utility::support::Expression;

    Tester::Tester(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Tester.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Tester.yaml
            this->log_level                     = config["log_level"].as<NUClear::LogLevel>();
            cfg.find_ball                       = config["tasks"]["find_ball"].as<bool>();
            cfg.find_ball_priority              = config["tasks"]["find_ball_priority"].as<int>();
            cfg.look_at_ball                    = config["tasks"]["look_at_ball"].as<bool>();
            cfg.look_at_ball_priority           = config["tasks"]["look_at_ball_priority"].as<int>();
            cfg.walk_to_ball                    = config["tasks"]["walk_to_ball"].as<bool>();
            cfg.walk_to_ball_priority           = config["tasks"]["walk_to_ball_priority"].as<int>();
            cfg.align_ball_to_goal              = config["tasks"]["align_ball_to_goal"].as<bool>();
            cfg.align_ball_to_goal_priority     = config["tasks"]["align_ball_to_goal_priority"].as<int>();
            cfg.kick_to_goal                    = config["tasks"]["kick_to_goal"].as<bool>();
            cfg.kick_to_goal_priority           = config["tasks"]["kick_to_goal_priority"].as<int>();
            cfg.walk_to_field_position          = config["tasks"]["walk_to_field_position"].as<bool>();
            cfg.walk_to_field_position_priority = config["tasks"]["walk_to_field_position_priority"].as<int>();
            cfg.kick_to                         = config["tasks"]["kick_to"].as<bool>();
            cfg.kick_to_priority                = config["tasks"]["kick_to_priority"].as<int>();
            cfg.look_around                     = config["tasks"]["look_around"].as<bool>();
            cfg.look_around_priority            = config["tasks"]["look_around_priority"].as<int>();
            cfg.stand_still                     = config["tasks"]["stand_still"].as<bool>();
            cfg.stand_still_priority            = config["tasks"]["stand_still_priority"].as<int>();
            cfg.walk_to_field_position_position = config["walk_to_field_position_position"].as<Expression>();
        });

        on<Startup>().then([this] {
            // Emit all the tasks
            if (cfg.find_ball) {
                emit<Task>(std::make_unique<FindBall>(), cfg.find_ball_priority);
            }
            if (cfg.look_at_ball) {
                emit<Task>(std::make_unique<LookAtBall>(), cfg.look_at_ball_priority);
            }
            if (cfg.walk_to_ball) {
                emit<Task>(std::make_unique<WalkToBall>(), cfg.walk_to_ball_priority);
            }
            if (cfg.align_ball_to_goal) {
                emit<Task>(std::make_unique<AlignBallToGoal>(), cfg.align_ball_to_goal_priority);
            }
            if (cfg.kick_to_goal) {
                emit<Task>(std::make_unique<KickToGoal>(), cfg.kick_to_goal_priority);
            }
            if (cfg.walk_to_field_position) {
                emit<Task>(
                    std::make_unique<WalkToFieldPosition>(Eigen::Vector3f(cfg.walk_to_field_position_position.x(),
                                                                          cfg.walk_to_field_position_position.y(),
                                                                          0),
                                                          cfg.walk_to_field_position_position.z()),
                    cfg.walk_to_field_position_priority);
            }
            if (cfg.kick_to) {
                emit<Task>(std::make_unique<KickTo>(), cfg.kick_to_priority);
            }
            if (cfg.look_around) {
                emit<Task>(std::make_unique<LookAround>(), cfg.look_around_priority);
            }
            if (cfg.stand_still) {
                emit<Task>(std::make_unique<StandStill>(), cfg.stand_still_priority);
            }
        });
    }

}  // namespace module::purpose
