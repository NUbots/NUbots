#include "DiveToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/BodySide.hpp"
#include "message/actuation/Limbs.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/skill/Dive.hpp"
#include "message/strategy/DiveToBall.hpp"

namespace module::strategy {

    using extension::Configuration;
    using DiveToBallTask = message::strategy::DiveToBall;
    using message::actuation::BodySequence;
    using message::actuation::BodySide;
    using message::localisation::FilteredBall;
    using message::skill::Dive;


    DiveToBall::DiveToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("DiveToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file DiveToBall.yaml
            this->log_level               = config["log_level"].as<NUClear::LogLevel>();
            cfg.diving_distance_threshold = config["diving_distance_threshold"].as<float>();
        });


        on<DiveToBallTask, Trigger<FilteredBall>>().then([this](const FilteredBall& ball) {
            // Get the distance to the ball
            float distance_to_ball = ball->rBTt.head(2).norm();

            // If the distance to the ball is greater than the threshold, do nothing
            if (distance_to_ball > cfg.diving_distance_threshold) {
                return;
            }

            // Determine angle to ball and whether we should dive right or left
            float yaw_angle         = std::atan2(ball->rBTt.y(), ball->rBTt.x());
            BodySide dive_direction = yaw_angle < 0 ? BodySide::RIGHT : BodySide::LEFT;
            emit<Dive>(std::make_unique<BodySequence>(), dive_direction);
        });
    }

}  // namespace module::strategy
