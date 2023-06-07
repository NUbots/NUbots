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
            cfg.diving_distance_threshold = config["diving_distance_threshold"].as<double>();
        });

        on<Provide<DiveToBallTask>, Trigger<FilteredBall>>().then(
            [this](const RunInfo& info, const FilteredBall& ball) {
                // If we ran because the Dive is done, then we don't keep running the Dive
                if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                    return;
                }

                // If the distance to the ball is less than the threshold, dive
                if (ball.rBTt.head(2).x() < cfg.diving_distance_threshold) {
                    // Determine angle to ball and whether we should dive right or left
                    double yaw_angle        = std::atan2(ball.rBTt.y(), ball.rBTt.x());
                    BodySide dive_direction = yaw_angle < 0 ? BodySide::RIGHT : BodySide::LEFT;
                    emit<Task>(std::make_unique<Dive>(dive_direction));
                }
            });
    }

}  // namespace module::strategy
