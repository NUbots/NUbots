#include "WalkToBall.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/localisation/FilteredBall.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/strategy/WalkToBall.hpp"

namespace module::strategy {

    using extension::Configuration;
    using message::localisation::FilteredBall;
    using message::planning::WalkTo;
    using WalkToBallTask = message::strategy::WalkToBall;

    WalkToBall::WalkToBall(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("WalkToBall.yaml").then([this](const Configuration& config) {
            // Use configuration here from file WalkToBall.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.ball_y_offset = config["ball_y_offset"].as<float>();
        });

        // If the Provider updates on Every and the last FilteredBall was too long ago, it won't emit any Task
        // Otherwise it will emit a Task to walk to the ball
        on<Provide<WalkToBallTask>, With<FilteredBall>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const FilteredBall& ball) {
                // If we have a ball, walk to it
                if (NUClear::clock::now() - ball.time_of_measurement < cfg.ball_search_timeout) {
                    // Add an offset to account for walking with the foot in front of the ball
                    const Eigen::Vector3f rBRr(ball.rBRr.x(), ball.rBRr.y() + cfg.ball_y_offset, ball.rBRr.z());
                    const float heading = std::atan2(ball.rBRr.y(), ball.rBRr.x());
                    emit<Task>(std::make_unique<WalkTo>(rBRr, heading));
                }
            });
    }

}  // namespace module::strategy
