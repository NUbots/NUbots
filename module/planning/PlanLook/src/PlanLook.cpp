#include "PlanLook.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/FilteredBall.hpp"
#include "message/planning/LookAtFeature.hpp"
#include "message/skill/Look.hpp"
#include "message/vision/Goal.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::FilteredBall;
    using message::planning::LookAround;
    using message::planning::LookAtBall;
    using message::planning::LookAtGoals;
    using message::skill::Look;
    using message::vision::Goals;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::support::Expression;

    PlanLook::PlanLook(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanLook.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanLook.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.ball_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["ball_search_timeout"].as<double>()));
            cfg.goal_search_timeout = duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["goal_search_timeout"].as<double>()));
            cfg.search_fixation_time = config["search_fixation_time"].as<float>();

            // Create vector of search positions
            for (const auto& search_position : config["search_positions"].config) {
                cfg.search_positions.push_back(search_position.as<Expression>());
            }
        });

        on<Provide<LookAtBall>, Optional<With<FilteredBall>>, Every<30, Per<std::chrono::seconds>>>().then(
            [this](const std::shared_ptr<const FilteredBall>& ball) {
                // If we have a ball and it is recent, look at it
                if (ball && (NUClear::clock::now() - ball->time_of_measurement < cfg.ball_search_timeout)) {
                    emit<Task>(std::make_unique<Look>(ball->rBCt.cast<double>(), true));
                }
                // Otherwise, look around for the ball
                else {
                    emit<Task>(std::make_unique<LookAround>());
                }
            });

        on<Provide<LookAtGoals>, Optional<With<Goals>>, With<Sensors>>().then(
            [this](const std::shared_ptr<const Goals>& goals, const Sensors& sensors) {
                // If we have goals, with at least one measurement and the goals are recent, look at the goals
                if (goals && !goals->goals.empty()
                    && (NUClear::clock::now() - goals->timestamp < cfg.goal_search_timeout)) {
                    // Convert goal measurement to cartesian coordinates
                    Eigen::Vector3d rGCc = sphericalToCartesian(goals->goals[0].measurements[0].srGCc.cast<double>());
                    // Convert to torso space
                    Eigen::Vector3d rGCt = Eigen::Isometry3d(sensors.Htw * goals->Hcw.inverse()).rotation() * rGCc;
                    // Look at the goal
                    emit<Task>(std::make_unique<Look>(rGCt, true));
                }
                // Otherwise, look around for the goals
                else {
                    emit<Task>(std::make_unique<LookAround>());
                }
            });

        on<Provide<LookAround>, Every<30, Per<std::chrono::seconds>>>().then([this] {
            // How long the look has lingered - will move to the next position if long enough
            float time_since_last_search_moved =
                std::chrono::duration_cast<std::chrono::duration<float>>(NUClear::clock::now() - search_last_moved)
                    .count();

            // Robot will move through the search positions, and linger for search_fixation_time. Once
            // search_fixation_time time has passed, send a new head command for the next position in the list
            // of cfg.search_positions
            if (time_since_last_search_moved > cfg.search_fixation_time) {
                // Send command for look position
                double yaw   = cfg.search_positions[search_idx][0];
                double pitch = cfg.search_positions[search_idx][1];

                // Make a vector pointing straight forwards and rotate it by the pitch and yaw
                Eigen::Vector3d uPCt = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
                                           .toRotationMatrix()
                                       * Eigen::Vector3d::UnitX();
                emit<Task>(std::make_unique<Look>(uPCt, false));

                // Move to next search position in list
                search_last_moved = NUClear::clock::now();
                // Increase the index and wrap around if we have reached the end of the list
                search_idx = (search_idx + 1) % cfg.search_positions.size();
            }
        });

        // Start from the first search position
        on<Start<LookAround>>().then([this] { search_idx = 0; });
    }

}  // namespace module::planning
