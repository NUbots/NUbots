#include "GoalLocalisation.hpp"

#include <fmt/format.h>

#include "extension/Configuration.hpp"

#include "message/localisation/Goal.hpp"
#include "message/vision/Goal.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"
#include "utility/vision/visualmesh/VisualMesh.hpp"

namespace module::localisation {

    using extension::Configuration;

    using LocalisationGoal  = message::localisation::Goal;
    using LocalisationGoals = message::localisation::Goals;
    using VisionGoal        = message::vision::Goal;
    using VisionGoals       = message::vision::Goals;

    using message::eye::DataPoint;
    using message::vision::GreenHorizon;

    using utility::math::geometry::point_in_convex_hull;
    using utility::nusight::graph;
    using utility::support::Expression;

    GoalLocalisation::GoalLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GoalLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GoalLocalisation.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            // Set our UKF filter parameters
            cfg.ukf.noise.measurement.position =
                Eigen::Vector2d(config["ukf"]["noise"]["measurement"]["goal_position"].as<Expression>()).asDiagonal();
            cfg.ukf.noise.process.position      = config["ukf"]["noise"]["process"]["position"].as<Expression>();
            cfg.ukf.noise.process.velocity      = config["ukf"]["noise"]["process"]["velocity"].as<Expression>();
            cfg.ukf.initial_covariance.position = config["ukf"]["initial_covariance"]["position"].as<Expression>();
            cfg.ukf.initial_covariance.velocity = config["ukf"]["initial_covariance"]["velocity"].as<Expression>();
            cfg.association_distance            = config["association_distance"].as<double>();
            cfg.max_missed_count                = config["max_missed_count"].as<int>();
        });


        on<Trigger<VisionGoals>, With<GreenHorizon>, Single>().then([this](const VisionGoals& vision_goals,
                                                                           const GreenHorizon& horizon) {
            // Print tracked_goals ids
            log<NUClear::DEBUG>("Goals tracked:");
            for (const auto& tracked_goal : tracked_goals) {
                log<NUClear::DEBUG>("\tID: ", tracked_goal.id);
            }

            // Set all tracked goals to unseen
            for (auto& tracked_goal : tracked_goals) {
                tracked_goal.seen = false;
            }

            // **************** Data association ****************
            if (!vision_goals.goals.empty()) {
                Eigen::Isometry3d Hwc = Eigen::Isometry3d(vision_goals.Hcw).inverse();
                for (const auto& vision_goal : vision_goals.goals) {
                    // Position of goal {r} in world {w} space
                    auto rGWw = Hwc * (vision_goal.post.bottom * vision_goal.post.distance);
                    log<NUClear::DEBUG>("Distance:", vision_goal.post.distance);
                    log<NUClear::DEBUG>("Vision goal:", rGWw.transpose());

                    // Only consider vision measurements within the green horizon
                    if (point_in_convex_hull(horizon.horizon, rGWw)) {
                        // Data association: find tracked goal which is associated with the vision measurement
                        data_association(rGWw);
                    }
                }
            }

            // **************** Goal tracking maintenance ****************
            for (auto& tracked_goal : tracked_goals) {
                auto state = GoalModel<double>::StateVec(tracked_goal.ukf.get_state());

                // If a tracked goal has moved outside of view, keep it as seen so we don't lose it
                // A goal is outside of view if it is not within the green horizon
                // TODO: It may be better to use fov and image size to determine if a goal should be seen
                if (!point_in_convex_hull(horizon.horizon, Eigen::Vector3d(state.rGWw.x(), state.rGWw.y(), 0))) {
                    tracked_goal.seen = true;
                }

                // If the tracked goal has not been seen, increment the consecutively missed count
                tracked_goal.missed_count = tracked_goal.seen ? 0 : tracked_goal.missed_count + 1;
            }

            // Make a vector to store kept goals
            std::vector<TrackedGoal> new_tracked_goals;
            // Only keep goals that are not missing or too close to others
            for (const auto& tracked_goal : tracked_goals) {
                if (tracked_goal.missed_count > cfg.max_missed_count) {
                    log<NUClear::DEBUG>(fmt::format("Removing goal {} due to missed count", tracked_goal.id));
                    continue;
                }

                if (std::any_of(tracked_goals.begin(), tracked_goals.end(), [&](const auto& other_goal) {
                        return &tracked_goal != &other_goal
                               && (tracked_goal.get_rGWw() - other_goal.get_rGWw()).norm() < cfg.association_distance;
                    })) {
                    log<NUClear::DEBUG>(fmt::format("Removing goal {} due to proximity", tracked_goal.id));
                    continue;
                }

                // If neither case is true, keep the goal
                new_tracked_goals.push_back(tracked_goal);
            }
            tracked_goals = std::move(new_tracked_goals);

            // Emit the localisation of the goals
            auto localisation_goals = std::make_unique<LocalisationGoals>();
            for (const auto& tracked_goal : tracked_goals) {
                auto state = GoalModel<double>::StateVec(tracked_goal.ukf.get_state());
                LocalisationGoal localisation_goal;
                localisation_goal.id                  = tracked_goal.id;
                localisation_goal.rGWw                = Eigen::Vector3d(state.rGWw.x(), state.rGWw.y(), 0);
                localisation_goal.covariance          = tracked_goal.ukf.get_covariance();
                localisation_goal.time_of_measurement = tracked_goal.last_time_update;
                localisation_goals->goals.push_back(localisation_goal);
            }
            emit(std::move(localisation_goals));
        });
    }

    void GoalLocalisation::data_association(const Eigen::Vector3d& rGWw) {
        // If we have no goals yet, this must be a new goal
        if (tracked_goals.empty()) {
            tracked_goals.emplace_back(TrackedGoal(rGWw, cfg.ukf, next_id++));
            return;
        }

        // Get the closest goal we have to the given vision measurement
        auto closest_goal_itr =
            std::min_element(tracked_goals.begin(),
                             tracked_goals.end(),
                             [&rGWw](const TrackedGoal& a, const TrackedGoal& b) {
                                 // Get each goal's x-y 2D position in the world
                                 auto a_rGWw = a.get_rGWw();
                                 auto b_rGWw = b.get_rGWw();
                                 // Compare to see which is closer to the goal vision measurement position
                                 return (rGWw.head<2>() - a_rGWw).norm() < (rGWw.head<2>() - b_rGWw).norm();
                             });
        double closest_distance = (rGWw.head<2>() - closest_goal_itr->get_rGWw()).norm();

        // If the closest goal is far enough away, add this as a new goal
        if (closest_distance > cfg.association_distance) {
            tracked_goals.emplace_back(TrackedGoal(rGWw, cfg.ukf, next_id++));
            return;
        }

        // Update the filter on the goal associated with the vision measurement
        auto now = NUClear::clock::now();
        const auto dt =
            std::chrono::duration_cast<std::chrono::duration<double>>(now - closest_goal_itr->last_time_update).count();
        closest_goal_itr->last_time_update = now;
        closest_goal_itr->ukf.time(dt);
        closest_goal_itr->ukf.measure(Eigen::Vector2d(rGWw.head<2>()),
                                      cfg.ukf.noise.measurement.position,
                                      MeasurementType::GOAL_POSITION());
        closest_goal_itr->seen = true;
    }


}  // namespace module::localisation
