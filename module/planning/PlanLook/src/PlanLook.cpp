#include "PlanLook.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/planning/LookAround.hpp"
#include "message/skill/Look.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::planning {

    using extension::Configuration;
    using message::planning::LookAround;
    using message::skill::Look;
    using utility::support::Expression;

    PlanLook::PlanLook(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("PlanLook.yaml").then([this](const Configuration& config) {
            // Use configuration here from file PlanLook.yaml
            this->log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.search_fixation_time = config["search_fixation_time"].as<float>();

            // Create vector of search positions
            for (const auto& search_position : config["search_positions"].config) {
                cfg.search_positions.push_back(search_position.as<Expression>());
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
            else {
                emit<Task>(std::make_unique<Idle>());
            }
        });

        // Start from the first search position
        on<Start<LookAround>>().then([this] { search_idx = 0; });
    }

}  // namespace module::planning
