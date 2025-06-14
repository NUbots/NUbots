#include "KickWalk.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::skill::Kick;
    using message::skill::Walk;

    KickWalk::KickWalk(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {
        log<INFO>("KickWalk module constructor called");
        on<Configuration>("KickWalk.yaml").then([this](const Configuration& config) {
            log<INFO>("KickWalk configuration loaded");
            // Use configuration here from file KickWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.kick_step_length  = config["kick_step_length"].as<double>();
            cfg.kick_step_height  = config["kick_step_height"].as<double>();
            cfg.kick_velocity_x   = config["kick_velocity_x"].as<double>();
            cfg.kick_velocity_y   = config["kick_velocity_y"].as<double>();
            cfg.normal_velocity_x = config["normal_velocity_x"].as<double>();
            cfg.kick_steps        = config["kick_steps"].as<int>();
        });

        on<Provide<Kick>>().then([this](const Kick& kick, const RunReason& run_reason) {
            log<INFO>("KickWalk module received a kick request for leg: ", kick.leg);
            // If the walk says the kick is done, return a Done task
            if (run_reason == RunReason::SUBTASK_DONE) {
                log<INFO>("KickWalk module SUBTASK_DONE for leg: ", kick.leg);
                emit<Task>(std::make_unique<Done>());
                return;
            }

            // If this is a new kick, initialize the kick state
            if (run_reason == RunReason::NEW_TASK) {
                cfg.current_kick_leg = kick.leg;
                cfg.kick_step_count  = 0;
                cfg.is_kicking       = true;
            }

            if (cfg.is_kicking && cfg.kick_step_count < cfg.kick_steps) {
                // Modified walk command for step kick
                Eigen::Vector3d walk_velocity;
                // Modify velocity for kicking step
                if (cfg.current_kick_leg == LimbID::RIGHT_LEG) {
                    // Right leg kick - step forward and slightly right
                    walk_velocity = Eigen::Vector3d(cfg.kick_velocity_x, -cfg.kick_velocity_y, 0.0);
                }
                else {
                    // Left leg kick - step forward and slightly left
                    walk_velocity = Eigen::Vector3d(cfg.kick_velocity_x, cfg.kick_velocity_y, 0.0);
                }
                cfg.kick_step_count++;

                // Check if kick sequence is complete
                if (cfg.kick_step_count >= cfg.kick_steps) {
                    cfg.is_kicking = false;
                }
                emit<Task>(std::make_unique<Walk>(walk_velocity, true));
            }
        });
    }

}  // namespace module::skill
