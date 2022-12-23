#include "GetUp.hpp"

#include <Eigen/Geometry>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"
#include "extension/behaviour/Script.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/GetUp.hpp"

namespace module::skill {

    using extension::Configuration;
    using extension::behaviour::Script;
    using message::actuation::BodySequence;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using GetUpTask = message::skill::GetUp;

    GetUp::GetUp(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GetUp.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.getup_front.clear();
            for (const auto& s : config["scripts"]["getup_front"].as<std::vector<std::string>>()) {
                cfg.getup_front.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.getup_back.clear();
            for (const auto& s : config["scripts"]["getup_back"].as<std::vector<std::string>>()) {
                cfg.getup_back.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.getup_right.clear();
            for (const auto& s : config["scripts"]["getup_right"].as<std::vector<std::string>>()) {
                cfg.getup_right.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.getup_left.clear();
            for (const auto& s : config["scripts"]["getup_left"].as<std::vector<std::string>>()) {
                cfg.getup_left.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.getup_upright.clear();
            for (const auto& s : config["scripts"]["getup_upright"].as<std::vector<std::string>>()) {
                cfg.getup_upright.push_back(extension::behaviour::ScriptRequest(s));
            }

            cfg.getup_upside_down.clear();
            for (const auto& s : config["scripts"]["getup_upside_down"].as<std::vector<std::string>>()) {
                cfg.getup_upside_down.push_back(extension::behaviour::ScriptRequest(s));
            }
        });

        on<Provide<GetUpTask>, Needs<BodySequence>, With<Sensors>>().then(
            [this](const RunInfo& info, const Sensors& sensors) {
                switch (info.run_reason) {
                    case RunInfo::NEW_TASK: {
                        // If we're running getup we fell over
                        emit(std::make_unique<Stability>(Stability::FALLEN));

                        // Transform to torso {t} from world {w} space
                        Eigen::Isometry3d Hwt = Eigen::Isometry3d(sensors.Htw).inverse();

                        // Decompose our basis axes of the torso {t} into world {w} space
                        Eigen::Vector3d uXTw = Hwt.rotation().col(0);
                        Eigen::Vector3d uYTw = Hwt.rotation().col(1);
                        Eigen::Vector3d uZTw = Hwt.rotation().col(2);

                        // Split into six cases (think faces of a cube) to work out which getup script we should run

                        // torso x is mostly world -z
                        bool on_front = (uXTw.z() <= uXTw.x() && uXTw.z() <= uXTw.y());
                        // torso x is mostly world z
                        bool on_back = (uXTw.z() >= uXTw.x() && uXTw.z() >= uXTw.y());
                        // torso y is mostly world z
                        bool on_right = (uYTw.z() >= uYTw.x() && uYTw.z() >= uYTw.y());
                        // torso y is mostly world -z
                        bool on_left = (uYTw.z() <= uYTw.x() && uYTw.z() <= uYTw.y());
                        // torso z is mostly world z
                        bool upright = (uZTw.z() >= uZTw.x() && uZTw.z() >= uZTw.y());
                        // torso z is mostly world -z
                        bool upside_down = (uZTw.z() <= uZTw.x() && uZTw.z() <= uZTw.y());

                        if (on_front) {
                            log<NUClear::INFO>("Getting up from front");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_front);
                        }
                        else if (on_back) {
                            log<NUClear::INFO>("Getting up from back");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_back);
                        }
                        else if (on_right) {
                            log<NUClear::INFO>("Getting up from right");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_right);
                        }
                        else if (on_left) {
                            log<NUClear::INFO>("Getting up from left");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_left);
                        }
                        else if (upright) {
                            log<NUClear::INFO>("Getting up from upright");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_upright);
                        }
                        else if (upside_down) {
                            log<NUClear::INFO>("Getting up from upside_down");
                            emit<Script>(std::make_unique<BodySequence>(), cfg.getup_upside_down);
                        }

                    } break;
                    case RunInfo::SUBTASK_DONE: {
                        // When the subtask is done, we are done
                        log<NUClear::INFO>("Finished getting up");
                        emit(std::make_unique<Stability>(Stability::STANDING));
                        emit<Task>(std::make_unique<Done>());
                    } break;
                    // These shouldn't happen but if they do just idle
                    default: emit<Task>(std::make_unique<Idle>()); break;
                }
            });
    }

}  // namespace module::skill
