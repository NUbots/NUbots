/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "GetUp.hpp"

#include <Eigen/Geometry>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/GetUp.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using GetUpTask = message::skill::GetUp;
    using utility::skill::load_script;

    GetUp::GetUp(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GetUp.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.getup_front       = config["scripts"]["getup_front"].as<std::vector<std::string>>();
            cfg.getup_back        = config["scripts"]["getup_back"].as<std::vector<std::string>>();
            cfg.getup_right       = config["scripts"]["getup_right"].as<std::vector<std::string>>();
            cfg.getup_left        = config["scripts"]["getup_left"].as<std::vector<std::string>>();
            cfg.getup_upright     = config["scripts"]["getup_upright"].as<std::vector<std::string>>();
            cfg.getup_upside_down = config["scripts"]["getup_upside_down"].as<std::vector<std::string>>();
        });

        on<Provide<GetUpTask>, Needs<BodySequence>, With<Sensors>>().then(
            [this](const RunReason& run_reason, const Sensors& sensors) {
                switch (run_reason) {
                    case RunReason::NEW_TASK: {
                        // If we're running getup we fell over
                        emit(std::make_unique<Stability>(Stability::FALLEN));

                        // Transform to torso {t} from world {w} space
                        Eigen::Isometry3d Hwt = sensors.Htw.inverse();

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
                            log<INFO>("Getting up from front");
                            emit<Task>(load_script<BodySequence>(cfg.getup_front));
                        }
                        else if (on_back) {
                            log<INFO>("Getting up from back");
                            emit<Task>(load_script<BodySequence>(cfg.getup_back));
                        }
                        else if (on_right) {
                            log<INFO>("Getting up from right");
                            emit<Task>(load_script<BodySequence>(cfg.getup_right));
                        }
                        else if (on_left) {
                            log<INFO>("Getting up from left");
                            emit<Task>(load_script<BodySequence>(cfg.getup_left));
                        }
                        else if (upright) {
                            log<INFO>("Getting up from upright");
                            emit<Task>(load_script<BodySequence>(cfg.getup_upright));
                        }
                        else if (upside_down) {
                            log<INFO>("Getting up from upside_down");
                            emit<Task>(load_script<BodySequence>(cfg.getup_upside_down));
                        }

                    } break;
                    case RunReason::SUBTASK_DONE: {
                        // When the subtask is done, we are done
                        log<INFO>("Finished getting up");
                        emit(std::make_unique<Stability>(Stability::STANDING));
                        emit<Task>(std::make_unique<Done>());
                    } break;
                    // These shouldn't happen but if they do just continue doing the same thing
                    default: emit<Task>(std::make_unique<Continue>()); break;
                }
            });
    }

}  // namespace module::skill
