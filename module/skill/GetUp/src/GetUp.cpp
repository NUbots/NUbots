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
#include "message/skill/SemiDynamicGetup.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::BodySequence;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using GetUpTask = message::skill::GetUp;
    using utility::skill::load_script;
    using message::skill::SemiDynamicGetup;

    GetUp::GetUp(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GetUp.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.delay_time        = config["delay_time"].as<int>();
            cfg.getup_front       = config["scripts"]["getup_front"].as<std::vector<std::string>>();
            cfg.getup_back        = config["scripts"]["getup_back"].as<std::vector<std::string>>();
            cfg.getup_right       = config["scripts"]["getup_right"].as<std::vector<std::string>>();
            cfg.getup_left        = config["scripts"]["getup_left"].as<std::vector<std::string>>();
            cfg.getup_upright     = config["scripts"]["getup_upright"].as<std::vector<std::string>>();
            cfg.getup_upside_down = config["scripts"]["getup_upside_down"].as<std::vector<std::string>>();
        });

        on<Provide<GetUpTask>, Needs<BodySequence>, With<Sensors>>().then([this](const RunReason& run_reason,
                                                                                 const Uses<BodySequence>& body,
                                                                                 const Sensors& sensors) {
            if (run_reason == RunReason::NEW_TASK) {
                // Wait so that sensors have time to settle
                log<INFO>("Delaying getup...");
                emit<Task>(std::make_unique<Wait>(NUClear::clock::now() + std::chrono::milliseconds(cfg.delay_time)));
            }
            // The initial Wait has completed
            else if (run_reason == RunReason::SUBTASK_DONE && !body.done) {
                // Check side and emit getup script
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
                    // emit<Task>(load_script<BodySequence>(cfg.getup_back));
                    emit<Task>(std::make_unique<SemiDynamicGetup>());
                }
                else if (on_right || on_left) {
                    log<INFO>("Landed on side, delaying...");
                    // Delay
                    emit<Task>(
                        std::make_unique<Wait>(NUClear::clock::now() + std::chrono::milliseconds(cfg.delay_time)));
                }
                else if (upright) {
                    log<INFO>("Getting up from upright");
                    emit<Task>(load_script<BodySequence>(cfg.getup_upright));
                }
                else if (upside_down) {
                    log<INFO>("Getting up from upside_down");
                    emit<Task>(load_script<BodySequence>(cfg.getup_upside_down));
                }
            }
            else if (run_reason == RunReason::SUBTASK_DONE) {
                // When the subtask is done, we are done
                log<INFO>("Finished getting up");
                // We are standing
                emit(std::make_unique<Stability>(Stability::STANDING));
                emit<Task>(std::make_unique<Done>());
            }
            else {
                // This shouldn't happen but if it does just continue doing the same thing
                emit<Task>(std::make_unique<Continue>());
            }
        });
    }

}  // namespace module::skill
