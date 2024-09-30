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
#ifndef MODULE_ACTUATION_FOOTCONTROLLER_HPP
#define MODULE_ACTUATION_FOOTCONTROLLER_HPP

#include <algorithm>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "extension/Behaviour.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/Servos.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"

#include "utility/file/fileutil.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/math/comparison.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::actuation {

    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::Servo;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    using message::actuation::ServoID;
    using utility::input::LimbID;
    using utility::nusight::graph;

    struct SetGains {};

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Foot controller mode
            std::string mode = "IK";

            /// @brief Map between ServoID and Servos
            std::map<ServoID, Servo> servos = {
                {ServoID::L_HIP_YAW, Servo()},
                {ServoID::L_HIP_ROLL, Servo()},
                {ServoID::L_HIP_PITCH, Servo()},
                {ServoID::L_KNEE, Servo()},
                {ServoID::L_ANKLE_PITCH, Servo()},
                {ServoID::L_ANKLE_ROLL, Servo()},
                {ServoID::R_HIP_YAW, Servo()},
                {ServoID::R_HIP_ROLL, Servo()},
                {ServoID::R_HIP_PITCH, Servo()},
                {ServoID::R_KNEE, Servo()},
                {ServoID::R_ANKLE_PITCH, Servo()},
                {ServoID::R_ANKLE_ROLL, Servo()},
            };
            /// @brief Startup gain before setting the desired gains
            double startup_gain = 0;
            /// @brief Delay before setting the desired gains
            std::chrono::seconds set_gain_delay = std::chrono::seconds(0);
            /// @brief Desired gains for each servo
            std::map<std::string, double> desired_gains{};
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Foot controller logic
        template <typename FootControlTask, typename IKTask>
        void control_foot(const FootControlTask& foot_control_task,
                          IKTask& ik_task,
                          const Sensors& sensors,
                          LimbID limb_id) {

            ik_task->Htf = foot_control_task.Htf;

            if (cfg.mode == "IK") {
                for (auto id : utility::input::LimbID::servos_for_limb(limb_id)) {
                    ik_task->servos[id]                = cfg.servos[id];
                    ik_task->servos[id].goal.goal_time = foot_control_task.time;
                }
            }
            else {
                throw std::runtime_error("Invalid mode");
            }
        }
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
