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

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/input/Sensors.hpp"
#include "message/skill/ControlFoot.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::actuation {


    using message::actuation::LeftLegIK;
    using message::actuation::RightLegIK;
    using message::actuation::ServoState;
    using message::input::Sensors;
    using message::skill::ControlLeftFoot;
    using message::skill::ControlRightFoot;

    using utility::input::LimbID;

    class FootController : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Mode of operation
            std::string mode = "IK";
            /// @brief Gains for the servos
            double servo_gain = 0.0;
        } cfg;

        // /// @brief Map between ServoID and ServoState
        // std::map<utility::input::ServoID, message::actuation::ServoState> servo_states = {
        //     {utility::input::ServoID::LEFT_HIP_YAW, message::actuation::ServoState()},
        //     {utility::input::ServoID::LEFT_HIP_ROLL, message::actuation::ServoState()},
        //     {utility::input::ServoID::LEFT_HIP_PITCH, message::actuation::ServoState()},
        //     {utility::input::ServoID::LEFT_KNEE, message::actuation::ServoState()},
        //     {utility::input::ServoID::LEFT_ANKLE_PITCH, message::actuation::ServoState()},
        //     {utility::input::ServoID::LEFT_ANKLE_ROLL, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_HIP_YAW, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_HIP_ROLL, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_HIP_PITCH, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_KNEE, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_ANKLE_PITCH, message::actuation::ServoState()},
        //     {utility::input::ServoID::RIGHT_ANKLE_ROLL, message::actuation::ServoState()},
        // };

    public:
        /// @brief Called by the powerplant to build and setup the FootController reactor.
        explicit FootController(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Foot controller logic
        template <typename FootControlTask, typename IKTask>
        void control_foot(const FootControlTask& foot_control_task,
                          IKTask& ik_task,
                          const Sensors& sensors,
                          LimbID limb_id) {
            // Set the time
            ik_task->time = foot_control_task.time;

            // Set the IK target
            ik_task->Htf = foot_control_task.Htf;

            for (auto id : utility::input::LimbID::servos_for_limb(limb_id)) {
                ik_task->servos[id] = ServoState(cfg.servo_gain, 100);
            }
        }
    };

}  // namespace module::actuation

#endif  // MODULE_ACTUATION_FOOTCONTROLLER_HPP
