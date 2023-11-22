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
#include "Look.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/actuation/LimbsIK.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/skill/Look.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::actuation::HeadIK;
    using message::actuation::LimbsSequence;
    using message::actuation::ServoState;
    using utility::input::ServoID;
    using utility::math::coordinates::screen_angular_from_object_direction;
    using utility::math::coordinates::sphericalToCartesian;
    using utility::nusight::graph;
    using LookTask = message::skill::Look;

    Look::Look(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Look.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Look.yaml
            log_level       = config["log_level"].as<NUClear::LogLevel>();
            cfg.head_gain   = config["head_gain"].as<float>();
            cfg.head_torque = config["head_torque"].as<float>();
            // Configure exponential filter
            uPCt_filter.set_alpha(config["smoothing_factor"].as<float>());
        });

        on<Provide<LookTask>, Needs<HeadIK>, Every<90, Per<std::chrono::seconds>>>().then([this](const LookTask& look) {
            // Normalise the look vector
            Eigen::Vector3d req_uPCt = look.rPCt.normalized();

            // If switching from non-smoothed to smoothed angle command, reset the initial goal angle to help
            // locking on to the target
            if (smooth == false && look.smooth == true) {
                uPCt_filter.set_value(req_uPCt);
            }
            smooth = look.smooth;

            // Create the HeadIK message
            auto head_ik  = std::make_unique<HeadIK>();
            head_ik->time = NUClear::clock::now();

            // If smoothing requested, smooth requested angles with exponential filter
            head_ik->uPCt = smooth ? uPCt_filter.update(req_uPCt) : req_uPCt;

            head_ik->servos[ServoID::HEAD_YAW]   = ServoState(cfg.head_gain, cfg.head_torque);
            head_ik->servos[ServoID::HEAD_PITCH] = ServoState(cfg.head_gain, cfg.head_torque);

            emit<Task>(head_ik);
        });
    }

}  // namespace module::skill
