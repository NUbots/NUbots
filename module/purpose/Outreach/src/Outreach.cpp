/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#include "Outreach.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/Limbs.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/planning/LookAround.hpp"
#include "message/purpose/Outreach.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/vision/BoundingBoxes.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::actuation::RightArmSequence;
    using message::behaviour::state::Stability;
    using message::input::Sensors;
    using message::planning::LookAround;
    using message::purpose::WaveAtPerson;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::vision::BoundingBoxes;
    using OutreachTask = message::purpose::Outreach;

    using utility::skill::load_script;
    using utility::skill::ScriptRequest;

    Outreach::Outreach(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Outreach.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Outreach.yaml
            this->log_level          = config["log_level"].as<NUClear::LogLevel>();
            cfg.confidence_threshold = config["confidence_threshold"].as<double>();
            cfg.person_timeout       = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["person_timeout"].as<double>()));
            cfg.turn_gain      = config["turn_gain"].as<double>();
            cfg.max_turn_speed = config["max_turn_speed"].as<double>();
            cfg.turn_deadband  = config["turn_deadband"].as<double>();
            cfg.wave_cooldown  = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config["wave_cooldown"].as<double>()));
            cfg.wave_speed             = config["wave_speed"].as<double>();
            cfg.look_priority          = config["tasks"]["look_priority"].as<int>();
            cfg.walk_priority          = config["tasks"]["walk_priority"].as<int>();
            cfg.wave_priority          = config["tasks"]["wave_priority"].as<int>();
            cfg.outreach_priority      = config["tasks"]["outreach_priority"].as<int>();
            cfg.fall_recovery_priority = config["tasks"]["fall_recovery_priority"].as<int>();
        });

        // The walk needs a stability state to run before anything has told us we are standing
        on<Startup>().then([this] { emit(std::make_unique<Stability>(Stability::STANDING)); });

        // Remember where the most confident person in the image is
        on<Trigger<BoundingBoxes>, With<Sensors>>().then([this](const BoundingBoxes& boxes, const Sensors& sensors) {
            auto person = std::max_element(boxes.bounding_boxes.begin(),
                                           boxes.bounding_boxes.end(),
                                           [](const auto& a, const auto& b) {
                                               // Anything that isn't a person always compares as the smaller box
                                               if (a.name != "person") {
                                                   return b.name == "person";
                                               }
                                               return b.name == "person" && a.confidence < b.confidence;
                                           });

            if (person == boxes.bounding_boxes.end() || person->name != "person"
                || person->confidence < cfg.confidence_threshold) {
                return;
            }

            // The corners are unit vectors to the corners of the box. Summing them gives a vector that is not itself
            // a unit vector, but that points the same way their mean does, which is the middle of the person
            Eigen::Vector3d corner_sum = Eigen::Vector3d::Zero();
            for (const auto& corner : person->corners) {
                corner_sum += corner.normalized();
            }

            // Corners that cancel each other out would leave nothing to normalise, and no direction to look in
            if (corner_sum.norm() < std::numeric_limits<double>::epsilon()) {
                log<WARN>("Ignoring a person bounding box with no usable centre direction");
                return;
            }

            // Normalising the sum turns it into the unit vector to the middle of the person, in camera space
            const Eigen::Vector3d uPCc = corner_sum.normalized();

            // Rotating into torso space keeps it a unit vector, as a rotation preserves length
            uPCt      = (sensors.Htw * boxes.Hcw.inverse()).rotation() * uPCc;
            last_seen = boxes.timestamp;

            log<DEBUG>("Person at bearing ", std::atan2(uPCt.y(), uPCt.x()), " with confidence ", person->confidence);
        });

        // Drive the Director tree at a fixed rate
        on<Every<BEHAVIOUR_UPDATE_RATE, Per<std::chrono::seconds>>>().then([this] {
            // Getting up beats greeting people, so it goes in at a higher priority
            emit<Task>(std::make_unique<FallRecovery>(), cfg.fall_recovery_priority);
            emit<Task>(std::make_unique<OutreachTask>(), cfg.outreach_priority);
        });

        on<Provide<OutreachTask>>().then([this] {
            // The wave decides for itself whether there is someone worth waving at
            emit<Task>(std::make_unique<WaveAtPerson>(), cfg.wave_priority);

            if (!person_seen_recently()) {
                // Nobody around, so stand still and scan for someone
                emit<Task>(std::make_unique<LookAround>(), cfg.look_priority);
                emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), cfg.walk_priority);
                return;
            }

            // Point the head at the person, which centres them in the image
            emit<Task>(std::make_unique<Look>(uPCt, true), cfg.look_priority);

            // Turn the body on the spot until the person is straight ahead, which unwinds the head yaw
            const double bearing = std::atan2(uPCt.y(), uPCt.x());
            const double omega   = std::abs(bearing) < cfg.turn_deadband
                                       ? 0.0
                                       : std::clamp(cfg.turn_gain * bearing, -cfg.max_turn_speed, cfg.max_turn_speed);
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(0.0, 0.0, omega)), cfg.walk_priority);
        });

        // Wave whenever someone is being tracked, but no more often than the cooldown allows
        on<Provide<WaveAtPerson>>().then([this] {
            if (person_seen_recently() && NUClear::clock::now() - last_wave > cfg.wave_cooldown) {
                last_wave = NUClear::clock::now();
                emit<Task>(load_script<RightArmSequence>(ScriptRequest{"Wave.yaml", float(cfg.wave_speed)}));
                return;
            }

            // Let a wave that is already running finish, and otherwise leave the arm to the walk
            emit<Task>(std::make_unique<Continue>());
        });
    }

}  // namespace module::purpose
