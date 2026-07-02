/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#include "HSLLocalisationDebug.hpp"

#include "extension/Configuration.hpp"

#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/support/nusight/HSLLocalisationDebug.hpp"
#include "message/vision/FieldIntersections.hpp"

namespace module::output {

    using extension::Configuration;

    using LocalisationBall         = message::localisation::Ball;
    using LocalisationField        = message::localisation::Field;
    using LocalisationRobots       = message::localisation::Robots;
    using VisionFieldIntersections = message::vision::FieldIntersections;
    using DebugMsg                 = message::support::nusight::HSLLocalisationDebug;
    using RobotSummary             = message::support::nusight::RobotSummary;

    HSLLocalisationDebug::HSLLocalisationDebug(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("HSLLocalisationDebug.yaml").then([this](const Configuration& cfg) {
            log_level                   = cfg["log_level"].as<NUClear::LogLevel>();
            max_field_association_lines = cfg["max_field_association_lines"].as<size_t>();
            max_field_particles         = cfg["max_field_particles"].as<size_t>();
        });

        on<Every<1, Per<std::chrono::seconds>>,
           Optional<With<VisionFieldIntersections>>,
           Optional<With<LocalisationRobots>>,
           Optional<With<LocalisationBall>>,
           Optional<With<LocalisationField>>,
           Single,
           Priority::LOW>()
            .then([this](const std::shared_ptr<const VisionFieldIntersections>& field_intersections,
                         const std::shared_ptr<const LocalisationRobots>& robots,
                         const std::shared_ptr<const LocalisationBall>& ball,
                         const std::shared_ptr<const LocalisationField>& field) {
                auto msg = std::make_unique<DebugMsg>();

                if (field_intersections) {
                    msg->field_intersections = *field_intersections;
                }
                if (robots) {
                    for (const auto& robot : robots->robots) {
                        RobotSummary summary;
                        summary.id       = robot.id;
                        summary.rRWw     = robot.rRWw;
                        summary.teammate = robot.teammate;
                        msg->robots.robots.push_back(summary);
                    }
                }
                if (ball) {
                    msg->ball = *ball;
                }
                if (field) {
                    msg->field = *field;
                    if (msg->field.association_lines.size() > max_field_association_lines) {
                        msg->field.association_lines.resize(max_field_association_lines);
                    }
                    if (msg->field.particles.size() > max_field_particles) {
                        msg->field.particles.resize(max_field_particles);
                    }
                }

                emit(msg);
            });
    }

}  // namespace module::output
