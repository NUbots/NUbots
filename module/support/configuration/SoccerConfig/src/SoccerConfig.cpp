/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#include "SoccerConfig.hpp"

#include "extension/Configuration.hpp"

#include "message/support/FieldDescription.hpp"

namespace module::support::configuration {

    using extension::Configuration;

    using message::support::FieldDescription;

    void SetGoalpostPositions(FieldDescription& desc) {
        // Unused formulas remain as comments for completeness.

        FieldDescription::FieldDimensions& d = desc.dimensions;
        auto half_length                     = d.field_length * 0.5;
        // auto goal_line_width = d.line_width * 0.5;
        auto goal_post_radius = d.goalpost_width * 0.5;
        // auto goal_x = (half_length - d.line_width * 0.5 + d.goal_depth + goal_line_width * 0.5);
        auto goal_y = (d.goal_width + d.goalpost_width) * 0.5;
        // auto goal_w = (d.goal_depth - d.line_width + goal_line_width * 0.5);
        // auto goal_h = (d.goal_width + d.goalpost_diameter);
        auto goal_post_x = half_length + goal_post_radius * 0.5;

        desc.goalpost_own_l = {-goal_post_x, -goal_y};
        desc.goalpost_own_r = {-goal_post_x, goal_y};
        desc.goalpost_opp_l = {goal_post_x, goal_y};
        desc.goalpost_opp_r = {goal_post_x, -goal_y};
    }

    FieldDescription LoadFieldDescription(const Configuration& config) {
        FieldDescription desc;

        desc.ball_radius = config["BallRadius"].as<double>();

        FieldDescription::FieldDimensions& d = desc.dimensions;
        d.line_width                         = config["LineWidth"].as<double>();
        d.field_length                       = config["FieldLength"].as<double>();
        d.field_width                        = config["FieldWidth"].as<double>();
        d.goal_depth                         = config["GoalDepth"].as<double>();
        d.goal_width                         = config["GoalWidth"].as<double>();
        d.goal_area_length                   = config["GoalAreaLength"].as<double>();
        d.goal_area_width                    = config["GoalAreaWidth"].as<double>();
        d.goal_crossbar_height               = config["GoalCrossbarHeight"].as<double>();
        d.goalpost_type                      = config["GoalpostType"].as<std::string>();
        d.goalpost_width                     = config["GoalpostWidth"].as<double>();
        d.goalpost_depth                     = config["GoalpostDepth"].as<double>();
        d.goal_crossbar_width                = config["GoalCrossbarWidth"].as<double>();
        d.goal_crossbar_depth                = config["GoalCrossbarDepth"].as<double>();
        d.goal_net_height                    = config["GoalNetHeight"].as<double>();
        d.penalty_mark_distance              = config["PenaltyMarkDistance"].as<double>();
        d.center_circle_diameter             = config["CenterCircleDiameter"].as<double>();
        d.border_strip_min_width             = config["BorderStripMinWidth"].as<double>();
        d.penalty_area_length                = config["PenaltyAreaLength"].as<double>();
        d.penalty_area_width                 = config["PenaltyAreaWidth"].as<double>();

        desc.goalpost_top_height = d.goal_crossbar_height + d.goal_crossbar_width;

        SetGoalpostPositions(desc);

        return desc;
    }

    SoccerConfig::SoccerConfig(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("FieldDescription.yaml")
            .then("FieldDescriptionConfig Update", [this](const Configuration& config) {
                log_level = config["log_level"].as<NUClear::LogLevel>();
                auto fd   = std::make_unique<message::support::FieldDescription>(LoadFieldDescription(config));
                emit(std::move(fd));
            });
    }
}  // namespace module::support::configuration
