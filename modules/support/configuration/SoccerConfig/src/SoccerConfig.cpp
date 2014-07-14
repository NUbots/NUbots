/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SoccerConfig.h"

#include <armadillo>

#include "messages/support/Configuration.h"
#include "messages/support/FieldDescription.h"

using messages::support::Configuration;
using messages::support::FieldDescription;

namespace modules {
namespace support {
namespace configuration {

    void SetGoalpostPositions(FieldDescription& desc) {
        // Unused formulas remain as comments for completeness.

        FieldDescription::FieldDimensions& d = desc.dimensions;
        auto half_length = d.field_length * 0.5;
        // auto goal_line_width = d.line_width * 0.5;
        auto goal_post_radius = d.goalpost_diameter * 0.5;
        // auto goal_x = (half_length - d.line_width * 0.5 + d.goal_depth + goal_line_width * 0.5);
        auto goal_y = (d.goal_width + d.goalpost_diameter) * 0.5;
        // auto goal_w = (d.goal_depth - d.line_width + goal_line_width * 0.5);
        // auto goal_h = (d.goal_width + d.goalpost_diameter);
        auto goal_post_x = half_length + goal_post_radius * 0.5;

        desc.goalpost_bl = { -goal_post_x, -goal_y };
        desc.goalpost_br = { -goal_post_x,  goal_y };
        desc.goalpost_yl = {  goal_post_x,  goal_y };
        desc.goalpost_yr = {  goal_post_x, -goal_y };
    }

    FieldDescription LoadFieldDescription(
        Configuration<FieldDescriptionConfig> config) {
        FieldDescription desc;

        desc.ball_radius = config.config["BallRadius"].as<double>();

        FieldDescription::FieldDimensions& d = desc.dimensions;
        d.line_width = config.config["LineWidth"].as<double>();
        d.mark_width = config.config["MarkWidth"].as<double>();
        d.field_length = config.config["FieldLength"].as<double>();
        d.field_width = config.config["FieldWidth"].as<double>();
        d.goal_depth = config.config["GoalDepth"].as<double>();
        d.goal_width = config.config["GoalWidth"].as<double>();
        d.goal_area_length = config.config["GoalAreaLength"].as<double>();
        d.goal_area_width = config.config["GoalAreaWidth"].as<double>();
        d.goal_crossbar_height = config.config["GoalCrossbarHeight"].as<double>();
        d.goalpost_diameter = config.config["GoalpostDiameter"].as<double>();
        d.goal_crossbar_diameter = config.config["GoalCrossbarDiameter"].as<double>();
        d.goal_net_height = config.config["GoalNetHeight"].as<double>();
        d.penalty_mark_distance = config.config["PenaltyMarkDistance"].as<double>();
        d.center_circle_diameter = config.config["CenterCircleDiameter"].as<double>();
        d.border_strip_min_width = config.config["BorderStripMinWidth"].as<double>();
        d.border_strip_min_width = config.config["BorderStripMinWidth"].as<double>();

        desc.goalpost_top_height = d.goal_crossbar_height + d.goal_crossbar_diameter;

        SetGoalpostPositions(desc);

        return desc;
    }

    SoccerConfig::SoccerConfig(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Configuration<FieldDescriptionConfig>>>(
            "FieldDescriptionConfig Update",
            [this](const Configuration<FieldDescriptionConfig>& config) {
            auto fd = std::make_unique<messages::support::FieldDescription>(LoadFieldDescription(config));
            emit(std::move(fd));
        });

    }

}
}
}

