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

        desc.ball_radius = config["BallRadius"].as<float>();

        FieldDescription::FieldDimensions& d = desc.dimensions;
        d.line_width                         = config["LineWidth"].as<float>();
        d.field_length                       = config["FieldLength"].as<float>();
        d.field_width                        = config["FieldWidth"].as<float>();
        d.goal_depth                         = config["GoalDepth"].as<float>();
        d.goal_width                         = config["GoalWidth"].as<float>();
        d.goal_area_length                   = config["GoalAreaLength"].as<float>();
        d.goal_area_width                    = config["GoalAreaWidth"].as<float>();
        d.goal_crossbar_height               = config["GoalCrossbarHeight"].as<float>();
        d.goalpost_type                      = config["GoalpostType"].as<std::string>();
        d.goalpost_width                     = config["GoalpostWidth"].as<float>();
        d.goalpost_depth                     = config["GoalpostDepth"].as<float>();
        d.goal_crossbar_width                = config["GoalCrossbarWidth"].as<float>();
        d.goal_crossbar_depth                = config["GoalCrossbarDepth"].as<float>();
        d.goal_net_height                    = config["GoalNetHeight"].as<float>();
        d.penalty_mark_distance              = config["PenaltyMarkDistance"].as<float>();
        d.center_circle_diameter             = config["CenterCircleDiameter"].as<float>();
        d.border_strip_min_width             = config["BorderStripMinWidth"].as<float>();
        d.penalty_area_length                = config["PenaltyAreaLength"].as<float>();
        d.penalty_area_width                 = config["PenaltyAreaWidth"].as<float>();

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
