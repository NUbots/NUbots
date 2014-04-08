/*
 * This file is part of the NUbots Codebase.
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

#include "FieldDescription.h"

#include <nuclear>
#include <armadillo>
#include "messages/support/Configuration.h"
#include "LocalisationFieldObject.h"

using messages::support::Configuration;

namespace utility {
namespace localisation {
    // LocalisationFieldObject BuildLFO(
    //     Configuration<FieldDescriptionConfig> config,
    //     LFOId id, const std::string& name) {
    //     std::vector<double> v = config.config[name];
    //     arma::vec2 lfo_pos;
    //     lfo_pos << v[0] << v[1];

    //     return LocalisationFieldObject(lfo_pos, id, name);
    // }

    void FieldDescription::AddLFO(LocalisationFieldObject lfo) {
        field_objects_[lfo.id()] = lfo;
    }

    LocalisationFieldObject FieldDescription::GetLFO(LFOId id) {
        return field_objects_.at(id);
    }

    void FieldDescription::AddGoals(FieldDescription::FieldDimensions d) {
        auto half_length = d.field_length * 0.5;
        // auto goal_line_width = d.line_width * 0.5;
        auto goal_post_radius = d.goalpost_diameter * 0.5;

        // auto goal_x = (half_length - d.line_width * 0.5 + d.goal_depth + goal_line_width * 0.5);
        auto goal_y = (d.goal_width + d.goalpost_diameter) * 0.5;
        // auto goal_w = (d.goal_depth - d.line_width + goal_line_width * 0.5);
        // auto goal_h = (d.goal_width + d.goalpost_diameter);
        auto goal_post_x = half_length + goal_post_radius * 0.5;

        arma::vec2 blue_left_goal_post = { -goal_post_x, -goal_y };
        arma::vec2 blue_right_goal_post = { -goal_post_x, goal_y };
        arma::vec2 yellow_left_goal_post = { goal_post_x, goal_y };
        arma::vec2 yellow_right_goal_post = { goal_post_x, -goal_y };

        // AddLFO(LocalisationFieldObject(config, LFOId::kBall, "Ball"));
        AddLFO(LocalisationFieldObject(blue_left_goal_post, LFOId::kGoalBL, "GoalBL"));
        AddLFO(LocalisationFieldObject(blue_right_goal_post, LFOId::kGoalBR, "GoalBR"));
        AddLFO(LocalisationFieldObject(yellow_left_goal_post, LFOId::kGoalYL, "GoalYL"));
        AddLFO(LocalisationFieldObject(yellow_right_goal_post, LFOId::kGoalYR, "GoalYR"));
    }

    FieldDescription::FieldDimensions LoadFieldDimensions(
        Configuration<FieldDescriptionConfig> config) {
        FieldDescription::FieldDimensions d;

         d.line_width = config.config["LineWidth"];
         d.mark_width = config.config["MarkWidth"];
         d.field_length = config.config["FieldLength"];
         d.field_width = config.config["FieldWidth"];
         d.goal_depth = config.config["GoalDepth"];
         d.goal_width = config.config["GoalWidth"];
         d.goal_area_length = config.config["GoalAreaLength"];
         d.goal_area_width = config.config["GoalAreaWidth"];
         d.goal_crossbar_height = config.config["GoalCrossbarHeight"];
         d.goalpost_diameter = config.config["GoalpostDiameter"];
         d.goal_net_height = config.config["GoalNetHeight"];
         d.penalty_mark_distance = config.config["PenaltyMarkDistance"];
         d.center_circle_diameter = config.config["CenterCircleDiameter"];
         d.border_strip_min_width = config.config["BorderStripMinWidth"];

        return d;
    }

    FieldDescription::FieldDescription(Configuration<FieldDescriptionConfig> config) {
        dimensions = LoadFieldDimensions(config);
        AddGoals(dimensions);

        // Log the loaded goals
        NUClear::log(__PRETTY_FUNCTION__, ": Load goal positions:");
        NUClear::log(GetLFO(LFOId::kGoalBL));
        NUClear::log(GetLFO(LFOId::kGoalBR));
        NUClear::log(GetLFO(LFOId::kGoalYL));
        NUClear::log(GetLFO(LFOId::kGoalYR));
    }
}
}
