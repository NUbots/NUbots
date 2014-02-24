/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#include "FieldDescription.h"

#include <armadillo>
#include "messages/support/Configuration.h"
#include "LocalisationFieldObject.h"

using messages::support::Configuration;

namespace modules {
namespace localisation {
    LocalisationFieldObject BuildLFO(
        Configuration<FieldDescriptionConfig> config,
        LFOId id, const std::string& name) {
        std::vector<double> v = config.config[name];
        arma::vec2 lfo_pos;
        lfo_pos << v[0] << v[1];

        return LocalisationFieldObject(lfo_pos, id, name);
    }

    void FieldDescription::AddLFO(LocalisationFieldObject lfo) {
        field_objects_[lfo.id()] = lfo;
    }

    LocalisationFieldObject FieldDescription::GetLFO(LFOId id) {
        return field_objects_.at(id);
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
        AddLFO(BuildLFO(config, LFOId::kBall, "Ball"));
        AddLFO(BuildLFO(config, LFOId::kGoalBR, "GoalBR"));
        AddLFO(BuildLFO(config, LFOId::kGoalBL, "GoalBL"));
        AddLFO(BuildLFO(config, LFOId::kGoalYR, "GoalYR"));
        AddLFO(BuildLFO(config, LFOId::kGoalYL, "GoalYL"));
        dimensions = LoadFieldDimensions(config);
    }
}
}
