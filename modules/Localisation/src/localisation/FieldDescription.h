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

#ifndef MODULES_FIELDDESCRIPTION_H
#define MODULES_FIELDDESCRIPTION_H

#include <unordered_map>
#include <armadillo>

#include "messages/support/Configuration.h"
#include "LocalisationFieldObject.h"

using messages::support::Configuration;

namespace modules {
namespace localisation {

struct FieldDescriptionConfig {
    static constexpr const char* CONFIGURATION_PATH = "FieldDescription.json";
};

class FieldDescription {
public:
    FieldDescription(Configuration<FieldDescriptionConfig> config);

    void AddLFO(LocalisationFieldObject lfo);
    LocalisationFieldObject GetLFO(LFOId id);

    // Can't use unordered map without a hash function
    std::map<LFOId, LocalisationFieldObject> field_objects_;

    struct FieldDimensions {
        double line_width;
        double mark_width;
        double field_length;
        double field_width;
        double goal_depth;
        double goal_width;
        double goal_area_length;
        double goal_area_width;
        double goal_crossbar_height;
        double goalpost_diameter;
        double goal_net_height;
        double penalty_mark_distance;
        double center_circle_diameter;
        double border_strip_min_width;
    } dimensions;
};

}
}

#endif
