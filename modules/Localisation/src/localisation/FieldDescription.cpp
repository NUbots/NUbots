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
#include <armadillo>
 
#include "messages/support/Configuration.h"
#include "localisation/FieldDescription.h"

using messages::support::Configuration;

namespace modules {
namespace localisation {

	void LocalisationFieldObject BuildLFO(
		Configuration<FieldDescriptionConfig> config,
		LFOId id, std::string name)
	{
    	arma::vec2 lfo_pos = config[name];
    	
    	auto lfo = LocalisationFieldObject(pos, id, name);
	}

    FieldDescription::FieldDescription(Configuration<FieldDescriptionConfig> config) {
    	field_objects.add(BuildLFO(config, kBall, "Ball"));
    	field_objects.add(BuildLFO(config, kGoalBR, "GoalBR"));
    	field_objects.add(BuildLFO(config, kGoalBL, "GoalBL"));
    	field_objects.add(BuildLFO(config, kGoalYR, "GoalYR"));
    	field_objects.add(BuildLFO(config, kGoalYL, "GoalYL"));
    }
}
}