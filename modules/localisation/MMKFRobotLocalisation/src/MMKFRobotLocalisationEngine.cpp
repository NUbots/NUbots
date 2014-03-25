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

#include "MMKFRobotLocalisationEngine.h"

#include "messages/vision/VisionObjects.h"
#include "utility/localisation/LocalisationFieldObject.h"

using messages::vision::VisionObject;
using utility::localisation::LocalisationFieldObject;
using utility::localisation::LFOId;

namespace modules {
namespace localisation {
    /// Integrate time-dependent observations on all objects
    void MMKFRobotLocalisationEngine::TimeUpdate(time_t current_time) {
        robot_models_.TimeUpdate();
    }

    std::vector<LocalisationFieldObject> MMKFRobotLocalisationEngine::GetPossibleObjects(
            const messages::vision::Goal& ambiguous_object) {
        std::vector<LocalisationFieldObject> possible;

        if (ambiguous_object.type == messages::vision::Goal::Type::LEFT) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYL));
        }

        if (ambiguous_object.type == messages::vision::Goal::Type::RIGHT) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBR));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYR));
        }

        if (ambiguous_object.type == messages::vision::Goal::Type::UNKNOWN) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBR));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYR));
        }

        return std::move(possible);
    }

    void MMKFRobotLocalisationEngine::ProcessAmbiguousObjects(const std::vector<messages::vision::Goal>& ambiguous_objects) {
        for (auto& ambiguous_object : ambiguous_objects) {
            // Get a vector of all field objects that the observed object could
            // possibly be
            auto possible_objects = GetPossibleObjects(ambiguous_object);
            robot_models_.AmbiguousMeasurementUpdate(ambiguous_object, possible_objects);
        }

        robot_models_.PruneModels();
    }

    void MMKFRobotLocalisationEngine::IndividualStationaryObjectUpdate(
        const std::vector<messages::vision::Goal>& goals,
        float time_increment) {

        for (auto& observed_object : goals) {

            LocalisationFieldObject actual_object;

            if (observed_object.type == messages::vision::Goal::Type::LEFT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBL);

            if (observed_object.type == messages::vision::Goal::Type::RIGHT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBR);

            robot_models_.MeasurementUpdate(observed_object, actual_object);
        }

        robot_models_.NormaliseAlphas();
    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void MMKFRobotLocalisationEngine::ProcessObjects(const std::vector<messages::vision::Goal>& goals) {
        ProcessAmbiguousObjects(goals);
    }
}
}