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

#include "localisation/LocalisationEngine.h"

#include "messages/vision/VisionObjects.h"
#include "FieldDescription.h"
#include "LocalisationFieldObject.h"

using messages::vision::VisionObject;

namespace modules {
namespace localisation {

    LocalisationEngine::LocalisationEngine() {
    }

    // void LocalisationEngine::RecordMeasurement(const messages::vision::Goal& m) {
    //     // ObservationRecord r;

    //     // r.type = ObservationRecord::ObservationType::kGoal;
    //     // r.goal = std::move(std::make_unique<messages::vision::Goal>(m));

    //     // observation_buffer_.push_back(std::move(r));

    //     observation_buffer_.push_back(m);
    // }


    /// Integrate time-dependent observations on all objects
    void LocalisationEngine::TimeUpdate(time_t current_time) {
        ball_model_.TimeUpdate();

        robot_models_.TimeUpdate();
    }

    // /// Estimate object positions based on observations
    // void LocalisationEngine::ObjectUpdate() {
    //     ProcessObjects(observation_buffer_);
    // }

    std::vector<LocalisationFieldObject> LocalisationEngine::GetPossibleObjects(
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

    void LocalisationEngine::ProcessAmbiguousObjects(const std::vector<messages::vision::Goal>& ambiguous_objects) {
        // auto& stat_fobs = fobs->stationaryFieldObjects;
        // // Note: These lines were commented in the robocup codebase.
        // // (I forget why I ported them.  If they're unnecessary, please delete them!)
        // bool blueGoalSeen = stat_fobs[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() || 
        //                     stat_fobs[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible();
        // bool yellowGoalSeen = stat_fobs[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() || 
        //                       stat_fobs[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible();
        // RemoveAmbiguousGoalPairs(fobs->ambiguousFieldObjects, yellowGoalSeen, blueGoalSeen);
        
        for (auto& ambiguous_object : ambiguous_objects) {
            // Get a vector of all field objects that the observed object could
            // possibly be
            auto possible_objects = GetPossibleObjects(ambiguous_object);
            robot_models_.AmbiguousMeasurementUpdate(ambiguous_object, possible_objects);
        }

        robot_models_.PruneModels();
    }

    void LocalisationEngine::IndividualStationaryObjectUpdate(
        const std::vector<messages::vision::Goal>& goals,
        float time_increment)
    {
        // unsigned int objects_added = 0;
        // unsigned int total_successful_updates = 0;

        for (auto& observed_object : goals) {

            LocalisationFieldObject actual_object;

            if (observed_object.type == messages::vision::Goal::Type::LEFT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBL);

            if (observed_object.type == messages::vision::Goal::Type::RIGHT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBR);

            robot_models_.MeasurementUpdate(observed_object, actual_object);
        }

        // // if (objects_added > 0 and total_successful_updates < 1) {
        // //     total_bad_known_objects += objects_added;
        // // } else {
        // //     total_bad_known_objects = 0;
        // // }

        // // if (total_bad_known_objects > 3) {
        // //     DoReset();

        // //     // reapply the updates.
        // //     for (auto& obj : fobs->stationaryFieldObjects) {
        // //         // Skip objects that were not seen.
        // //         if (!obj.isObjectVisible())
        // //             continue;

        // //         total_successful_updates += robot_models_.MeasurementUpdate(obj);
        // //         objects_added++;
        // //     }
        // // }

        // NormaliseAlphas();
    }

 
    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void LocalisationEngine::ProcessObjects(const std::vector<messages::vision::Goal>& goals) {
        // int useful_object_count = 0;

        // Known object update
        // IndividualStationaryObjectUpdate(goals, 0);

        // // Two object update
        // if (kTwoObjectUpdateEnabled) {
        //     AttemptTwoObjectUpdate(fobs);
        // }

        // Update robot models
        // if (kMultipleModelsEnabled) { 
            // useful_object_count += ProcessAmbiguousObjects(fobs);
            ProcessAmbiguousObjects(goals);
            // PruneModels();
        // }

        // // Ball update
        // BallUpdate(fobs->mobileFieldObjects[FieldObjects::FO_BALL]);
    
        // if (useful_object_count > 0)
        //     time_since_field_object_last_seen_ = 0;
        // else
        //     time_since_field_object_last_seen_ += time_increment;
    }

    // int LocalisationEngine::multipleLandmarkUpdate(std::vector<StationaryFieldObject*>& landmarks) {
    //     const unsigned int num_objects = landmarks.size();
    //     if (num_objects == 0) 
    //         return 0;

    //     Matrix locations(2 * num_objects, 1, false);
    //     Matrix measurements(2 * num_objects, 1, false);
    //     Matrix r_measurements(2 * num_objects, 2 * num_objects, false);
    //     std::vector<unsigned int> objIds;


    //     unsigned int num_measurements = 0;
    //     for (auto& landmark : landmarks) {
    //         const int index = 2 * num_measurements;

    //         if (!landmark.validMeasurement())
    //             continue;

    //         double dist = landmark.measuredDistance();
    //         double elevation = landmark.measuredElevation();
    //         double flat_dist =  dist * cos(elevation);
    //         double flat_dist_squared = flat_dist * flat_dist;
            
    //         MeasurementError temp_error;
    //         temp_error.setDistance(
    //             kObjectRangeOffsetVariance + 
    //             kObjectRangeRelativeVariance * flat_dist_squared);
    //         temp_error.setHeading(c_obj_theta_variance);


    //         // Locations
    //         locations[index][0] = landmark.X();
    //         locations[index+1][0] = landmark.Y();

    //         measurements[index][0] = flat_dist;
    //         measurements[index+1][0] = landmark.measuredBearing();

    //         // R
    //         r_measurements[index][index] = 
    //             kObjectRangeOffsetVariance + 
    //             kObjectRangeRelativeVariance * flat_dist_squared;
    //         r_measurements[index+1][index+1] = c_obj_theta_variance;

    //         objIds.push_back(landmark.get_id());

    //         num_measurements++;
    //     }

    //     for (auto& model : robot_models_) {
    //         if (!model.active()) 
    //             continue;

    //         model.MultipleObjectUpdate(locations, measurements, r_measurements);
    //     }
    // }

    // int LocalisationEngine::AmbiguousLandmarkUpdateExhaustive(
    //     AmbiguousObject &ambiguous_object,
    //     const std::vector<StationaryFieldObject*>& possible_objects)
    // {
    //     return AmbiguousLandmarkUpdateExhaustive(ambiguous_object,
    //                                              possible_objects);
    // }

    // /// Performs a two object update if it is currently possible to do so
    // int AttemptTwoObjectUpdate(std::vector<std::shared_ptr<VisionObject>>& fobs) {
    //     // StationaryFieldObject& left_blue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
    //     // StationaryFieldObject& right_blue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
    //     // StationaryFieldObject& left_yellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
    //     // StationaryFieldObject& right_yellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];

    //     // if (left_blue.isObjectVisible() && right_blue.isObjectVisible())
    //     //     TwoObjectUpdate(left_blue, right_blue);

    //     // if (left_yellow.isObjectVisible() && right_yellow.isObjectVisible())
    //     //     TwoObjectUpdate(left_yellow, right_yellow);

    //     return 0;
    // }

}
}