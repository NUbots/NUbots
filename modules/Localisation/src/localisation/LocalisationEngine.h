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

#ifndef MODULES_LOCALISATIONENGINE_H
#define MODULES_LOCALISATIONENGINE_H

#include <nuclear>

#include "MultiModalRobotModel.h"
#include "LocalisationBall.h"
#include "messages/vision/VisionObjects.h"
#include "LocalisationFieldObject.h"
#include "FieldDescription.h"

namespace modules {
namespace localisation {

    // struct ObservationRecord {
    //     enum class ObservationType {
    //         kInvalid,
    //         kGoal,
    //     } type;

    //     std::unique_ptr<messages::vision::Goal> goal;
    // };


    class LocalisationEngine {
        public:

        LocalisationEngine();

        // void RecordMeasurement(const messages::vision::Goal& m);

        void TimeUpdate(time_t current_time);
        void ObjectUpdate();

        void SwapMeasurementBuffers();

        int ProcessAmbiguousObjects(
            std::vector<std::shared_ptr<messages::vision::VisionObject>>& fobs);
        
        void IndividualStationaryObjectUpdate(
            const std::vector<messages::vision::Goal>& goals,
            float time_increment);

        void ProcessObjects(const std::vector<messages::vision::Goal>& goals);
        
        void LandmarkUpdate(messages::vision::Goal &landmark);
        
        int multipleLandmarkUpdate(std::vector<StationaryFieldObject*>& landmarks);
        
        // MeasurementError CalculateError(const Object& theObject);
        
        // IWeightedKalmanFilter* newRobotModel(
        //     IWeightedKalmanFilter* filter, 
        //     const StationaryFieldObject& measured_object, 
        //     const MeasurementError &error,
        //     int ambiguous_id, double timestamp);
        
        // int AmbiguousLandmarkUpdateExhaustive(
        //     AmbiguousObject &ambiguous_object,
        //     const std::vector<StationaryFieldObject*>& possible_objects);

        std::shared_ptr<localisation::FieldDescription> field_description() {
            return field_description_;
        };

        void set_field_description(std::shared_ptr<localisation::FieldDescription> desc) {
            field_description_ = desc;
        };

    private:
        /// Contains the dimensions of the field
        std::shared_ptr<localisation::FieldDescription> field_description_;

        // TODO: Consider extracting the robot models into an actual class,
        // that handles the robot model without regard to whether or not
        // it is represented by a multi-modal distribution. (e.g. methods like
        // removeInactiveModels(), would be instance methods  of this new
        // class)
        MultiModalRobotModel robot_models_;

        LocalisationBall ball_model_;

        // // Should be a queue?
        // std::vector<ObservationRecord> observation_buffer_;
    };
}
}
#endif
