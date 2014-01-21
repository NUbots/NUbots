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

#include "LocalisationEngine.h"

namespace modules {
    LocalisationEngine::LocalisationEngine() {

    }

    /// Integrate time-dependent measurements on all objects
    void LocalisationEngine::TimeUpdate(time_t current_time) {
        ball_model_.TimeUpdate();

        for (auto& model : robot_models_)
            model.TimeUpdate();
    }

    /// Estimate object positions based on measurements
    void LocalisationEngine::ObjectUpdate() {

    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.

     */
    void SelfLocalisation::ProcessObjects(FieldObjects* fobs, float time_increment)
    {
        int numUpdates = 0;
        int updateResult;
        int usefulObjectCount = 0;

        // all objects at once.
        std::vector<StationaryObject*> update_objects;
        unsigned int objectsAdded = 0;
        for(auto& sfob : fobs->stationaryFieldObjects)
        {
            if(!sfob.isObjectVisible())
                continue;

            update_objects.push_back(sfob);
            objectsAdded++;
        }

        numUpdates += objectsAdded;
        usefulObjectCount += objectsAdded;
        updateResult = multipleLandmarkUpdate(update_objects);

        NormaliseAlphas();

    #if MULTIPLE_MODELS_ON
            bool blueGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() || 
                                fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible();
            bool yellowGoalSeen = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() || 
                                  fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible();
            removeAmbiguousGoalPairs(fobs->ambiguousFieldObjects, yellowGoalSeen, blueGoalSeen);
            
            for (auto& ambobj : fobs->ambiguousFieldObjects) {
                if(!ambobj.isObjectVisible()) 
                    continue;

                std::vector<int> possible_ids = ambobj.getPossibleObjectIDs();
                std::vector<StationaryObject*> poss_obj(possible_ids.size());
                for (auto& id : possible_ids)
                    poss_obj.push_back(&(fobs->stationaryFieldObjects[id]));

                updateResult = ambiguousLandmarkUpdate(ambobj, poss_obj);

                numUpdates++;
                if(ambobj.getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN || 
                   ambobj.getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                    usefulObjectCount++;

                NormaliseAlphas();
                PruneModels();
            }
    #endif // MULTIPLE_MODELS_ON

        // // Two Object update
        // //#if TWO_OBJECT_UPDATE_ON
        // StationaryObject& leftBlue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        // StationaryObject& rightBlue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        // StationaryObject& leftYellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        // StationaryObject& rightYellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];

        // if( leftBlue.isObjectVisible() and rightBlue.isObjectVisible())
        // {
        //     doTwoObjectUpdate(leftBlue, rightBlue);
        // }
        // if( leftYellow.isObjectVisible() and rightYellow.isObjectVisible())
        // {
        //     doTwoObjectUpdate(leftYellow, rightYellow);
        // }
        // //#endif

        NormaliseAlphas();
        PruneModels();

        MobileObject& ball = fobs->mobileFieldObjects[FieldObjects::FO_BALL];
        ballUpdate(ball);
    
        if (usefulObjectCount > 0)
            m_timeSinceFieldObjectSeen = 0;
        else
            m_timeSinceFieldObjectSeen += time_increment;
    }

    /*! @brief Normalises the alphas of all exisiting models.
        The alphas of all active models are normalised so that the total probablility of the set sums to 1.0.
    */
    void SelfLocalisation::NormaliseAlphas()
    {
        double sumAlpha = 0.0;
        
        for (auto& model : robot_models_)
            if (model.active())
                sumAlpha += (*model_it)->alpha();

        if(sumAlpha == 1) 
            return;

        if (sumAlpha == 0) 
            sumAlpha = 1e-12;

        for (auto& model : robot_models_)
            if (model.active())
                model.setAlpha(model.alpha() / sumAlpha);
    }

    void LocalisationEngine::landmarkUpdate(StationaryObject &landmark)
    {
        if(!landmark.validMeasurement())
            return SelfModel::RESULT_OUTLIER;

        double dist = landmark.measuredDistance();
        double elevation = landmark.measuredElevation();
        double flat_distance =  dist * cos(elevation);
        double distance_squared = flat_distance * flat_distance;
        
        MeasurementError temp_error;
        temp_error.setDistance(
            c_obj_range_offset_variance + 
            c_obj_range_relative_variance * distance_squared);
        temp_error.setHeading(c_obj_theta_variance);

        for (auto& model : robot_models_)
        {
            if(!model.active())
                continue;

            model.MeasurementUpdate(landmark, temp_error);
        }
    }

    int SelfLocalisation::multipleLandmarkUpdate(std::vector<StationaryObject*>& landmarks)
    {
        const unsigned int num_objects = landmarks.size();
        if (num_objects == 0) 
            return 0;

        Matrix locations(2 * num_objects, 1, false);
        Matrix measurements(2 * num_objects, 1, false);
        Matrix R_measurement(2 * num_objects, 2 * num_objects, false);
        std::vector<unsigned int> objIds;


        unsigned int num_measurements = 0;
        for(auto& landmark : landmarks)
        {
            const int index = 2 * num_measurements;

            if(!landmark.validMeasurement())
                continue;

            double dist = landmark.measuredDistance();
            double elevation = landmark.measuredElevation();
            double flat_distance =  dist * cos(elevation);
            double distance_squared = flat_distance * flat_distance;
            
            MeasurementError temp_error;
            temp_error.setDistance(
                c_obj_range_offset_variance + 
                c_obj_range_relative_variance * distance_squared);
            temp_error.setHeading(c_obj_theta_variance);


            // Locations
            locations[index][0] = landmark.X();
            locations[index+1][0] = landmark.Y();

            measurements[index][0] = flat_distance;
            measurements[index+1][0] = landmark.measuredBearing();

            // R
            r_measurements[index][index] = 
                c_obj_range_offset_variance + 
                c_obj_range_relative_variance * distance_squared;
            r_measurements[index+1][index+1] = c_obj_theta_variance;

            objIds.push_back(landmark.getID());

            num_measurements++;
        }

        for (auto& model : robot_models_)
        {
            if(!model.active()) 
                continue;

            model.MultipleObjectUpdate(locations, measurements, r_measurements);
        }
    }

    /*! @brief Performs an ambiguous measurement update using the exhaustive 
     *  process. 
     *  This creates a new model for each possible location for the measurement.
     */
    int SelfLocalisation::ambiguousLandmarkUpdateExhaustive(
        AmbiguousObject &ambiguousObject, 
        const vector<StationaryObject*>& possible_objects)
    {
        const float outlier_factor = 0.0001;
        ModelContainer new_models;

        MeasurementError error = calculateError(ambiguousObject);

        for (auto& model : robot_models_)
        {
            if((model.inactive())
                continue;

            unsigned int models_added = 0;
            for(auto& obj : possible_objects)
            {
                auto* temp_mod = new Model(model, ambiguousObject, obj, error, GetTimestamp());
                new_models.push_back(temp_mod);
            }

            removeInactiveModels(new_models);

            if(models_added)
            {
                model.setActive(false);
            }
            else
            {
                model.setAlpha(outlier_factor * model.alpha());
            }
        }

        if(new_models.size() > 0)
        {
            m_models.insert(m_models.end(), new_models.begin(), new_models.end());
            new_models.clear();
        }

        return SelfModel::RESULT_OUTLIER;
    }
}
