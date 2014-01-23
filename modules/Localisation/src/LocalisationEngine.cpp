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

    /// Integrate time-dependent observations on all objects
    void LocalisationEngine::TimeUpdate(time_t current_time) {
        ball_model_.TimeUpdate();

        for (auto& model : robot_models_)
            model.TimeUpdate();
    }

    /// Estimate object positions based on observations
    void LocalisationEngine::ObjectUpdate() {

    }

    /*! @brief Removes all inactive models from the given container
     *  @param container The container to remove inactive models from.
     *  @retun The number of models removed.
     */
    unsigned int SelfLocalisation::RemoveInactiveModels(std::list<IWeightedKalmanFilter*>& container) {
        const unsigned int num_before = container.size();   // Save original size

        for (auto* model : robot_models_) {
            if (!model->active()) {
                delete model;
                model = NULL;
            }
        }

        container.erase(
            remove_if(container.begin(),
                      container.end(),
                      [](const IWeightedKalmanFilter* p) { return p == NULL; }), 
            container.end());
        
        // Return number removed: original size - new size
        return num_before - container.size();
    }

    /*! @brief Remove all inactive models from the default container.
     *  @retun The number of models removed.
     */
    unsigned int SelfLocalisation::RemoveInactiveModels() {
        return RemoveInactiveModels(robot_models_);
    }

    /* @brief Prunes the models using the Viterbi method. This removes lower
     *        probability models to a maximum total models.
     * @param order The number of models to be kept at the end for the process.
     * @return The number of models that were removed during this process.
     */
    int SelfLocalisation::PruneViterbi(unsigned int order)
    {
        RemoveInactiveModels();

        // No pruning required if not above maximum.
        if(robot_models_.size() <= order) 
            return 0;

        // Sort, results in order smallest to largest.
        robot_models_.sort(model_ptr_cmp());

        // Number of models that need to be removed.
        unsigned int num_to_remove = robot_models_.size() - order;

        // Beginning of removal range
        auto begin_remove = robot_models_.begin();
        
        // End of removal range (not removed)
        auto end_remove = robot_models_.begin();
        std::advance(end_remove, num_to_remove);

        std::for_each (
            begin_remove, 
            end_remove, 
            std::bind2nd(std::mem_fun(&IWeightedKalmanFilter::setActive), false));

        // Clear out all deactivated models.
        int num_removed = RemoveInactiveModels();

        // Result should have been achieved or something is broken.
        assert(robot_models_.size() == order);

        return num_removed;
    }

    void SelfLocalisation::PruneModels() {
        RemoveInactiveModels();

        // if (m_settings.pruneMethod() == LocalisationSettings::prune_viterbi)
        // {
            RemoveSimilarModels();
            PruneViterbi(c_MAX_MODELS_AFTER_MERGE);
        // }

        NormaliseAlphas();
    }

    int ProcessAmbiguousObjects(FieldObjects* fobs) {
        int useful_object_count = 0;

        auto& stat_fobs = fobs->stationaryFieldObjects;

        // // Note: These lines were commented in the robocup codebase.
        // // (I forget why I ported them.  If they're unnecessary, please delete them!)
        // bool blueGoalSeen = stat_fobs[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible() || 
        //                     stat_fobs[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible();
        // bool yellowGoalSeen = stat_fobs[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible() || 
        //                       stat_fobs[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible();
        // RemoveAmbiguousGoalPairs(fobs->ambiguousFieldObjects, yellowGoalSeen, blueGoalSeen);
        
        for (auto& ambobj : fobs->ambiguousFieldObjects) {
            if (!ambobj.isObjectVisible()) 
                continue;

            auto possible_ids = ambobj.getPossibleObjectIDs();
            std::vector<StationaryObject*> poss_obj(possible_ids.size());
            
            for (auto& id : possible_ids)
                poss_obj.push_back(&(stat_fobs[id]));

            AmbiguousLandmarkUpdate(ambobj, poss_obj);

            if (ambobj.getID() == FieldObjects::FO_BLUE_GOALPOST_UNKNOWN || 
                ambobj.getID() == FieldObjects::FO_YELLOW_GOALPOST_UNKNOWN)
                useful_object_count++;

            NormaliseModelAlphas();
            PruneModels();
        }

        return useful_object_count;
    }

    /// Performs a two object update if it is currently possible to do so
    int AttemptTwoObjectUpdate(FieldObjects* fobs) {
        StationaryObject& left_blue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& right_blue = fobs->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        StationaryObject& left_yellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        StationaryObject& right_yellow = fobs->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];

        if (left_blue.isObjectVisible() && right_blue.isObjectVisible())
            doTwoObjectUpdate(left_blue, right_blue);

        if (left_yellow.isObjectVisible() && right_yellow.isObjectVisible())
            doTwoObjectUpdate(left_yellow, right_yellow);
    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void SelfLocalisation::ProcessObjects(FieldObjects* fobs, 
                                          float time_increment) {
        int useful_object_count = 0;

        // Known object update
        IndividualStationaryObjectUpdate(fobs, time_increment);

        // Two object update
        if (kTwoObjectUpdateEnabled) {
            AttemptTwoObjectUpdate(fobs);
        }

        // Update robot models
        if (kMultipleModelsEnabled) {
            useful_object_count += ProcessAmbiguousObjects(fobs);
        }

        // Pruning
        NormaliseModelAlphas();
        PruneModels();

        // Ball update
        BallUpdate(fobs->mobileFieldObjects[FieldObjects::FO_BALL]);
    
        if (useful_object_count > 0)
            time_since_field_object_last_seen_ = 0;
        else
            time_since_field_object_last_seen_ += time_increment;
    }

    float TranslationDistance(const MultivariateGaussian& a, const MultivariateGaussian& b) {
        float diff_x = a.mean(RobotModel::kstates_x) - b.mean(RobotModel::kstates_x);
        float diff_y = a.mean(RobotModel::kstates_y) - b.mean(RobotModel::kstates_y);
        return sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    float HeadingDistance(const MultivariateGaussian& a, const MultivariateGaussian& b) {
        float diff_head = a.mean(RobotModel::kstates_heading) - b.mean(RobotModel::kstates_heading);
        return diff_head;
    }

    float ModelsAreSimilar(const IWeightedKalmanFilter* a, const IWeightedKalmanFilter* b) {
        const float min_trans_dist = 5; // TODO: Add to config system
        const float min_head_dist = 0.01; // TODO: Add to config system

        float trans_dist = TranslationDistance(model_a->estimate(), model_b->estimate());
        float head_dist = HeadingDistance(model_a->estimate(), model_b->estimate());
        
        return (trans_dist < min_trans_dist) && (head_dist < min_head_dist);
    }

    /// Reduces the number of active models by merging similar models together
    void SelfLocalisation::RemoveSimilarModels() {
        // Loop through each pair of active models
        for (auto* model_a : robot_models_) {
            if (!model_a->active())
                continue;

            for (auto* model_b : robot_models_) {
                if (!model_b->active())
                    continue;

                if (model_a == model_b)
                    continue;

                if (ModelsAreSimilar(model_a, model_b)) {
                    float total_alpha = model_a->getFilterWeight() + model_b->getFilterWeight();
                    
                    if (model_a->getFilterWeight() < model_b->getFilterWeight()) {
                        model_a->setActive(false);
                        model_b->setFilterWeight(total_alpha);
                    } else {
                        model_a->setFilterWeight(total_alpha);
                        model_b->setActive(false);
                    }
                }
            }
        }

        RemoveInactiveModels();

        NormaliseModelAlphas();
    }


    /*! @brief Normalises the alphas of all exisiting models.
        The alphas of all active models are normalised so that the total probablility of the set sums to 1.0.
    */
    void SelfLocalisation::NormaliseModelAlphas() {
        double sumAlpha = 0.0;
        
        for (auto& model : robot_models_)
            if (model.active())
                sumAlpha += (*model_it)->alpha();

        if (sumAlpha == 1) 
            return;

        if (sumAlpha == 0) 
            sumAlpha = 1e-12;

        for (auto& model : robot_models_)
            if (model.active())
                model.setAlpha(model.alpha() / sumAlpha);
    }

    void LocalisationEngine::landmarkUpdate(StationaryObject &landmark) {
        if (!landmark.validMeasurement())
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

        for (auto& model : robot_models_) {
            if(!model.active())
                continue;

            model.MeasurementUpdate(landmark, temp_error);
        }
    }

    int SelfLocalisation::multipleLandmarkUpdate(std::vector<StationaryObject*>& landmarks) {
        const unsigned int num_objects = landmarks.size();
        if (num_objects == 0) 
            return 0;

        Matrix locations(2 * num_objects, 1, false);
        Matrix measurements(2 * num_objects, 1, false);
        Matrix R_measurement(2 * num_objects, 2 * num_objects, false);
        std::vector<unsigned int> objIds;


        unsigned int num_measurements = 0;
        for (auto& landmark : landmarks) {
            const int index = 2 * num_measurements;

            if (!landmark.validMeasurement())
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

        for (auto& model : robot_models_) {
            if (!model.active()) 
                continue;

            model.MultipleObjectUpdate(locations, measurements, r_measurements);
        }
    }

    int SelfLocalisation::AmbiguousLandmarkUpdateExhaustive(
        AmbiguousObject &ambiguous_object,
        const std::vector<StationaryObject*>& possible_objects)
    {
        return AmbiguousLandmarkUpdateExhaustive(ambiguous_object,
                                                 possible_objects);
    }

    /*!
     * @brief Calculate the variance in the given measurement based on the objects distance.
     * @param theObject The object the error variance is to be calculated for.
     * @return The error of the measurement given as the variance.
     */
    MeasurementError SelfLocalisation::CalculateError(const Object& theObject)
    {
        MeasurementError error;

        float flat_dist = theObject.measuredDistance() * 
                          cos(theObject.measuredElevation());
        float flat_dist_sqr = flat_dist * flat_dist;

        error.setDistance(c_obj_range_offset_variance + 
                          c_obj_range_relative_variance * flat_dist_sqr);
        error.setHeading(c_obj_theta_variance);

        return error;
    }

    IWeightedKalmanFilter* SelfLocalisation::newRobotModel(
        IWeightedKalmanFilter* filter, 
        const StationaryObject& measured_object, 
        const MeasurementError &error,
        int ambiguous_id, double timestamp)
    {
        Matrix meas_noise = error.errorCovariance();

        IWeightedKalmanFilter* new_filter = filter->Clone();
        new_filter->AssignNewId();  // update with a new ID.

        Matrix meas(2, 1, false);
        meas[0][0] = measured_object.measuredDistance() * cos(measured_object.measuredElevation());
        meas[1][0] = measured_object.measuredBearing();

        Matrix args(2,1,false);
        args[0][0] = measured_object.X();
        args[1][0] = measured_object.Y();

        bool success = new_filter->measurementUpdate(meas, meas_noise, args,
                                                     RobotModel::klandmark_measurement);
        new_filter->setActive(success);

        if(new_filter->active())
        {
            new_filter->m_creation_time = timestamp;
            new_filter->m_parent_history_buffer = filter->m_parent_history_buffer;
            new_filter->m_parent_history_buffer.push_back(filter->id());
            new_filter->m_parent_id = filter->id();
            new_filter->m_split_option = measured_object.getID();
            new_filter->m_previous_decisions = filter->m_previous_decisions;
            new_filter->m_previous_decisions[ambiguous_id] = measured_object.getID();
        }

        return new_filter;
    }

    /*! @brief Performs an ambiguous measurement update using the exhaustive 
     *  process. 
     *  This creates a new model for each possible location for the measurement.
     */
    int SelfLocalisation::AmbiguousLandmarkUpdateExhaustive(
        AmbiguousObject &ambiguous_object,
        const std::vector<StationaryObject*>& possible_objects)
    {
        const float outlier_factor = 0.001; // TODO: Add to config system
        std::list<IWeightedKalmanFilter*> new_models;

        MeasurementError error = CalculateError(ambiguous_object);

        for (auto* model : robot_models_) {
            if (!model->active())
                continue;

            unsigned int models_added = 0;

            for (StationaryObject* possible_object : possible_objects) {
                auto temp_object = *possible_object;
                temp_object.CopyMeasurement(ambiguous_object);
                
                auto* temp_mod = newRobotModel(model, temp_object, error, 
                                               ambiguous_object.getID(), 
                                               GetTimestamp());
                
                new_models.push_back(temp_mod);

                if(temp_mod->active())
                    models_added++;
            }

            RemoveInactiveModels(new_models);

            if (models_added)
                model->setActive(false);
            else
                model->setmodelWeight(outlier_factor * model->getFilterWeight());
        }

        if (new_models.size() > 0) {
            robot_models_.insert(robot_models_.end(), new_models.begin(), new_models.end());
            new_models.clear();
        }

        RemoveInactiveModels();

        return 0;
    }
}
