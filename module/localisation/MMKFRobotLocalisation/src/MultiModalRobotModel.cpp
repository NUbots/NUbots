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

#include <iomanip>
#include <math.h>

#include "MultiModalRobotModel.h"
#include "RobotModel.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/localisation/transform.h"
#include "message/input/Sensors.h"

using message::input::Sensors;
using utility::localisation::LocalisationFieldObject;
using utility::math::coordinates::spherical2Radial;
using utility::localisation::transform::SphericalRobotObservation;

namespace module {
namespace localisation {

std::ostream & operator<<(std::ostream &os, const RobotHypothesis& h) {
    arma::vec::fixed<robot::RobotModel::size> est = h.filter_.get();

    return os
        << "{ "
        << "weight: "
            << std::setw(7) << h.weight_ << ", "
        << "estimate: ["
            << std::setw(7) << est(robot::kX) << ", "
            << std::setw(7) << est(robot::kY) << ", "
            // << std::setw(7) << est(robot::kHeadingX) << ", "
            // << std::setw(7) << est(robot::kHeadingY) << "]"
            << std::setw(7) << est(robot::kImuOffset) << "]"
        // << ", observation trail: [" << h.obs_trail_ << "]"
        << ", covariance:\n" << h.GetCovariance()
        << ", observation count: " << h.obs_count_
        << " }";
}

void MultiModalRobotModel::TimeUpdate(double seconds, const Sensors& sensors) {
    for (auto& model : robot_models_)
        model->TimeUpdate(seconds, sensors);
}
void RobotHypothesis::TimeUpdate(double seconds, const Sensors& sensors) {
    filter_.timeUpdate(seconds, sensors);
}

void MultiModalRobotModel::MeasurementUpdate(
    const message::vision::Goal& observed_object,
    const LocalisationFieldObject& actual_object) {

    for (auto& model : robot_models_)
        model->MeasurementUpdate(observed_object, actual_object);
}


void MultiModalRobotModel::MeasurementUpdate(
    const std::vector<message::vision::Goal>& observed_objects,
    const std::vector<LocalisationFieldObject>& actual_objects) {

    for (auto& model : robot_models_)
        model->MeasurementUpdate(observed_objects, actual_objects);
}

double RobotHypothesis::MeasurementUpdate(
    const message::vision::Goal& observed_object,
    const LocalisationFieldObject& actual_object) {

    double quality = 1.0;

    for (auto& measurement : observed_object.measurements) {
        // Spherical from ground:
        arma::vec3 measured_pos = measurement.position;
        arma::mat33 cov = measurement.error;

        arma::vec2 actual_2d = actual_object.location();
        arma::vec3 actual_pos = arma::vec3({actual_2d(0), actual_2d(1), 0});


        quality *= filter_.measurementUpdate(measured_pos, cov, actual_pos, *(observed_object.sensors));
    }

    return quality;
}

//Odometry
void MultiModalRobotModel::MeasurementUpdate(const Sensors& sensors){
    for (auto& model : robot_models_)
        model->MeasurementUpdate(sensors);
}

double RobotHypothesis::MeasurementUpdate(const Sensors& sensors){
    auto quality = filter_.measurementUpdate(sensors.odometry, sensors.odometryCovariance, sensors);
    return quality;
}

double RobotHypothesis::MeasurementUpdate(
    const std::vector<message::vision::Goal>& /*observed_objects (commented for warnings)*/,
    const std::vector<LocalisationFieldObject>& /*actual_objects*/) {

    std::cout << __PRETTY_FUNCTION__ << std::endl;

    // auto& obv_a = observed_objects[0];
    // auto& obv_b = observed_objects[1];
    // auto& lfo_a = actual_objects[0];
    // auto& lfo_b = actual_objects[1];

    // std::vector<arma::vec> actual_positions = {
    //     arma::vec(lfo_a.location()), arma::vec(lfo_b.location())
    // };

    // // Use a dot product to calculate heading distance:
    // arma::vec unit_a = arma::normalise(obv_a.measurements[0].position.rows(0, 1));
    // arma::vec unit_b = arma::normalise(obv_b.measurements[0].position.rows(0, 1));
    // double heading_diff = utility::math::angle::acos_clamped(arma::dot(arma::vec(unit_a), arma::vec(unit_b)));

    // arma::vec measurement = { std::abs(heading_diff) };
    // arma::mat cov = arma::eye(1, 1) * 0.1; // TODO: Calculate correct covariance

    // double quality = filter_.measurementUpdate(measurement, cov, actual_positions);

    // return quality;
    return 1;
}

// void MultiModalRobotModel::SensorsUpdate(const message::input::Sensors& sensors) {
//     for (auto& model : robot_models_)
//         model->SetSensorsData(sensors);
// }

/*! @brief Performs an ambiguous measurement update using the exhaustive
 *  process.
 *  This creates a new model for each possible location for the measurement.
 */
// For a single observation that could be one of several objects.
void MultiModalRobotModel::AmbiguousMeasurementUpdate(
    const message::vision::Goal& ambiguous_object,
    const std::vector<LocalisationFieldObject>& possible_objects) {

    std::vector<std::unique_ptr<RobotHypothesis>> new_models;
    new_models.reserve(possible_objects.size() * robot_models_.size());

    // For each model:
    while (!robot_models_.empty()) {
        auto model = std::move(robot_models_.back());
        robot_models_.pop_back();

        // Split the model for each possible object, and observe that object:
        // (TODO: Micro-optimisation: use model as the last split_model)
        for (auto& possible_object : possible_objects) {

            auto split_model = std::make_unique<RobotHypothesis>(*model);

            split_model->obs_count_++;

            auto quality = split_model->MeasurementUpdate(ambiguous_object,
                                                          possible_object);

            // Weight the new model based on the 'quality' of the observation
            // just made.
            auto weight = split_model->GetFilterWeight();
            split_model->SetFilterWeight(weight * quality);

            new_models.push_back(std::move(split_model));
        }
    }

    robot_models_ = std::move(new_models);
}


// For an ordered list of observations which is known to correspond one of
// several ordered lists of possible objects.
// Splits for each possible set, then performs a measurment update on each
// observation against each possible object within a set.
// (i.e. When two goal posts are seen simultaneously, they are either both on
// (one side of the field, or both on the other side)
void MultiModalRobotModel::AmbiguousMeasurementUpdate(
    const std::vector<message::vision::Goal>& ambiguous_objects,
    const std::vector<std::vector<LocalisationFieldObject>>& possible_object_sets) {

    std::vector<std::unique_ptr<RobotHypothesis>> new_models;
    new_models.reserve(possible_object_sets.size() * robot_models_.size());

    // For each model:
    while (!robot_models_.empty()) {
        auto model = std::move(robot_models_.back());
        robot_models_.pop_back();

        // Split the model for each possible object, and observe that object:
        // (TODO: Micro-optimisation: use model as the last split_model)
        for (auto& object_set : possible_object_sets) {
            auto split_model = std::make_unique<RobotHypothesis>(*model);

            for (int i = 0; i < int(object_set.size()); i++) {
                split_model->obs_count_++;

                auto quality = split_model->MeasurementUpdate(ambiguous_objects[i], object_set[i]);

                // Weight the new model based on the 'quality' of the observation
                // just made.
                auto weight = split_model->GetFilterWeight();
                split_model->SetFilterWeight(weight * quality);
            }
            // std::cout << "split model " << *split_model <<  std::endl;

            new_models.push_back(std::move(split_model));
        }
    }

    robot_models_ = std::move(new_models);

}

// For an ordered list of observations which is known to be correspond one of
// several ordered lists of possible objects.
// Splits for each possible set, then performs a measurment update on each set
// of observations against each set of possible objects.
// (i.e. When two goal posts are seen simultaneously, they are either both on
// (one side of the field, or both on the other side)
void MultiModalRobotModel::AmbiguousMultipleMeasurementUpdate(
    const std::vector<message::vision::Goal>& ambiguous_objects,
    const std::vector<std::vector<LocalisationFieldObject>>& possible_object_sets) {

    std::vector<std::unique_ptr<RobotHypothesis>> new_models;
    new_models.reserve(possible_object_sets.size() * robot_models_.size());

    // For each model:
    while (!robot_models_.empty()) {
        auto model = std::move(robot_models_.back());
        robot_models_.pop_back();

        // Split the model for each possible object, and observe that object:
        // (TODO: Micro-optimisation: use model as the last split_model)
        for (auto& object_set : possible_object_sets) {
            auto split_model = std::make_unique<RobotHypothesis>(*model);

            split_model->obs_count_++;

            auto quality = split_model->MeasurementUpdate(ambiguous_objects,
                                                          object_set);

            // Weight the new model based on the 'quality' of the observation
            // just made.
            auto weight = split_model->GetFilterWeight();
            split_model->SetFilterWeight(weight * quality);

            new_models.push_back(std::move(split_model));
        }
    }

    robot_models_ = std::move(new_models);
}

void MultiModalRobotModel::RemoveOldModels() {
    std::vector<std::unique_ptr<RobotHypothesis>> new_models;
    new_models.reserve(robot_models_.size());

    // For each model:
    while (!robot_models_.empty()) {
        auto model = std::move(robot_models_.back());

        if (model->obs_count_ <= 4) {
            new_models.push_back(std::move(model));
        }

        robot_models_.pop_back();
    }

    robot_models_ = std::move(new_models);
}

void MultiModalRobotModel::PruneModels() {
    if (cfg_.merging_enabled)
        MergeSimilarModels();

    PruneViterbi(cfg_.max_models_after_merge);

    NormaliseAlphas();
}

bool MultiModalRobotModel::ModelsAreSimilar(
    const std::unique_ptr<RobotHypothesis> &model_a,
    const std::unique_ptr<RobotHypothesis> &model_b) {
    arma::vec::fixed<robot::RobotModel::size> diff = model_a->GetEstimate() - model_b->GetEstimate();

    double translation_dist = arma::norm(diff.rows(0, 1), 2);

    // // Radial coords
    // auto heading_dist = std::abs(utility::math::angle::normalizeAngle(diff[kHeading]));
    auto heading_dist = diff(robot::kImuOffset);

    // Unit vector orientation
    // auto heading_dist = diff[robot::kHeadingX] + diff[robot::kHeadingY];

    return (translation_dist < cfg_.merge_min_translation_dist) &&
           (heading_dist < cfg_.merge_min_heading_dist);
}

/// Reduces the number of active models by merging similar models together
// TODO: Find a neater, yet performant way of writing this method
void MultiModalRobotModel::MergeSimilarModels() {

    // Sort models by weight from smallest to largest.
    std::sort(robot_models_.begin(), robot_models_.end(),
        [](const std::unique_ptr<RobotHypothesis> & a,
           const std::unique_ptr<RobotHypothesis> & b) {
            return a->GetFilterWeight() < b->GetFilterWeight();
        });

    std::vector<std::unique_ptr<RobotHypothesis>> new_models;
    new_models.reserve(robot_models_.size());

    // Create a bool array to indicate which models have been merged
    // into another model and should thus be ignored
    std::vector<bool> merged;
    merged.resize(robot_models_.size(), false);

    // Loop through each pair of merged models
    for (int ma = robot_models_.size() - 1; ma >= 0; ma--) {

        if (merged[ma])
            continue;

        // Compare the last model in the list to all the models before it
        for (int mb = ma - 1; mb >= 0; mb--) {
            if (merged[mb])
                continue;

            auto& model_a = robot_models_[ma];
            auto& model_b = robot_models_[mb];

            if (!ModelsAreSimilar(model_a, model_b)) {
                continue;
            }

            float wa = model_a->GetFilterWeight();
            float wb = model_b->GetFilterWeight();

            if (wa < wb) {
                merged[ma] = true;
                model_b->SetFilterWeight(wa + wb);
            } else {
                model_a->SetFilterWeight(wa + wb);
                merged[mb] = true;
            }
        }

        // If the model remains unmerged, move it to the new list
        if (!merged[ma]) {
            new_models.push_back(std::move(robot_models_[ma]));

            // Pop the now invalid last model from robot_models
            // Note: not strictly necessary
            robot_models_.pop_back();
        }
    }

    // Replace the old models with the new list of merged models
    robot_models_ = std::move(new_models);
}

/* @brief Prunes the models using the Viterbi method. This removes lower
 *        probability models to a maximum total models.
 * @param order The maximum number of models to remain after pruning.
 */
void MultiModalRobotModel::PruneViterbi(unsigned int order) {
    // No pruning required if not above maximum.

    if(robot_models_.size() <= order)
        return;

    // Sort models by weight from largest to smallest.
    std::sort(robot_models_.begin(), robot_models_.end(),
        [](const std::unique_ptr<RobotHypothesis> & a,
           const std::unique_ptr<RobotHypothesis> & b) {
            return a->GetFilterWeight() > b->GetFilterWeight();
        });

    // Keep only the desired number of elements.
    robot_models_.resize(order);
}

/*! @brief Normalises the alphas of all exisiting models.
    The alphas of all active models are normalised so that the total probablility of the set sums to 1.0.
*/
void MultiModalRobotModel::NormaliseAlphas() {
    double sumAlpha = 0.0;

    for (auto& model : robot_models_)
        sumAlpha += model->GetFilterWeight();


    // if (sumAlpha == 1)
    //     return;

    if (sumAlpha == 0)
        sumAlpha = 1e-12;

    for (auto& model : robot_models_){
        model->SetFilterWeight(model->GetFilterWeight() / sumAlpha);
        // std::cout << "model->GetFilterWeight() = " << model->GetFilterWeight() << std::endl;
    }
    // std::cout << "sumAlpha = " << sumAlpha << std::endl;

}

}
}
