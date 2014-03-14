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

#include <iomanip>

#include "MultiModalRobotModel.h"
#include "RobotModel.h"
#include "LocalisationFieldObject.h"
#include "utility/math/angle.h"

namespace modules {
namespace localisation {

std::ostream & operator<<(std::ostream &os, const RobotHypothesis& h) {
    arma::vec::fixed<RobotModel::size> est = h.filter_.get();

    return os 
        << "{ "
        << "weight: "
            << std::setw(7) << h.weight_ << ", "
        << "estimate: ["
            << std::setw(7) << est[kX] << ", "
            << std::setw(7) << est[kY] << ", "
            << std::setw(7) << est[kHeadingX] << ", "
            << std::setw(7) << est[kHeadingY] << "]"
        // << ", observation trail: [" << h.obs_trail_ << "]"
        // << ", covariance:\n" << h.GetCovariance()
        << ", observation count: " << h.obs_count_
        << " }";
}

void MultiModalRobotModel::TimeUpdate() {

    // robot_models_ = std::vector<std::unique_ptr<RobotHypothesis>>();
    // robot_models_.push_back(std::make_unique<RobotHypothesis>());

    for (auto& model : robot_models_)
        model->TimeUpdate();
}


void RobotHypothesis::TimeUpdate() {
    arma::vec3 tmp = { 0, 0, 0 };
    filter_.timeUpdate(0.1, tmp);
}


void MultiModalRobotModel::MeasurementUpdate(
    const messages::vision::VisionObject& observed_object,
    const LocalisationFieldObject& actual_object) {

    for (auto& model : robot_models_)
        model->MeasurementUpdate(observed_object, actual_object);
}


double RobotHypothesis::MeasurementUpdate(
    const messages::vision::VisionObject& observed_object,
    const LocalisationFieldObject& actual_object) {

    obs_trail_ += actual_object.name() + " ";

    // // Radial coordinates
    // arma::vec2 actual_pos = actual_object.location();
    // arma::vec2 measurement = { observed_object.sphericalFromNeck[0],
    //                            observed_object.sphericalFromNeck[1] };
    // arma::mat22 cov = { observed_object.sphericalError[0], 0,
    //                     0, observed_object.sphericalError[1] };

    // Unit vector orientation
    arma::vec2 actual_pos = actual_object.location();
    arma::vec3 measurement = { observed_object.sphericalFromNeck[0],
                               std::cos(observed_object.sphericalFromNeck[1]),
                               std::sin(observed_object.sphericalFromNeck[1]) };
    arma::mat33 cov = { observed_object.sphericalError[0], 0, 0,
                        0, observed_object.sphericalError[1], 0,
                        0, 0, observed_object.sphericalError[1] };

    double quality = filter_.measurementUpdate(measurement, cov, actual_pos);

    return quality;
}


/*! @brief Performs an ambiguous measurement update using the exhaustive 
 *  process. 
 *  This creates a new model for each possible location for the measurement.
 */
void MultiModalRobotModel::AmbiguousMeasurementUpdate(
    const messages::vision::VisionObject& ambiguous_object,
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

        // auto model1 = std::make_unique<RobotHypothesis>();
        // auto model2 = std::make_unique<RobotHypothesis>(*model1);
        // auto quality1 = model1->MeasurementUpdate(ambiguous_object, possible_objects[0]);
        // auto quality2 = model2->MeasurementUpdate(ambiguous_object, possible_objects[1]);
        // model1->SetFilterWeight(model1->GetFilterWeight() * quality1);
        // model2->SetFilterWeight(model2->GetFilterWeight() * quality2);
        // new_models.push_back(std::move(model1));
        // new_models.push_back(std::move(model2));
    }

    robot_models_ = std::move(new_models);

    // Sort models by weight from largest to smallest.
    std::sort(robot_models_.begin(), robot_models_.end(), 
        [](const std::unique_ptr<RobotHypothesis> & a, 
           const std::unique_ptr<RobotHypothesis> & b) { 
            return a->GetFilterWeight() > b->GetFilterWeight();
        });
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
    const float kMaxModelsAfterMerge = 10; // TODO: Add to config system

    // NUClear::log(__PRETTY_FUNCTION__, "Number of models before merging: ",
    //                      robot_models_.size());

    MergeSimilarModels();

    // NUClear::log(__PRETTY_FUNCTION__, "Number of models before pruning: ",
    //                      robot_models_.size());

    // RemoveOldModels();

    PruneViterbi(kMaxModelsAfterMerge);

    NormaliseAlphas();
}

bool ModelsAreSimilar(const RobotHypothesis& model_a,
                      const RobotHypothesis& model_b) {
    const float kMinTransDist = 0.06; // TODO: Add to config system
    const float kMinHeadDist = 0.01; // TODO: Add to config system

    arma::vec::fixed<RobotModel::size> diff = model_a.GetEstimate() - model_b.GetEstimate();

    auto trans_dist = arma::norm(diff.rows(0, 1), 2);

    // // Radial coords
    // auto head_dist = std::abs(utility::math::angle::normalizeAngle(diff[kHeading]));

    // Unit vector orientation
    auto head_dist = diff[kHeadingX] + diff[kHeadingY];

    return (trans_dist < kMinTransDist) && (head_dist < kMinHeadDist);
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

            if (!ModelsAreSimilar(*model_a, *model_b))
                continue;

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

// /// Reduces the number of active models by merging similar models together
// void MultiModalRobotModel::MergeSimilarModels() {
//     // Loop through each pair of active models
//     for (auto& model_a : robot_models_) {
//         if (!model_a.active())
//             continue;

//         for (auto& model_b : robot_models_) {
//             if (!model_b.active())
//                 continue;

//             if (&model_a == &model_b)
//                 continue;

//             if (ModelsAreSimilar(model_a, model_b)) {
//                 float total_alpha = model_a.GetFilterWeight() + model_b.GetFilterWeight();
                
//                 if (model_a.GetFilterWeight() < model_b.GetFilterWeight()) {
//                     model_a.set_active(false);
//                     model_b.SetFilterWeight(total_alpha);
//                 } else {
//                     model_a.SetFilterWeight(total_alpha);
//                     model_b.set_active(false);
//                 }
//             }
//         }
//     }

//     RemoveInactiveModels();

//     NormaliseAlphas();
// }

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

    for (auto& model : robot_models_)
            model->SetFilterWeight(model->GetFilterWeight() / sumAlpha);
}

}
}
