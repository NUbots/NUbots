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


#include "MultiModalRobotModel.h"

namespace modules {

/*! @brief Remove all inactive models from the default container.
 *  @retun The number of models removed.
 */
unsigned int MultiModalRobotModel::RemoveInactiveModels() {
    return RemoveInactiveModels(robot_models_);
}

/*! @brief Removes all inactive models from the given container
 *  @param container The container to remove inactive models from.
 *  @retun The number of models removed.
 */
unsigned int MultiModalRobotModel::RemoveInactiveModels(std::list<IWeightedKalmanFilter*>& container) {
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

void MultiModalRobotModel::PruneModels() {
    const float kMaxModelsAfterMerge = 5; // TODO: Add to config system

    RemoveInactiveModels();

    // if (m_settings.pruneMethod() == LocalisationSettings::prune_viterbi)
    // {
        RemoveSimilarModels();
        PruneViterbi(kMaxModelsAfterMerge);
    // }

    NormaliseAlphas();
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
    const float kMinTransDist = 6; // TODO: Add to config system
    const float kMinHeadDist = 0.01; // TODO: Add to config system

    float trans_dist = TranslationDistance(model_a->estimate(), model_b->estimate());
    float head_dist = HeadingDistance(model_a->estimate(), model_b->estimate());
    
    return (trans_dist < kMinTransDist) && (head_dist < kMinHeadDist);
}

/// Reduces the number of active models by merging similar models together
void MultiModalRobotModel::RemoveSimilarModels() {
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

/* @brief Prunes the models using the Viterbi method. This removes lower
 *        probability models to a maximum total models.
 * @param order The number of models to be kept at the end for the process.
 * @return The number of models that were removed during this process.
 */
int MultiModalRobotModel::PruneViterbi(unsigned int order) {
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

/*! @brief Normalises the alphas of all exisiting models.
    The alphas of all active models are normalised so that the total probablility of the set sums to 1.0.
*/
void MultiModalRobotModel::NormaliseAlphas() {
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

}