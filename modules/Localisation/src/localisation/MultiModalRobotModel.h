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

#ifndef MODULES_MULTIMODALROBOTMODEL_H
#define MODULES_MULTIMODALROBOTMODEL_H

#include <nuclear>
#include <armadillo>

#include "utility/math/kalman/UKF.h"

#include "RobotModel.h"
#include "messages/vision/VisionObjects.h"
#include "LocalisationFieldObject.h"

namespace modules {
namespace localisation {

    class RobotHypothesis {
    private:
        // bool active_;
        double weight_;

        utility::math::kalman::UKF<RobotModel> filter_;

        std::string obs_trail_;

    public:

        int obs_count_;

        RobotHypothesis() : 
            filter_(
                {0, 0, 3.141},
                arma::eye(RobotModel::size, RobotModel::size) * 1),
            obs_trail_(""),
            obs_count_(0),
            // active_(true), 
            weight_(1) { }

        // bool active() const { return active_; }
        // void set_active(bool active) { active_ = active; }

        float GetFilterWeight() const { return weight_; }
        void SetFilterWeight(float weight) { weight_ = weight; }

        arma::vec::fixed<RobotModel::size> GetEstimate() const {
            return filter_.get();
        }

        arma::mat::fixed<RobotModel::size, RobotModel::size> GetCovariance() const {
            return filter_.getCovariance();
        }

        double MeasurementUpdate(
            const messages::vision::VisionObject& observed_object,
            const LocalisationFieldObject& actual_object);

        void TimeUpdate();


        friend std::ostream& operator<<(std::ostream &os, const RobotHypothesis& h);
    };

    class MultiModalRobotModel {
    public:
        MultiModalRobotModel() { 
            robot_models_.push_back(std::make_unique<RobotHypothesis>());
        }

        void RemoveOldModels();

        unsigned int RemoveInactiveModels();
        unsigned int RemoveInactiveModels(std::vector<RobotHypothesis>& container);
        void PruneModels();
        void MergeSimilarModels();
        void NormaliseAlphas();

        void TimeUpdate();

        void MeasurementUpdate(
            const messages::vision::VisionObject& observed_object,
            const LocalisationFieldObject& actual_object);
        // void MultipleLandmarkUpdate();

        void AmbiguousMeasurementUpdate(
            const messages::vision::VisionObject& ambiguous_object,
            const std::vector<LocalisationFieldObject>& possible_objects);

        arma::vec::fixed<RobotModel::size> GetEstimate() {
            return robot_models_[0]->GetEstimate();
        }

        arma::mat::fixed<RobotModel::size, RobotModel::size> GetCovariance() {
            return robot_models_[0]->GetCovariance();
        }

        const std::vector<std::unique_ptr<RobotHypothesis>>& hypotheses() {
            return robot_models_;
        }


    private:
        void PruneViterbi(unsigned int order);
        // int AmbiguousLandmarkUpdateExhaustive(
        //     AmbiguousObject &ambiguous_object,
        //     const std::vector<StationaryObject*>& possible_objects);

        std::vector<std::unique_ptr<RobotHypothesis>> robot_models_;
    };
}
}
#endif


