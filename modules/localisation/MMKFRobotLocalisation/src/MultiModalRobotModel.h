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

#ifndef MODULES_MULTIMODALROBOTMODEL_H
#define MODULES_MULTIMODALROBOTMODEL_H

#include <armadillo>
#include <nuclear>
#include "utility/localisation/LocalisationFieldObject.h"
#include "utility/math/kalman/UKF.h"
#include "messages/support/Configuration.h"
#include "messages/vision/VisionObjects.h"
#include "RobotModel.h"
#include "messages/input/Sensors.h"

namespace modules {
namespace localisation {
    struct MultiModalRobotModelConfig {
        static constexpr const char* CONFIGURATION_PATH = "MultiModalRobotModel.yaml";
    };

    class RobotHypothesis {
    private:
        utility::math::kalman::UKF<robot::RobotModel> filter_;

        double weight_;

    public:

        // std::string obs_trail_;
        int obs_count_;

        RobotHypothesis() :
            filter_(
                {0, 0, 0}, // mean
                // {0, 0, 3.141},
                arma::eye(robot::RobotModel::size, robot::RobotModel::size) * 1, // cov
                1), // alpha
            weight_(1),
            // obs_trail_(""),
            obs_count_(0) { }

        // bool active() const { return active_; }
        // void set_active(bool active) { active_ = active; }

        void SetProcessNoiseFactor(double process_noise_factor) {
            filter_.model.processNoiseFactor = process_noise_factor;
        };

        float GetFilterWeight() const { return weight_; }
        void SetFilterWeight(float weight) { weight_ = weight; }

        arma::vec::fixed<robot::RobotModel::size> GetEstimate() const {
            return filter_.get();
        }

        arma::mat::fixed<robot::RobotModel::size, robot::RobotModel::size> GetCovariance() const {
            return filter_.getCovariance();
        }

        double MeasurementUpdate(
            const messages::vision::VisionObject& observed_object,
            const utility::localisation::LocalisationFieldObject& actual_object);

        double MeasurementUpdate(
            const std::vector<messages::vision::VisionObject>& observed_objects,
            const std::vector<utility::localisation::LocalisationFieldObject>& actual_objects);

        void TimeUpdate(double seconds);
        void TimeUpdate(double seconds, const messages::localisation::FakeOdometry& odom);
        void TimeUpdate(double seconds, const messages::input::Sensors& sensors);


        friend std::ostream& operator<<(std::ostream &os, const RobotHypothesis& h);
    };

    class MultiModalRobotModel {
    public:
        MultiModalRobotModel() :
            cfg_({ 4, 0.025, 0.01, 1e-3 }) {
            robot_models_.push_back(std::make_unique<RobotHypothesis>());
        }

        void UpdateConfiguration(
            const messages::support::Configuration<modules::localisation::MultiModalRobotModelConfig>& config) {
            cfg_.max_models_after_merge = config["MaxModelsAfterMerge"].as<int>();
            cfg_.merge_min_translation_dist = config["MergeMinTranslationDist"].as<float>();
            cfg_.merge_min_heading_dist = config["MergeMinHeadingDist"].as<float>();
            cfg_.process_noise_factor = config["ProcessNoiseFactor"].as<float>();

            for (auto& model : robot_models_)
                model->SetProcessNoiseFactor(cfg_.process_noise_factor);
        };

        void RemoveOldModels();

        unsigned int RemoveInactiveModels();
        unsigned int RemoveInactiveModels(std::vector<RobotHypothesis>& container);
        void PruneModels();
        void MergeSimilarModels();
        void NormaliseAlphas();

        bool ModelsAreSimilar(const std::unique_ptr<RobotHypothesis>& model_a,
                              const std::unique_ptr<RobotHypothesis>& model_b);

        void TimeUpdate(double seconds);
        void TimeUpdate(double seconds, const messages::localisation::FakeOdometry& odom);
        void TimeUpdate(double seconds, const messages::input::Sensors& sensors);

        void MeasurementUpdate(
            const messages::vision::VisionObject& observed_object,
            const utility::localisation::LocalisationFieldObject& actual_object);

        void MeasurementUpdate(
            const std::vector<messages::vision::VisionObject>& observed_objects,
            const std::vector<utility::localisation::LocalisationFieldObject>& actual_objects);

        void AmbiguousMeasurementUpdate(
            const messages::vision::VisionObject& ambiguous_object,
            const std::vector<utility::localisation::LocalisationFieldObject>& possible_objects);

        void AmbiguousMeasurementUpdate(
            const std::vector<messages::vision::VisionObject>& ambiguous_objects,
            const std::vector<std::vector<utility::localisation::LocalisationFieldObject>>& possible_object_sets);

        void AmbiguousMultipleMeasurementUpdate(
            const std::vector<messages::vision::VisionObject>& ambiguous_objects,
            const std::vector<std::vector<utility::localisation::LocalisationFieldObject>>& possible_object_sets);

        arma::vec::fixed<robot::RobotModel::size> GetEstimate() {
            return robot_models_[0]->GetEstimate();
        }

        arma::mat::fixed<robot::RobotModel::size, robot::RobotModel::size> GetCovariance() {
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

    public: // temporary - debugging 09/07/2014
        std::vector<std::unique_ptr<RobotHypothesis>> robot_models_;

        struct {
            int max_models_after_merge;
            float merge_min_translation_dist;
            float merge_min_heading_dist;
            float process_noise_factor;
        } cfg_;
    };
}
}
#endif


