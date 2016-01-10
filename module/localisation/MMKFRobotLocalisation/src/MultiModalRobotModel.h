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
#include "utility/math/filter/ParticleFilter.h"
#include "utility/math/filter/UKF.h"
#include "message/support/Configuration.h"
#include "message/vision/VisionObjects.h"
#include "RobotModel.h"
#include "message/input/Sensors.h"
#include "message/localisation/ResetRobotHypotheses.h"

namespace module {
namespace localisation {

    class RobotHypothesis {
    // private:
    public: // for unit testing.
        utility::math::filter::UKF<robot::RobotModel> filter_;
        // utility::math::filter::ParticleFilter<robot::RobotModel> filter_;

        double weight_;

    public:

        // std::string obs_trail_;
        int obs_count_;

        RobotHypothesis()
            : filter_(
                {-4.5, 0, 0}, // mean
                arma::eye(robot::RobotModel::size, robot::RobotModel::size) * 0.1, // cov
                0.1) // alpha
            , weight_(1)
            , obs_count_(0) {
                arma::mat cov = arma::eye(robot::RobotModel::size, robot::RobotModel::size) * 0.1;
                cov(2,2) = 1;
                filter_.setState(arma::vec::fixed<robot::RobotModel::size>({-4.5, 0, 0}), cov);
            }

        RobotHypothesis(const message::localisation::ResetRobotHypotheses::Self& reset_self, const message::input::Sensors& sensors)
            : RobotHypothesis() {
            arma::vec2 imuDirection = arma::normalise(sensors.orientation.col(0).rows(0,1));
            double imuHeading = std::atan2(imuDirection(1), imuDirection(0));
            double imuOffset = reset_self.heading + imuHeading;

            // arma::vec3 mean = arma::join_rows(reset_self.position, arma::vec(imuOffset));
            arma::vec::fixed<robot::RobotModel::size> mean = arma::vec::fixed<robot::RobotModel::size>({reset_self.position(0), reset_self.position(1), imuOffset, 0, 0 });
            arma::mat::fixed<robot::RobotModel::size, robot::RobotModel::size> cov = arma::eye(5,5) * 0.1;
            cov.submat(0,0,1,1) = reset_self.position_cov;
            cov(2,2) = reset_self.heading_var;
            filter_.setState(mean, cov);
        }

        void SetConfig(const robot::RobotModel::Config& cfg) {
            filter_.model.cfg_ = cfg;
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
            const message::vision::Goal& observed_object,
            const utility::localisation::LocalisationFieldObject& actual_object);

        //Odometry
        double MeasurementUpdate(const message::input::Sensors& sensors);

        double MeasurementUpdate(
            const std::vector<message::vision::Goal>& observed_objects,
            const std::vector<utility::localisation::LocalisationFieldObject>& actual_objects);

        void TimeUpdate(double seconds, const message::input::Sensors& sensors);

        friend std::ostream& operator<<(std::ostream &os, const RobotHypothesis& h);
    };

    class MultiModalRobotModel {
    public:
        MultiModalRobotModel() {
            robot_models_.push_back(std::make_unique<RobotHypothesis>());
        }

        void UpdateConfiguration( const message::support::Configuration& config) {
            cfg_.merging_enabled = config["MergingEnabled"].as<bool>();
            cfg_.max_models_after_merge = config["MaxModelsAfterMerge"].as<int>();
            cfg_.merge_min_translation_dist = config["MergeMinTranslationDist"].as<float>();
            cfg_.merge_min_heading_dist = config["MergeMinHeadingDist"].as<float>();

            robot::RobotModel::Config rm_cfg;
            rm_cfg.processNoisePositionFactor = config["ProcessNoisePositionFactor"].as<double>();
            rm_cfg.processNoiseHeadingFactor = config["ProcessNoiseHeadingFactor"].as<double>();
            rm_cfg.processNoiseVelocityFactor = config["ProcessNoiseVelocityFactor"].as<double>();
            rm_cfg.observationDifferenceBearingFactor = config["ObservationDifferenceBearingFactor"].as<double>();
            rm_cfg.observationDifferenceElevationFactor = config["ObservationDifferenceElevationFactor"].as<double>();

            for (auto& model : robot_models_) {
                std::cout << __FILE__ << "," << __LINE__ << ": SEGMENTATION FAULT occurs when this cout is absent." << std::endl;
                model->SetConfig(rm_cfg);
            }
        };

        // void SensorsUpdate(const message::input::Sensors& sensors);

        void RemoveOldModels();

        unsigned int RemoveInactiveModels();
        unsigned int RemoveInactiveModels(std::vector<RobotHypothesis>& container);
        void PruneModels();
        void MergeSimilarModels();
        void NormaliseAlphas();

        bool ModelsAreSimilar(const std::unique_ptr<RobotHypothesis>& model_a,
                              const std::unique_ptr<RobotHypothesis>& model_b);

        void TimeUpdate(double seconds, const message::input::Sensors& sensors);

        void MeasurementUpdate(
            const message::vision::Goal& observed_object,
            const utility::localisation::LocalisationFieldObject& actual_object);

        void MeasurementUpdate(const message::input::Sensors& sensors);

        void MeasurementUpdate(
            const std::vector<message::vision::Goal>& observed_objects,
            const std::vector<utility::localisation::LocalisationFieldObject>& actual_objects);

        void AmbiguousMeasurementUpdate(
            const message::vision::Goal& ambiguous_object,
            const std::vector<utility::localisation::LocalisationFieldObject>& possible_objects);

        void AmbiguousMeasurementUpdate(
            const std::vector<message::vision::Goal>& ambiguous_objects,
            const std::vector<std::vector<utility::localisation::LocalisationFieldObject>>& possible_object_sets);

        void AmbiguousMultipleMeasurementUpdate(
            const std::vector<message::vision::Goal>& ambiguous_objects,
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

    public: // For unit testing
        std::vector<std::unique_ptr<RobotHypothesis>> robot_models_;

        struct {
            bool merging_enabled = true;
            int max_models_after_merge = 2;
            float merge_min_translation_dist = 0.05;
            float merge_min_heading_dist = 0.01;
        } cfg_;
    };
}
}
#endif


