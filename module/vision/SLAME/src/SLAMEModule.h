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
#ifndef MODULES_VISION_SLAME_MODULE_H
#define MODULES_VISION_SLAME_MODULE_H

#include <nuclear>
#include "message/vision/VisionObjects.h"
#include "message/input/Image.h"
#include "message/localisation/FieldObject.h"
#include "message/input/Sensors.h"
#include "extension/Configuration.h"
#include "utility/math/filter/UKF.h"
#include "utility/math/vision.h"
#include "utility/math/angle.h"
#include "utility/math/kalman/InverseDepthPointModel.h"
#include <algorithm>

namespace module{
    namespace vision{

        template <class FeatureDetectorClass>
        class SLAMEModule{
        private:
            std::vector<float> featureStrengths;
            std::vector<typename FeatureDetectorClass::ExtractedFeature> features;
            std::vector<utility::math::filter::UKF<utility::math::filter::InverseDepthPointModel>> featureFilters;

            static constexpr size_t MODEL_SIZE = utility::math::kalman::InverseDepthPointModel::size;
            using StateVector = arma::vec::fixed<MODEL_SIZE>;
            using CovarianceMatrix = arma::mat::fixed<MODEL_SIZE, MODEL_SIZE>;
            using stateOf = utility::math::kalman::IDPStateComponents;

            //Testing with mock
            std::vector<typename FeatureDetectorClass::MockFeature> knownFeatures;

            FeatureDetectorClass featureExtractor;
            size_t MAX_MATCHES;
            float MEASUREMENT_COV_FACTOR = 0.0174;

            float RHO_INITIAL = 0.1;
            float RHO_COV_INITIAL = 0.5;
            float ANGULAR_COVARIANCE = 0.0174 * 0.0174;

            float FOV_X = 1.0472;
            float FOV_Y = 0.785398;

            int MIN_MEASUREMENTS_FOR_THROWOUT = 5;
            int THRESHOLD_FOR_THROWOUTS = 10;

            NUClear::clock::time_point lastTime;

        public:
            SLAMEModule():featureExtractor()
            {
                lastTime = NUClear::clock::now();
            }
            void setParameters(const extension::Configuration& config){
                MAX_MATCHES = config["MAX_MATCHES"].template as<size_t>();
                MEASUREMENT_COV_FACTOR = config["MEASUREMENT_COV_FACTOR"].template as<float>();
                RHO_INITIAL = config["RHO_INITIAL"].template as<float>();
                RHO_COV_INITIAL = config["RHO_COV_INITIAL"].template as<float>();
                ANGULAR_COVARIANCE = config["ANGULAR_COVARIANCE"].template as<float>();

                FOV_X = config["FOV_X"].template as<float>();
                FOV_Y = config["FOV_Y"].template as<float>();

                MIN_MEASUREMENTS_FOR_THROWOUT = config["MIN_MEASUREMENTS_FOR_THROWOUT"].template as<int>();
                THRESHOLD_FOR_THROWOUTS = config["THRESHOLD_FOR_THROWOUTS"].template as<int>();

                knownFeatures = featureExtractor.setParameters(config);
            }

            arma::mat getMeasurementCovariance(){
                return arma::eye(2,2) * MEASUREMENT_COV_FACTOR;
            }

            /*!
                @param Rwc is the worldToCamera transform
            */
            StateVector getInitialMean(arma::vec screenAngular, arma::mat Rwc, const message::localisation::Self& self, const message::input::Sensors& sensors){
                StateVector v;
                arma::mat Rcw = utility::math::matrix::orthonormal44Inverse(Rwc);
                v.rows(stateOf::kX,stateOf::kZ) = Rcw.submat(0,3,2,3);
                v[stateOf::kRHO] = RHO_INITIAL;
                arma::vec worldAngular = utility::math::vision::rotateAngularDirection(screenAngular, Rcw);
                v[stateOf::kTHETA] = worldAngular[0];
                v[stateOf::kPHI] = worldAngular[1];
                return v;
            }

            CovarianceMatrix getInitialCovariance(arma::vec screenAngular, arma::mat Rwc, const message::localisation::Self& self, const message::input::Sensors& sensors){
                CovarianceMatrix M = arma::eye(MODEL_SIZE, MODEL_SIZE);
                M(stateOf::kX, stateOf::kX) = self.sr_xx;
                M(stateOf::kY, stateOf::kX) = self.sr_xy;
                M(stateOf::kX, stateOf::kY) = self.sr_xy;
                M(stateOf::kY, stateOf::kY) = self.sr_yy;
                M(stateOf::kZ, stateOf::kZ) = 0.1*0.1; //std dev = 10cm
                M(stateOf::kRHO, stateOf::kRHO) = RHO_COV_INITIAL;
                M(stateOf::kTHETA, stateOf::kTHETA) = ANGULAR_COVARIANCE;
                M(stateOf::kPHI, stateOf::kPHI) = ANGULAR_COVARIANCE;
                return M;
            }

            arma::vec calculateErrors(){
                arma::running_stat<double> euclidean_errors;
                arma::running_stat<double> global_bearing_errors;
                arma::running_stat<double> strength_errors;
                for(int i = 0; i < featureFilters.size(); i++){
                    auto& f = featureFilters[i];
                    int featureID = features[i].featureID;
                    arma::vec state = f.get();
                    if(featureID<=knownFeatures.size()){
                        if(state[stateOf::kRHO] > 0){

                            arma::vec knownPosition = knownFeatures[featureID-1].position.rows(0,2);
                            float knownStrength = (1-knownFeatures[featureID-1].FALSE_NEGATIVE_PROB)*(1-knownFeatures[featureID-1].MISCLASSIFIED_PROB);

                            arma::vec p = utility::math::kalman::InverseDepthPointModel::getFieldPosFromState(state).rows(0,2);
                            euclidean_errors(double(arma::norm(p-knownPosition)));

                            float measured_global_bearing = std::atan2(p[1],p[0]);
                            float known_global_bearing = std::atan2(knownPosition[1],knownPosition[0]);
                            global_bearing_errors(double(std::fabs(utility::math::angle::normalizeAngle(measured_global_bearing - known_global_bearing))));

                            strength_errors(std::fabs(knownStrength-featureStrengths[i]));
                        }
                    }
                }
                return {euclidean_errors.max(),euclidean_errors.mean(),euclidean_errors.min(),
                        global_bearing_errors.max(),global_bearing_errors.mean(),global_bearing_errors.min(),
                        strength_errors.max(),strength_errors.mean(),strength_errors.min()};
            }

            std::vector<bool> getExpectations(arma::mat worldToCameraTransform){
                std::vector<bool> expectations;
                for (auto& filter : featureFilters){
                    arma::vec screenAngular = utility::math::kalman::InverseDepthPointModel::predictedObservation(filter.get(), worldToCameraTransform);
                    expectations.push_back(std::fabs(screenAngular[0]) < FOV_X/2.0 && std::fabs(screenAngular[1]) < FOV_Y/2.0);
                }
                return expectations;
            }

            void sortAndTruncateFeatureList(){
                //sort: EWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW

                std::vector<float> sortedStrengths(featureStrengths);
                std::sort(sortedStrengths.begin(),sortedStrengths.end());
                NUClear::log("strengths sortedStrengths");

                for(int i = 0; i<featureStrengths.size(); i++){
                    NUClear::log(featureStrengths[i], sortedStrengths[i]);
                    // Do not truncate if not enough measurements performed on each feature
                    if(features[i].numberOfTimesUpdated < MIN_MEASUREMENTS_FOR_THROWOUT) return;
                }

                float cutoffStrength = sortedStrengths[std::max(0, std::min(int(sortedStrengths.size() - THRESHOLD_FOR_THROWOUTS), int(sortedStrengths.size()-1)))];

                if(featureStrengths.size() != featureFilters.size() || features.size() != featureFilters.size()){
                    NUClear::log("filterStrengths.size() != featureFilters.size() || features.size() != featureFilters.size()");
                    throw std::exception();
                }
                //remove weak features
                int index = 0;
                while(index < featureStrengths.size()){
                    if(featureStrengths[index] < cutoffStrength){
                        NUClear::log("removing feature:", features[index].featureID, "strength", featureStrengths[index], " < ", cutoffStrength, ", N = ", features[index].numberOfTimesUpdated);
                        featureStrengths.erase(featureStrengths.begin() + index);
                        featureFilters.erase(featureFilters.begin() + index);
                        features.erase(features.begin() + index);
                    } else {
                        index++;
                    }
                }
                // END EWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW
            }

            std::unique_ptr<std::vector<message::vision::SLAMEObject>> getSLAMEObjects(const message::input::Image& image,
                                                                                        const message::localisation::Self& self,
                                                                                        const message::input::Sensors& sensors)
            {

                arma::mat worldToCameraTransform = utility::math::vision::calculateWorldToCameraTransform(sensors, self);

                auto objectMessage = std::make_unique<std::vector<message::vision::SLAMEObject>>();

                std::vector<typename FeatureDetectorClass::ExtractedFeature> extractedFeatures = featureExtractor.extractFeatures(image, self, sensors);

                //Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
                //Order of vector is strongest to weakest
                //Add new features here to the feature list and pick up missing filters and strengths below
                std::vector<std::tuple<int, int, float>> matches = featureExtractor.matchFeatures(features, extractedFeatures, MAX_MATCHES);

                double deltaT = (sensors.timestamp - lastTime).count() / double(NUClear::clock::period::den);

                std::vector<bool> expectations = getExpectations(worldToCameraTransform);

                for(auto& match : matches){


                    int fI; //featureIndex
                    int eFI; //extractedFeatureIndex
                    float strength;
                    std::tie(fI, eFI, strength) = match;

                    if(fI < featureFilters.size()){
                        //That is, we have seen this object before
                        //Create message about where we have seen the feature
                        objectMessage->push_back(message::vision::SLAMEObject());

                        objectMessage->back().expectedState = featureFilters[fI].get();
                        objectMessage->back().screenAngular = extractedFeatures[eFI].screenAngular;
                        objectMessage->back().screenPosition = extractedFeatures[eFI].screenPosition;
                        objectMessage->back().strength = featureStrengths[fI];
                        objectMessage->back().timestamp = sensors.timestamp;

                        //Update our beleif
                        //Strength updated with total average model

                        featureStrengths[fI] = ( features[fI].numberOfTimesUpdated * featureStrengths[fI] + strength * int(expectations[fI]) ) / (features[fI].numberOfTimesUpdated+1);
                        features[fI].numberOfTimesUpdated++;
                        expectations[fI] = false;
                        featureFilters[fI].timeUpdate(deltaT);
                        featureFilters[fI].measurementUpdate(extractedFeatures[eFI].screenAngular, getMeasurementCovariance(),worldToCameraTransform);
                    } else {    //Otherwise we initialise a filter for the object
                        featureStrengths.push_back(strength);
                        expectations.push_back(false);
                        auto initialMean = getInitialMean(extractedFeatures[eFI].screenAngular, worldToCameraTransform, self, sensors);
                        auto initialCovariance = getInitialCovariance(extractedFeatures[eFI].screenAngular, worldToCameraTransform, self, sensors);
                        featureFilters.push_back(utility::math::kalman::UKF<utility::math::kalman::InverseDepthPointModel>(initialMean, initialCovariance));
                    }
                }

                for(int i = 0; i < featureStrengths.size(); i++){
                    if(expectations[i]){    //Expected but not seen
                        featureStrengths[i] = features[i].numberOfTimesUpdated * featureStrengths[i] / (features[i].numberOfTimesUpdated+1);
                        features[i].numberOfTimesUpdated++;
                    }
                }

                // NUClear::log("Extrated features:", extractedFeatures.size());
                // NUClear::log("Matches:", matches.size());
                // NUClear::log("number of strengths", featureStrengths.size());
                // NUClear::log("Number of features:", features.size());
                // NUClear::log("Feature filter states:", featureFilters.size());
                std::cerr << "====================================================" << std::endl;
                std::cerr << calculateErrors() << std::endl;
                std::cerr << "====================================================" << std::endl;
                std::cout << "====================================================" << std::endl;
                for(int i = 0; i < featureFilters.size(); i++){
                    auto& f = featureFilters[i];
                    arma::vec state = f.get();
                    std::cout << features[i].featureID << " ";
                    if(state[stateOf::kRHO] > 0){
                        arma::vec p = utility::math::kalman::InverseDepthPointModel::getFieldPosFromState(state);
                        for(auto coord : p){
                            std::cout << coord << " ";
                        }
                    }
                    std::cout << featureStrengths[i] << " " << features[i].numberOfTimesUpdated;
                    std::cout << std::endl;
                }
                if(featureStrengths.size() > THRESHOLD_FOR_THROWOUTS){
                   sortAndTruncateFeatureList();
                }
                std::cout << "====================================================" << std::endl;
                lastTime = sensors.timestamp;

                return std::move(objectMessage);
            }

        };
    }
 }

 #endif