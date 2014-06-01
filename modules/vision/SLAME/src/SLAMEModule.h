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
#include "messages/vision/VisionObjects.h"
#include "messages/input/Image.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "utility/math/kalman/UKF.h"
#include "utility/math/vision.h"
#include "utility/math/matrix.h"
#include "utility/math/kalman/InverseDepthPointModel.h"


namespace modules{
	namespace vision{

	    

 		template <class FeatureDetectorClass>
 		class SLAMEModule{
 		private:

 			std::vector<float> featureStrengths;
            std::vector<typename FeatureDetectorClass::ExtractedFeature> features;
            std::vector<utility::math::kalman::UKF<utility::math::kalman::InverseDepthPointModel>> featureFilters;
            
            static constexpr size_t MODEL_SIZE = utility::math::kalman::InverseDepthPointModel::size;
            using StateVector = arma::vec::fixed<MODEL_SIZE>;
            using CovarianceMatrix = arma::mat::fixed<MODEL_SIZE, MODEL_SIZE>;
            using stateOf = utility::math::kalman::IDPStateComponents;

            FeatureDetectorClass featureExtractor;
            size_t MAX_MATCHES;
            float MEASUREMENT_COV_FACTOR = 0.0174;

            float RHO_INITIAL = 0.1;
			float RHO_COV_INITIAL = 0.5;
			float ANGULAR_COVARIANCE = 0.0174 * 0.0174;

            NUClear::clock::time_point lastTime;

 		public:
 		 	SLAMEModule():featureExtractor()
 		 	{
 		 		lastTime = NUClear::clock::now();
 		 	}
 		 	void setParameters(const messages::support::Configuration<FeatureDetectorClass>& config){
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 		MAX_MATCHES = config["MAX_MATCHES"];
 		 		MEASUREMENT_COV_FACTOR = config["MEASUREMENT_COV_FACTOR"];
 		 		RHO_INITIAL = config["RHO_INITIAL"];
 		 		RHO_COV_INITIAL = config["RHO_COV_INITIAL"];
 		 		ANGULAR_COVARIANCE = config["ANGULAR_COVARIANCE"];
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 		featureExtractor.setParameters(config);
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 	}

 		 	arma::mat getMeasurementCovariance(){
 		 		return arma::eye(2,2) * MEASUREMENT_COV_FACTOR;
 		 	}

 		 	/*!
 		 		@param Rwc is the worldToCamera transform
 		 	*/
 		 	StateVector getInitialMean(arma::vec2 screenAngular, arma::mat44 Rwc, const messages::localisation::Self& self, const messages::input::Sensors& sensors){
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 		StateVector v;
 		 		arma::mat44 Rcw = utility::math::matrix::orthonormal44Inverse(Rwc);
 		 		v.rows(stateOf::kX,stateOf::kZ) = Rcw.submat(0,3,2,3);
 		 		v[stateOf::kRHO] = RHO_INITIAL;
 		 		arma::vec2 worldAngular = utility::math::vision::rotateAngularDirection(screenAngular, Rcw);
 		 		v[stateOf::kTHETA] = worldAngular[0];
 		 		v[stateOf::kPHI] = worldAngular[1];
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 		return v;
 		 	}

			CovarianceMatrix getInitialCovariance(arma::vec2 screenAngular, arma::mat44 Rwc, const messages::localisation::Self& self, const messages::input::Sensors& sensors){
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
				CovarianceMatrix M;
				double max_pos_variance = std::max(self.sr_xx, std::max(self.sr_xy,self.sr_yy));
				M(stateOf::kX, stateOf::kX) = self.sr_xx;
				M(stateOf::kY, stateOf::kX) = self.sr_xy;
				M(stateOf::kX, stateOf::kY) = self.sr_xy;
				M(stateOf::kY, stateOf::kY) = self.sr_yy;
				M(stateOf::kZ, stateOf::kZ) = 0.1*0.1; //std dev = 10cm
				M(stateOf::kRHO, stateOf::kRHO) = RHO_COV_INITIAL;
				M(stateOf::kTHETA, stateOf::kTHETA) = ANGULAR_COVARIANCE;
				M(stateOf::kPHI, stateOf::kPHI) = ANGULAR_COVARIANCE;
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
				return M;
			}

 		 	std::unique_ptr<std::vector<messages::vision::SLAMEObject>> getSLAMEObjects(const messages::input::Image& image, 
 		 		                                                                        const messages::localisation::Self& self, 
 		 		                                                                        const messages::input::Sensors& sensors)
 		 	{
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
 		 		arma::mat44 worldToCameraTransform = utility::math::vision::calculateWorldToCameraTransform(sensors, self);

 		 		auto objectMessage = std::make_unique<std::vector<messages::vision::SLAMEObject>>();	            

	            std::vector<typename FeatureDetectorClass::ExtractedFeature> extractedFeatures = featureExtractor.extractFeatures(image, self, sensors);

	            //Get matches: tuple = (featureIndex in featureFilters, extractedFeatureIndex in extractedFeatures, matchStrength)
	            //Order of vector is strongest to weakest
	            //Add new features here to the feature list and pick up missing filters and strengths below
	            std::vector<std::tuple<int, int, float>> matches = featureExtractor.matchFeatures(features, extractedFeatures, MAX_MATCHES);

	            double deltaT = (sensors.timestamp - lastTime).count() / double(NUClear::clock::period::den);
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);

	            for(auto& match : matches){

                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
	            	int fI = std::get<0>(match);	//featureIndex
	            	int eFI = std::get<1>(match);	//extractedFeatureIndex
	            	float strength = std::get<2>(match);

	                if(fI < featureFilters.size()){     
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
	                	//That is, we have seen this object before
	                    //Create message about where we have seen the feature
	                    objectMessage->push_back(messages::vision::SLAMEObject());

	                    objectMessage->back().expectedState = featureFilters[fI].get();
	                    objectMessage->back().screenAngular = extractedFeatures[eFI].screenAngular;
	                    objectMessage->back().strength = featureStrengths[fI];
	                    objectMessage->back().timestamp = sensors.timestamp;

	                    //Update our beleif
	                    featureStrengths[fI] += strength;	//TODO make this better and use strengths
	                    featureFilters[fI].timeUpdate(deltaT, int(0));
	                    featureFilters[fI].measurementUpdate(extractedFeatures[eFI].screenAngular, getMeasurementCovariance(),worldToCameraTransform);
	                } else {    //Otherwise we initialise a filter for the object
	                    featureStrengths.push_back(strength);

                NUClear::log(__PRETTY_FUNCTION__,__LINE__);
	                    auto initialMean = getInitialMean(extractedFeatures[eFI].screenAngular, worldToCameraTransform, self, sensors);
	                    auto initialCovariance = getInitialCovariance(extractedFeatures[eFI].screenAngular, worldToCameraTransform, self, sensors);
	                    featureFilters.push_back(utility::math::kalman::UKF<utility::math::kalman::InverseDepthPointModel>(initialMean, initialCovariance));
	                }
	            }
	            //TODO: sort objectMessage by strengths and take top k
	            lastTime = sensors.timestamp;
                NUClear::log(__PRETTY_FUNCTION__,__LINE__);

	            return std::move(objectMessage);
 		 	}

 		};
 	}
 }

 #endif