/*
 * Should produce world to robot coordinate transform
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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#include "FieldModel.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Transform2D.h"
#include "utility/math/vision.h"

namespace module {
    namespace localisation {

        using utility::math::matrix::Rotation3D;
        using utility::math::matrix::Transform3D;
        using utility::math::matrix::Transform2D;
        using utility::math::vision::cameraSpaceGoalProjection;
        using message::vision::Goal;
        using message::support::FieldDescription;
        using message::input::Sensors;
        using message::input::ServoID;

        arma::vec::fixed<FieldModel::size> FieldModel::timeUpdate(const arma::vec::fixed<size>& state, double /*deltaT*/) {
            return state;
        }

        arma::vec FieldModel::predictedObservation(const arma::vec::fixed<size>& state
            , const std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>>& measurements
            , const FieldDescription& field
            , const Sensors& sensors
            , const MeasurementType::GOAL&) 
        {

            //make a storage for our goal locations
            arma::vec3 goalLocation;
            goalLocation[2] = 0.0;
            arma::mat::fixed<3,4> goalNormals;
            Transform2D world = sensors.world.projectTo2D(arma::vec3({0,0,1}),arma::vec3({1,0,0}));
            //Get the x/y position for goals
            arma::vec prediction(3*measurements.size());
            int counter = 0;
            for(auto& type : measurements) {
                arma::vec3 lastGoalLocation = goalLocation;
                //choose which goalpost we are looking at
                // Switch on Team
                switch(std::get<0>(type)) {
                    // Switch on Side
                    case Goal::Team::OWN:
                        switch(std::get<1>(type)) {
                            case Goal::Side::LEFT:
                                goalLocation.rows(0,1) = field.goalpost_own_l;
                                break;
                            case Goal::Side::RIGHT:
                                goalLocation.rows(0,1) = field.goalpost_own_r;
                                break;
                            case Goal::Side::UNKNOWN:
                                break;
                        }
                    case Goal::Team::OPPONENT:
                        switch(std::get<1>(type)) {
                            case Goal::Side::LEFT:
                                goalLocation.rows(0,1) = field.goalpost_opp_l;
                                break;
                            case Goal::Side::RIGHT:
                                goalLocation.rows(0,1) = field.goalpost_opp_r;
                                break;
                            case Goal::Side::UNKNOWN:
                                break;
                        }

                    case Goal::Team::UNKNOWN:
                        break;
                }
                //XXX: this should use a torso transform that includes our IMU orientation
                if ( arma::any(lastGoalLocation == goalLocation) ) {
                    goalNormals = cameraSpaceGoalProjection(state + world,goalLocation,field,sensors.orientationCamToGround);
                }
                // Switch on normal type
                switch(std::get<2>(type)) {

                    case Goal::MeasurementType::LEFT_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(0);
                    } break;

                    case Goal::MeasurementType::RIGHT_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(1);
                    } break;

                    case Goal::MeasurementType::TOP_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(2);
                    } break;

                    case Goal::MeasurementType::BASE_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(3);
                    } break;

                }
                counter += 3;
            }
            return prediction;
        }

        arma::vec FieldModel::observationDifference(const arma::vec& a, const arma::vec& b) const {
            return (a - b);
        }

        arma::vec::fixed<FieldModel::size> FieldModel::limitState(const arma::vec::fixed<size>& state) const {
            return state;
        }

        arma::mat::fixed<FieldModel::size, FieldModel::size> FieldModel::processNoise() const {
            return arma::diagmat(processNoiseDiagonal);
        }

    }
}
