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

#include <Eigen/Core>

#include "utility/input/ServoID.h"
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
        using message::support::FieldDescription;
        using message::input::Sensors;
        using ServoID             = utility::input::ServoID;
        using GoalSide            = message::vision::Goal::Side::Value;
        using GoalTeam            = message::vision::Goal::Team::Value;
        using GoalMeasurementType = message::vision::Goal::MeasurementType;

        Eigen::Matrix<double, FieldModel::size, 1> FieldModel::timeUpdate(const Eigen::Matrix<double, size, 1>& state, double /*deltaT*/) {
            return state;
        }

        Eigen::VectorXd FieldModel::predictedObservation(const Eigen::Matrix<double, size, 1>& state
            , const std::vector<std::tuple<GoalTeam, GoalSide, GoalMeasurementType>>& measurements
            , const FieldDescription& field
            , const Sensors& sensors
            , const MeasurementType::GOAL&)
        {

            //make a storage for our goal locations
            Eigen::Vector3d goalLocation;
            goalLocation.fill(0.0);
            Eigen::Matrix<double, 3, 4> goalNormals;
            // Transform2D world = sensors.world.projectTo2D(Eigen::Vector3d(0,0,1),Eigen::Vector3d(1,0,0));

            //Transform2D world = sensors.world.projectTo2D(Eigen::Vector3d(0,0,1),Eigen::Vector3d(1,0,0));
            const Transform3D& Htw = sensors.world;
            const Transform3D& Htc = sensors.forwardKinematics.at(ServoID::HEAD_PITCH);
            Transform3D Hwc = Htw.i() * Htc;
            //Get the x/y position for goals
            Eigen::VectorXd prediction(3*measurements.size());
            int counter = 0;
            //std::string ans = "";
            for(auto& type : measurements) {
                Eigen::Vector3d lastGoalLocation = goalLocation;
                //choose which goalpost we are looking at
                // Switch on Team
                switch(std::get<0>(type)) {
                    // Switch on Side
                    case GoalTeam::OWN:
                        //ans += " own";
                        switch(std::get<1>(type)) {
                            case GoalSide::LEFT:
                                goalLocation.rows(0,1) = field.goalpost_own_l;
                                //ans += " left";
                                break;
                            case GoalSide::RIGHT:
                                goalLocation.rows(0,1) = field.goalpost_own_r;
                                //ans += " right";
                                break;
                            case GoalSide::UNKNOWN_SIDE:
                                break;
                        }
                        break;
                    case GoalTeam::OPPONENT:
                        //ans += " opponent";
                        switch(std::get<1>(type)) {
                            case GoalSide::LEFT:
                                //ans += " left";
                                goalLocation.rows(0,1) = field.goalpost_opp_l;
                                break;
                            case GoalSide::RIGHT:
                                //ans += " right";
                                goalLocation.rows(0,1) = field.goalpost_opp_r;
                                break;
                            case GoalSide::UNKNOWN_SIDE:
                                break;
                        }
                        break;
                    case GoalTeam::UNKNOWN_TEAM:
                        break;
                }
                //only update the goal data if we're looking at a different i
                if ( !arma::all(lastGoalLocation == goalLocation) ) {
                    //std::cerr << ans << " goal" << std::endl;
                    goalNormals = cameraSpaceGoalProjection(state,goalLocation,field,Hwc,false);
                }
                // Switch on normal type
                switch(std::get<2>(type).value) {

                    case GoalMeasurementType::LEFT_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(0);
                    } break;

                    case GoalMeasurementType::RIGHT_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(1);
                    } break;

                    case GoalMeasurementType::TOP_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(2);
                    } break;

                    case GoalMeasurementType::BASE_NORMAL: {
                        prediction.rows(counter,counter+2) = goalNormals.col(3);
                    } break;
                    default: break;
                }
                counter += 3;
            }
            return prediction;
        }

        Eigen::VectorXd FieldModel::observationDifference(const arma::vec& a, const arma::vec& b) const {
            //std::cerr << (a-b) << std::endl << a << std::endl << b << std::endl << std::endl;
            return (a - b);
        }

        Eigen::Matrix<double, FieldModel::size, 1> FieldModel::limitState(const Eigen::Matrix<double, size, 1>& state) const {
            return state;
        }

        Eigen::Matrix<double, FieldModel::size, FieldModel::size> FieldModel::processNoise() const {
            return arma::diagmat(processNoiseDiagonal);
        }

    }
}
