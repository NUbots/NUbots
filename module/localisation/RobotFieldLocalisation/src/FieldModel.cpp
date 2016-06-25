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

namespace module {
    namespace platform {
        namespace darwin {

            arma::vec::fixed<FieldModel::size> FieldModel::limitState(const arma::vec::fixed<size>& state) {
                return state;
            }

            arma::vec::fixed<FieldModel::size> FieldModel::timeUpdate(const arma::vec::fixed<size>& state, double deltaT) {

                //TODO: create the Rwf transformation matrix so we can make Rcf

                return state;
            }

            arma::vec FieldModel::predictedObservation(const arma::vec::fixed<size>& state
                , const std::vector<std::tuple<Goal::Team, Goal::Side, Goal::MeasurementType>& measurements
                , const FieldDescription& field
                , const Sensors& sensors
                , const MeasurementType::GOAL&) {

                

                //make a storage for our goal locations
                arma::vec3 goalLocation;
                goalLocation[2] = 0.;

                //choose which goalpost we are looking at
                // Switch on Team
                switch(std::get<0>(type)) {
                    // Switch on Side
                    case Goal::Team::OWN:
                        switch(std::get<1>(type)) {
                            case Goal::Side::LEFT:
                                goalLocation.rows(0,1) = FieldModel.goalpost_own_l;
                            case Goal::Side::RIGHT:
                                goalLocation.rows(0,1) = FieldModel.goalpost_own_r;
                        }
                    case Goal::Team::OPPONENT:
                        switch(std::get<1>(type)) {
                            case Goal::Side::LEFT:
                                goalLocation.rows(0,1) = FieldModel.goalpost_opp_l;
                            case Goal::Side::RIGHT:
                                goalLocation.rows(0,1) = FieldModel.goalpost_opp_r;
                        }
                }


                //NOTE: this code assumes that goalposts are boxes with width and high of goalpost_diameter
                //make the base goal corners
                arma::mat goalBaseCorners(3,4);
                goalBaseCorners.each_col() = goalLocation;
                goalBaseCorners.cols(0,1) -= 0.5*field.dimensions.goalpost_diameter;
                goalBaseCorners.submat(0,0,1,0) += field.dimensions.goalpost_diameter;
                goalBaseCorners.submat(1,1,2,1) += field.dimensions.goalpost_diameter;

                //make the top corner points
                arma::mat goalTopCorners = goalBaseCorners;
                goalTopCorners.row(2) += field.goalpost_top_height;

                //create the camera to field transformation
                Transform3D Hct = sensors.forwardKinematics[ServoID::HEAD_PITCH].i();
                Transform3D Htw = sensors.forwardKinematics[ServoID::HEAD_PITCH];
                arma::vec3 rFWf = state.rows(0,1);

                //XXX: check correctness
                rFWf[2] = 0.;
                rFWf = state.rows(0,1);
                Transform3D Hwf = Transform3D::rotateZLocal(state[2],rFWf);

                //We create camera world by using camera-torso -> torso-world -> world->field
                Transform3D Hcf = Hct * Htw * Hwf;

                //transform the goals from field to camera
                goalBaseCorners = Hcf.i() * goalBaseCorners;
                goalTopCorners = Hcf.i() * goalTopCorners;

                //Select the (tl, tr, bl, br) corner points for normals
                arma::ivec4 cornerIndices;
                cornerIndices.fill(0);

                arma::vec pvals = goalBaseCorners*arma::cross(goalBaseCorners[0],goalTopCorners[0]);
                cornerIndices[2] = pvals.max_index();
                cornerIndices[3] = pvals.min_index();

                pvals = goalTopCorners*arma::cross(goalBaseCorners[0],goalTopCorners[0]);
                cornerIndices[0] = pvals.max_index();
                cornerIndices[1] = pvals.min_index();

                //Get the x/y position for goals
                arma::vec prediction(3*measurements.size());
                int counter = 0;
                for(auto& type : measurements) {
                    // Switch on normal type
                    switch(std::get<2>) {

                        case Goal::MeasurementType::LEFT_NORMAL:
                            arma::vec3 normal = arma::normalize(
                                                        arma::cross(
                                                            goalBaseCorners[cornerIndices[2]],
                                                            goalTopCorners[cornerIndices[0]]
                                                            )
                                                    );
                            prediction.rows(counter,counter+2) = normal;

                        case Goal::MeasurementType::RIGHT_NORMAL:
                            arma::vec3 normal = arma::normalize(
                                                        arma::cross(
                                                            goalTopCorners[cornerIndices[1]],
                                                            goalBaseCorners[cornerIndices[3]]
                                                            )
                                                    );
                            prediction.rows(counter,counter+2) = normal;

                        case Goal::MeasurementType::TOP_NORMAL:
                            arma::vec3 normal = arma::normalize(
                                                        arma::cross(
                                                            goalTopCorners[cornerIndices[0]],
                                                            goalTopCorners[cornerIndices[1]]
                                                            )
                                                    );
                            prediction.rows(counter,counter+2) = normal;

                        case Goal::MeasurementType::BASE_NORMAL:
                            arma::vec3 normal = arma::normalize(
                                                        arma::cross(
                                                            goalBaseCorners[cornerIndices[3]],
                                                            goalBaseCorners[cornerIndices[2]]
                                                            )
                                                    );
                            prediction.rows(counter,counter+2) = normal;

                    }
                    counter += 3;
                }

                return prediction;
            }

            arma::vec FieldModel::observationDifference(const arma::vec& a, const arma::vec& b) {
                return a - b;

            }

            arma::mat::fixed<FieldModel::size, FieldModel::size> FieldModel::processNoise() {
                return arma::diagmat(processNoiseDiagonal);
            }

        }
    }
}
