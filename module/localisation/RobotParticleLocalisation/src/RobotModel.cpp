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

#include "RobotModel.h"

#include <armadillo>
#include <nuclear>
#include <iostream>

#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "message/localisation/FieldObject.h"
#include "utility/localisation/transform.h"
#include "message/input/Sensors.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"


namespace module {
namespace localisation {

    using message::input::Sensors;
    using message::vision::Goal;
    using utility::input::ServoID;
    using utility::localisation::transform::SphericalRobotObservation;
    using utility::localisation::transform::WorldToRobotTransform;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::localisation::transform::ImuToWorldHeadingTransform;
    using utility::math::coordinates::cartesianToRadial;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::angle::normalizeAngle;
    using message::support::FieldDescription;


    using utility::math::matrix::Transform3D;
    arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(
        const arma::vec::fixed<RobotModel::size>& state, double /*deltaT*/) {
        arma::vec::fixed<RobotModel::size> new_state = state;

        return new_state;
    }


    /// Return the predicted observation of an object at the given position
    arma::vec RobotModel::predictedObservation(
        const arma::vec::fixed<RobotModel::size>& state,
        const arma::vec& actual_position,
        const Sensors& sensors,
        const Goal::MeasurementType& type,
        const FieldDescription& fd) {
        
        // Get our transform to world coordinates
        const Transform3D& Htw = convert<double, 4, 4>(sensors.world);
        const Transform3D& Htc = convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH));
        Transform3D Hcw = Htc.i() * Htw;


        Transform3D Hfw;
        Hfw.translation() = arma::vec3{state[kX], state[kY],0};
        Hfw = Hfw.rotateZ(state[kAngle]);

        Transform3D Hcf = Hcw * Hfw.i();
        Transform3D Htf = Htw * Hfw.i();

        //rZFf = vector from field origin to zenith high in the sky
        arma::vec3 rZFf = {0,0,1000000};
        arma::vec3 rZCc = Hcf.transformPoint(rZFf);

        switch(FieldDescription::GoalpostType::Value(fd.dimensions.goalpost_type)) {
            case FieldDescription::GoalpostType::CIRCLE: {
                if (type == Goal::MeasurementType::CENTRE){
                    //rGCc = vector from camera to goal post expected position
                    arma::vec3 rGCc = Hcf.transformPoint(arma::vec3{actual_position[0],actual_position[1],0});
                    arma::vec3 rGCc_sph = cartesianToSpherical(rGCc); // in r,theta,phi
                    return rGCc_sph;
                }
                else if (type == Goal::MeasurementType::LEFT_NORMAL){
                    //rGFf = vector from field origin to goal post expected position.
                    arma::vec3 rGFf = {actual_position[0], actual_position[1], 0};
                    arma::vec3 rGCc = Hcf.transformPoint(rGFf);
                    
                    // Finding the vector from robot to the goal
                    arma::vec3 rRFf = Htf.translation();
                    arma::vec3 rGRf = rGFf - rRFf;
                    // The vector pointing from the robot off to the left from a cross product
                    arma::vec3 rLRf = 1000000*arma::normalise(arma::cross(rZFf,rGFf));

                    // Find the vector to the top and bottom left edge points. bl = bottom left, tl = top left.
                    // TODO: Circular Tangental effect
                    arma::vec3 rG_blCc = rGCc + 0.5*fd.dimensions.goalpost_width*arma::normalise(rLRf);
                    arma::vec3 rG_tlCc = rG_blCc + fd.dimensions.goal_crossbar_height*arma::normalise(rZCc);

                    //creating the normal vector (following convention stipulated in VisionObjects)
                    arma::vec3 rNCc = arma::normalise(arma::cross(rG_blCc, rG_tlCc));
                    arma::vec2 angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
                    return angles;
                }
                else if (type == Goal::MeasurementType::RIGHT_NORMAL){
                    //rGFf = vector from field origin to goal post expected position.
                    arma::vec3 rGFf = {actual_position[0], actual_position[1], 0};
                    arma::vec3 rGCc = Hcf.transformPoint(rGFf);

                    // Finding the vector from robot to the goal
                    arma::vec3 rRFf = Htf.translation();
                    arma::vec3 rGRf = rGFf - rRFf;
                    // The vector pointing from the robot off to the left from a cross product
                    arma::vec3 rLRf = 1000000*arma::normalise(arma::cross(rZFf,rGFf));

                    // Find the vector to the top and bottom right edge points  bl = bottom left, tl = top left.
                    // TODO: Circular Tangental effect
                    arma::vec3 rG_brCc = rGCc - 0.5*fd.dimensions.goalpost_width*arma::normalise(rLRf);
                    arma::vec3 rG_trCc = rG_brCc + fd.dimensions.goal_crossbar_height*arma::normalise(rZCc);
                    
                    //creating the normal vector (following convention stipulated in VisionObjects)
                    arma::vec3 rNCc = arma::normalise(arma::cross(rG_trCc, rG_brCc));
                    arma::vec2 angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
                    return angles;
                }
            } break;
            case FieldDescription::GoalpostType::RECTANGLE: {
                // Finding 4 corners of goalpost (4 corners x XY)
                arma::mat goalBaseCorners(4,3);
                goalBaseCorners.submat(0,0,3,1).each_row() = actual_position;
                goalBaseCorners.col(2).fill(0.0);
                goalBaseCorners.submat(0,0,1,0) -= 0.5*fd.dimensions.goalpost_depth; // x for front goal corners
                goalBaseCorners.submat(2,0,3,0) += 0.5*fd.dimensions.goalpost_depth; // x for back  goal corners
                goalBaseCorners.submat(1,1,2,1) -= 0.5*fd.dimensions.goalpost_width; // y for right goal corners
                goalBaseCorners.submat(0,1,0,1) += 0.5*fd.dimensions.goalpost_width; // y for front left goal corners
                goalBaseCorners.submat(3,1,3,1) += 0.5*fd.dimensions.goalpost_width; // y for back  left goal corners

                if (type == Goal::MeasurementType::CENTRE){
                    //rGCc = vector from camera to goal post expected position
                    arma::vec3 rGCc = Hcf.transformPoint(arma::vec3{actual_position[0],actual_position[1],0});
                    arma::vec3 rGCc_sph = cartesianToSpherical(rGCc); // in r,theta,phi
                    return rGCc_sph;
                }
                else if ((type == Goal::MeasurementType::LEFT_NORMAL) || (type == Goal::MeasurementType::RIGHT_NORMAL)){
                    //rGFf = vector from field origin to goal post expected position.
                    arma::vec3 rGFf = {actual_position[0], actual_position[1], 0};

                    // Finding the vector from robot to the goal corners
                    arma::vec3 rTFf = Htf.translation();
                    arma::mat  vecs2GoalCorners(4,3); // 4 rGTfs
                    vecs2GoalCorners = goalBaseCorners;
                    vecs2GoalCorners.each_row() -= rTFf;
                    
                    // This section calculates the 6 possible angles to the goal edges to find the widest 2 edges
                    int j_store = 1;
                    double theta;
                    arma::vec3 a;
                    arma::vec3 b;
                    arma::vec3 rG_blCc;
                    arma::vec3 rG_brCc;
                    double max_theta = 0.0;
                    for (int i=0;i<3;++i){
                        for (int j=j_store;j<4;++j){
                            a = vecs2GoalCorners.submat(i,0,i,2);
                            b = vecs2GoalCorners.submat(j,0,j,2);
                            theta = std::acos(arma::norm_dot(a,b));
                            if (std::abs(theta) > max_theta){
                                max_theta = theta;
                                if (theta > 0) {
                                    rG_blCc = Hcf.transformPoint(goalBaseCorners.submat(j,0,j,2));
                                    rG_brCc = Hcf.transformPoint(goalBaseCorners.submat(i,0,i,2));
                                }
                                else {
                                    rG_blCc = Hcf.transformPoint(goalBaseCorners.submat(i,0,i,2));
                                    rG_brCc = Hcf.transformPoint(goalBaseCorners.submat(j,0,j,2));
                                }
                            }
                        }
                        j_store += 1;
                    }
                    arma::vec2 angles;
                    if (type == Goal::MeasurementType::LEFT_NORMAL) {
                        arma::vec3 rG_tlCc = rG_blCc + fd.dimensions.goal_crossbar_height*arma::normalise(rZCc);
                        //creating the normal vector (following convention stipulated in VisionObjects)
                        arma::vec3 rNCc = arma::normalise(arma::cross(rG_blCc, rG_tlCc));
                        angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
                    }
                    else {
                        arma::vec3 rG_trCc = rG_brCc + fd.dimensions.goal_crossbar_height*arma::normalise(rZCc);
                        //creating the normal vector (following convention stipulated in VisionObjects)
                        arma::vec3 rNCc = arma::normalise(arma::cross(rG_trCc, rG_brCc));
                        angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
                    }
                    return angles;
                }
            } break;
        }
    }


    arma::vec RobotModel::observationDifference(const arma::vec& a,
                                                const arma::vec& b) {
        return a-b;
    }

    arma::vec::fixed<RobotModel::size> RobotModel::limitState(
        const arma::vec::fixed<RobotModel::size>& state) {
        auto state2 = state;
        state2[kAngle] = normalizeAngle(state2[kAngle]);
        // TODO: Clip robot's state to the field?
        return state2;
    }

    arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise(){
        arma::mat noise = arma::eye(size, size);
        //TODO: this
        // noise(kX, kX) *= cfg_.processNoisePositionFactor;
        // noise(kY, kY) *= cfg_.processNoisePositionFactor;
        // noise(kImuOffset, kImuOffset) *= cfg_.processNoiseHeadingFactor;
        // std::cout << "process noise = \n" << noise << std::endl;
        return arma::diagmat(processNoiseDiagonal);
    }


}
}
