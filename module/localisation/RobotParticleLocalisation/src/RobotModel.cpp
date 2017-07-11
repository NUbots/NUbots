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
        /*
        switch(fd.dimensions.goalpost_type) {
            case FieldDescription::GoalpostType::CIRCLE: {
                // Calculate given a circular goalpost crosssection
            } break;
            case FieldDescription::GoalpostType::RECTANGLE: {

                // Calculate given a rectangular goalpost crosssection
            } break;
        }
        */
        // Get our transform to world coordinates
        const Transform3D& Htw = convert<double, 4, 4>(sensors.world);
        const Transform3D& Htc = convert<double, 4, 4>(sensors.forwardKinematics.at(ServoID::HEAD_PITCH));
        Transform3D Hcw = Htc.i() * Htw;


        Transform3D Hfw;
        Hfw.translation() = arma::vec3{state[kX], state[kY],0};
        Hfw = Hfw.rotateZ(state[kAngle]);

        Transform3D Hcf = Hcw * Hfw.i();
        if (type == Goal::MeasurementType::CENTRE){
            //rGCc = vector from camera to goal post expected position
            arma::vec3 rGCc = Hcf.transformPoint(arma::vec3{actual_position[0],actual_position[1],0});
            arma::vec3 rGCc_sph = cartesianToSpherical(rGCc); // in r,theta,phi
            return rGCc_sph;
        }
        else if (type == Goal::MeasurementType::LEFT_NORMAL){
            //rGFf = vector from field origin to goal post expected position. bl = bottom left, tl = top left.
            arma::vec3 rGFf = {actual_position[0], actual_position[1], 0};

            // Find the vector to the top and bottom left edge points
            //TODO: support non-cylindrical goal posts
            arma::vec3 rG_blFf = rGFf + arma::vec3{0, fd.dimensions.goalpost_width*0.5, 0};
            arma::vec3 rG_tlFf = {rG_blFf[0], rG_blFf[1], fd.dimensions.goal_crossbar_height};

            //creating the normal vector (following convention stipulated in VisionObjects)
            arma::vec3 rNFf = arma::normalise(arma::cross(rG_blFf, rG_tlFf));
            arma::vec3 rNCc = Hcf.transformPoint(rNFf);
            arma::vec2 angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
            return angles;
        }
        else if (type == Goal::MeasurementType::RIGHT_NORMAL){
            //rGFf = vector from field origin to goal post expected position. bl = bottom left, tl = top left.
            arma::vec3 rGFf = {actual_position[0], actual_position[1], 0};

            // Find the vector to the top and bottom right edge points
            //TODO: support non-cylindrical goal posts
            arma::vec3 rG_brFf = rGFf + arma::vec3{0, fd.dimensions.goalpost_width*0.5, 0};
            arma::vec3 rG_trFf = {rG_brFf[0], rG_brFf[1], fd.dimensions.goal_crossbar_height};
            
            //creating the normal vector (following convention stipulated in VisionObjects)
            arma::vec3 rNFf = arma::normalise(arma::cross(rG_trFf, rG_brFf));
            arma::vec3 rNCc = Hcf.transformPoint(rNFf);
            arma::vec2 angles = { std::atan2(rNCc[1],rNCc[0]) , std::atan2(rNCc[2],std::sqrt(rNCc[0]*rNCc[0] + rNCc[1]*rNCc[1]))};
            return rNCc;
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
