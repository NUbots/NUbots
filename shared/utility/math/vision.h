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
#ifndef UTILITY_MATH_VISION_H
#define UTILITY_MATH_VISION_H

#include <cmath>
#include <armadillo>
#include <nuclear>
#include "message/localisation/FieldObject.h"
#include "message/input/Sensors.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/Plane.h"
#include "utility/math/geometry/ParametricLine.h"
#include "utility/math/angle.h"
#include "message/vision/VisionObjects.h"
#include "message/support/FieldDescription.h"


namespace utility {
namespace math {
namespace vision {

    /**************************************************************
     *SLAME STUFF: TO BE REMOVED FOR GOOD STUFF BELOW (DO NOT USE)*
     **************************************************************/
    /*! @brief Calculates the transformation for taking homogeneous points from world coordinates to camera coordinates
    */
    inline arma::mat calculateWorldToCameraTransform(const message::input::Sensors& sensors, const message::localisation::Self& self){
        arma::vec selfHeading = arma::normalise(self.heading);
        arma::mat robotToWorld_world;
        robotToWorld_world <<  selfHeading[0]  <<  -selfHeading[1]  <<  0 <<      self.position[0] << arma::endr
                           <<  selfHeading[1]  <<   selfHeading[0]  <<  0 <<      self.position[1] << arma::endr
                           <<               0  <<                0  <<  1 <<  sensors.bodyCentreHeight << arma::endr
                           <<               0  <<                0  <<  0 <<                                 1;

        arma::mat cameraToBody_body = sensors.forwardKinematics.at(message::input::ServoID::HEAD_PITCH);
        arma::mat robotToBody_body = arma::eye(4,4);
        robotToBody_body.submat(0,0,2,2) = sensors.world.rotation();

        auto worldToCamera_camera = cameraToBody_body.i() * robotToBody_body * robotToWorld_world.i();
        //Confirmed to be correct by Jake Fountain 2014
        return worldToCamera_camera;
    }


    inline arma::vec directionVectorFromScreenAngular(const arma::vec& screenAngular){
        double theta = screenAngular[0];
        double phi = screenAngular[1];
        return {std::cos(theta) * std::cos(phi), std::sin(theta) * std::cos(phi), std::sin(phi), 0};
    }
    /*! Uses vec for backwards compatibility with 3d homogeneous coordinates
    */
    inline arma::vec screenAngularFromDirectionVector(const arma::vec& direction){
        return { std::atan2(direction[1], direction[0]) , std::atan2(direction[2], arma::norm(arma::vec({direction[0], direction[1]})))};
    }

    inline arma::vec rotateAngularDirection(const arma::vec& screenAngular, const arma::mat& R){
        return screenAngularFromDirectionVector(R*directionVectorFromScreenAngular(screenAngular));
    }

    inline arma::vec screenPositionFromDirectionVector(const arma::vec& directionVector){
        return {directionVector[1] / directionVector[0], directionVector[2] / directionVector[0]};
    }

    inline arma::vec screenPositionFromScreenAngular(const arma::vec& screenAngular){
        return screenPositionFromDirectionVector(directionVectorFromScreenAngular(screenAngular));
    }

    /************************************************
     *                 GOOD STUFF                   *
     ************************************************/

    inline double getParallaxAngle(const arma::vec2& screen1, const arma::vec2& screen2, const double& camFocalLengthPixels){
        arma::vec3 camSpaceP1 = {camFocalLengthPixels, screen1[0], screen1[1]};
        arma::vec3 camSpaceP2 = {camFocalLengthPixels, screen2[0], screen2[1]};

        return utility::math::angle::acos_clamped(arma::dot(camSpaceP1,camSpaceP2) / (arma::norm(camSpaceP1) * arma::norm(camSpaceP2)));
    }

    inline double widthBasedDistanceToCircle(const double& radius, const arma::vec2& s1, const arma::vec2& s2, const double& camFocalLengthPixels){
        double parallaxAngle = getParallaxAngle(s1, s2, camFocalLengthPixels);
        double correctionForClosenessEffect = radius * std::sin(parallaxAngle / 2.0);

        return radius / std::tan(parallaxAngle / 2.0) + correctionForClosenessEffect;
    }

    /*! @param separation - Known distance between points in camera space
        @param s1,s2 - Measured screen coordinates in pixels of points
        @param camFocalLengthPixels - Distance to the virtual camera screen in pixels
    */
    inline double distanceToEquidistantPoints(const double& separation, const arma::vec2& s1, const arma::vec2& s2, const double& camFocalLengthPixels){
        double parallaxAngle = getParallaxAngle(s1, s2, camFocalLengthPixels);
        return (separation / 2) / std::tan(parallaxAngle / 2);
    }

    /*! @brief
        @param cam - coordinates in camera space of the pixel (cam[0] = y coordinate pixels, cam[1] = z coordinate pixels)
        @return im - coordinates on the screen in image space measured x across, y down, zero at top left
    */
    inline arma::ivec2 screenToImage(const arma::vec2& screen, const arma::uvec2& imageSize){
        arma::vec2 v = arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - screen;
        return arma::ivec2({ int(lround(v[0])), int(lround(v[1])) });
    }
    inline arma::vec2 imageToScreen(const arma::ivec2& im, const arma::uvec2& imageSize){
        return arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - im;
    }
    inline arma::vec2 imageToScreen(const arma::vec2& im, const arma::uvec2& imageSize){
        return arma::vec2({ double(imageSize[0] - 1) * 0.5, double(imageSize[1] - 1) * 0.5 }) - im;
    }

    /*! @brief uses pinhole cam model
        @param point - Point in camera space (x along view axis, y to left of screen, z up along screen)
    */
    inline arma::vec2 projectCamSpaceToScreen(const arma::vec3& point, const double& camFocalLengthPixels){
        return {camFocalLengthPixels * point[1] / point[0], camFocalLengthPixels * point[2] / point[0]};
    }

    inline arma::vec2 projectWorldPointToScreen(const arma::vec4& point, const utility::math::matrix::Transform3D& camToGround, const double& camFocalLengthPixels){
        arma::vec4 camSpacePoint = camToGround.i() * point;
        return projectCamSpaceToScreen(camSpacePoint.rows(0,2), camFocalLengthPixels);
    }
    inline arma::vec2 projectWorldPointToScreen(const arma::vec3& point, const utility::math::matrix::Transform3D& camToGround, const double& camFocalLengthPixels){
        arma::vec4 point_ = arma::ones(4);
        point_.rows(0,2) = point;
        return projectWorldPointToScreen(point_, camToGround, camFocalLengthPixels);
    }

    inline arma::vec3 getCamFromScreen(const arma::vec2& screen, const double& camFocalLengthPixels){
        return arma::vec3{camFocalLengthPixels, screen[0], screen[1]};
    }

    inline arma::vec3 projectCamToPlane(const arma::vec3& cam, const utility::math::matrix::Transform3D& camToGround, const utility::math::geometry::Plane<3>& plane){
        arma::vec3 lineDirection = camToGround.submat(0,0,2,2) * cam;
        arma::vec3 linePosition = camToGround.submat(0,3,2,3);

        utility::math::geometry::ParametricLine<3> line;
        line.setFromDirection(lineDirection, linePosition);

        return plane.intersect(line);
    }

    inline arma::vec3 getGroundPointFromScreen(const arma::vec2& screenPos, const utility::math::matrix::Transform3D& camToGround, const double& camFocalLengthPixels){
        return projectCamToPlane(getCamFromScreen(screenPos, camFocalLengthPixels), camToGround, utility::math::geometry::Plane<3>({ 0, 0, 1 }, { 0, 0, 0 }));
    }

    inline double distanceToVerticalObject(const arma::vec2& top, const arma::vec2& base, const double& objectHeight, const double& robotHeight, const double& camFocalLengthPixels) {

        // Parallax from top to base
        double theta = getParallaxAngle(top, base, camFocalLengthPixels);

        // The following equation comes from the dot product identity a*b = |a||b|cos(theta)
        // As we can calculate theta and |a||b| in terms of perpendicular distance to the object we can solve this equation
        // for an inverse equation. It may not be pretty but it will get the job done

        // Cos theta
        const double c = cos(theta);
        const double c2 = c * c;
        // Object height
        const double H = objectHeight;
        const double H2 = H * H;
        // Robot Height
        const double h = robotHeight;
        const double h2 = h * h;

        double innerExpr = std::abs(c)*sqrt(H2*(4.0*H*h + H2*c2 + 4.0*h2*c2 - 4.0*h2 - 4.0*H*h*c2));
        double divisor = 2*std::abs(std::sin(theta));
        return M_SQRT2*sqrt(2.0*H*h + H2*c2 + 2.0*h2*c2 + innerExpr - 2.0*h2 - 2.0*H*h*c2)/divisor;

    }

    inline arma::vec objectDirectionFromScreenAngular(const arma::vec& screenAngular){
        if(std::fmod(std::fabs(screenAngular[0]),M_PI) == M_PI_2 || std::fmod(std::fabs(screenAngular[1]),M_PI) == M_PI_2){
            return {0,0,0};
        }
        double tanTheta = std::tan(screenAngular[0]);
        double tanPhi = std::tan(screenAngular[1]);
        double x = 0;
        double y = 0;
        double z = 0;
        double denominator_sqr = 1+tanTheta*tanTheta+tanPhi*tanPhi;
        //Assume facing forward st x>0 (which is fine for screen angular)
        x = 1 / std::sqrt(denominator_sqr);
        y = x * tanTheta;
        z = x * tanPhi;

        return {x,y,z};
    }

    inline arma::vec screenAngularFromObjectDirection(const arma::vec& v){
        return {std::atan2(v[1],v[0]),std::atan2(v[2],v[0])};
    }

    inline arma::vec measureGoals(
            const arma::vec3& robotPose,
            const std::vector<std::tuple<message::vision::Goal::Team, message::vision::Goal::Side, message::vision::Goal::MeasurementType>>& measurements,
            const message::support::FieldDescription& field, 
            const message::input::Sensors& sensors)
    {
        using message::input::ServoID;
        using message::vision::Goal;

        //Get the x/y position for goals
        arma::vec prediction(3*measurements.size());
        int counter = 0;
        for(auto& type : measurements) {
            //make a storage for our goal locations
            arma::vec3 goalLocation;
            goalLocation[2] = 0.0;

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
            utility::math::matrix::Transform3D Hct = sensors.forwardKinematics.find(ServoID::HEAD_PITCH)->second.i();
            utility::math::matrix::Transform3D Htw = sensors.world;

            //create the world-field transform
            arma::vec3 rFWf;
            rFWf[2] = 0.0;
            rFWf.rows(0,1) = robotPose.rows(0,1);
            //XXX: check correctness
            utility::math::matrix::Transform3D Hwf = utility::math::matrix::Transform3D::createRotationZ(robotPose[2]) 
                                                   + utility::math::matrix::Transform3D::createTranslation(rFWf);
            Hwf(3,3) = 1.0;

            //We create camera world by using camera-torso -> torso-world -> world->field
            utility::math::matrix::Transform3D Hcf = Hct * Htw * Hwf;

            //transform the goals from field to camera
            goalBaseCorners = arma::vec(Hcf.i() * goalBaseCorners).rows(0,2);
            goalTopCorners = arma::vec(Hcf.i() * goalTopCorners).rows(0,2);

            //Select the (tl, tr, bl, br) corner points for normals
            arma::ivec4 cornerIndices;
            cornerIndices.fill(0);

            arma::vec pvals = goalBaseCorners * arma::cross(goalBaseCorners.col(0), goalTopCorners.col(0));
            cornerIndices[2] = pvals.index_max();
            cornerIndices[3] = pvals.index_min();

            pvals = goalTopCorners * arma::cross(goalBaseCorners.col(0), goalTopCorners.col(0));
            cornerIndices[0] = pvals.index_max();
            cornerIndices[1] = pvals.index_min();

            // Switch on normal type
            switch(std::get<2>(type)) {

                case Goal::MeasurementType::LEFT_NORMAL: {
                    arma::vec3 normal = arma::normalise(
                                                arma::cross(
                                                    goalBaseCorners.col(cornerIndices[2]),
                                                    goalTopCorners.col(cornerIndices[0])
                                                    )
                                            );
                    prediction.rows(counter,counter+2) = normal;
                } break;

                case Goal::MeasurementType::RIGHT_NORMAL: {
                    arma::vec3 normal = arma::normalise(
                                                arma::cross(
                                                    goalTopCorners.col(cornerIndices[1]),
                                                    goalBaseCorners.col(cornerIndices[3])
                                                    )
                                            );
                    prediction.rows(counter,counter+2) = normal;
                } break;

                case Goal::MeasurementType::TOP_NORMAL: {
                    arma::vec3 normal = arma::normalise(
                                                arma::cross(
                                                    goalTopCorners.col(cornerIndices[0]),
                                                    goalTopCorners.col(cornerIndices[1])
                                                    )
                                            );
                    prediction.rows(counter,counter+2) = normal;
                } break;

                case Goal::MeasurementType::BASE_NORMAL: {
                    arma::vec3 normal = arma::normalise(
                                                arma::cross(
                                                    goalBaseCorners.col(cornerIndices[3]),
                                                    goalBaseCorners.col(cornerIndices[2])
                                                    )
                                            );
                    prediction.rows(counter,counter+2) = normal;
                } break;

            }
            counter += 3;
        }

        return prediction;

    }


}
}
}

#endif
