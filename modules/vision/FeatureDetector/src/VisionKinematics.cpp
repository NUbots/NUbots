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

#include "VisionKinematics.h"

#include "messages::input::ServoID";

namespace modules {
    namespace vision {

        using messages::input::ServoID;

        VisionKinematics::VisionKinematics() {
            m_FOV << 0 << 0;
            m_effectiveCameraDistancePixels = 0;

            m_headPitch = m_headYaw = m_bodyRoll = 0;

            m_neckPosition << 0 << 0 << 0;

            m_imageSize << 0 << 0;
            m_imageCentre << 0 << 0;
            m_tanHalfFOV << 0 << 0;
            m_screenToRadialFactor << 0 << 0;
        }

        void VisionKinematics::setParameters(float RADIAL_CORRECTION_COEFFICIENT_,
                                             float SCREEN_LOCATION_UNCERTAINTY_PIXELS_) {
            RADIAL_CORRECTION_COEFFICIENT = RADIAL_CORRECTION_COEFFICIENT_;

            SCREEN_LOCATION_UNCERTAINTY_PIXELS = SCREEN_LOCATION_UNCERTAINTY_PIXELS_;
        }

        /**
          * Applies radial distortion correction to the given pixel location.
          * @param pt The pixel location to correct.
          */
        arma::vec2 VisionKinematics::correctDistortion(const arma::vec2& point) {
            // Gget position relative to centre.
            arma::vec2 halfSize = m_imageSize * 0.5;
            arma::vec2 centreRelative = point - halfSize;

            // Calculate correction factor -> 1+kr^2
        //    double correctionFactor = (1 + (VisionConstants::RADIAL_CORRECTION_COEFFICIENT * centreRelative.squareAbs());
            double correctionFactor = (1 + (RADIAL_CORRECTION_COEFFICIENT * arma::dot(centreRelative, centreRelative)));

            // Multiply by factor.
            arma::vec2 result = centreRelative * correctionFactor;

            // Scale the edges back out to meet again.
            result[0] /= (1 + (RADIAL_CORRECTION_COEFFICIENT * halfSize[0] * halfSize[0]));
            result[1] /= (1 + (RADIAL_CORRECTION_COEFFICIENT * halfSize[1] * halfSize[1]));

            // Get the original position back from the centre relative position.
            return (result + halfSize);
        }

        void VisionKinematics::calculateRepresentationsFromPixelLocation(NUPoint& point, bool known_distance, double val) const {
            // Calculate the radial position (relative to the camera vector) from the pixel position.
            //CHECK SCREEN COORDINATE SYSTEM: im[0] = to right, im[1] = down
            point.screenAngular[0] = atan( (m_imageCentre[0] - point.screenCartesian[0]) * m_screenToRadialFactor[0]);  //along camera y
            point.screenAngular[1] = atan( (m_imageCentre[1] - point.screenCartesian[1]) * m_screenToRadialFactor[1]);  //along camera z

            if (known_distance) {
                // In this case val represents known distance (for e.g. found by perspective comparison).
                arma::vec3 imagePositionSpherical;
                imagePositionSpherical << val << point.screenAngular[0] << arma::math::pi()/2-point.screenAngular[1];       //Changed to agree with standard convention for spherical coordinates(using declination)
                point.bodyRelativeSpherical = utility::math::coordinates::Cartesian2Spherical(m_camToBodyMatrix.submat(0,0,2,2) * utility::math::coordinates::Spherical2Cartesian(imagePositionSpherical));
            }

            else {
                // In this case val represents known height
                // Calculate the radial position relative to
                point.bodyRelativeSpherical = distanceToPoint(point.screenCartesian, val);
            }
            arma::vec3 cartesian = utility::math::coordinates::Spherical2Cartesian(point.bodyRelativeSpherical);
            point.groundCartesian[0] = cartesian[0];
            point.groundCartesian[1] = cartesian[1];
        }

        void VisionKinematics::calculateRepresentationsFromPixelLocation(std::vector<NUPoint>& points, bool knownDistance, double val) const {
            for (NUPoint& point : points) {
                calculateRepresentationsFromPixelLocation(point, knownDistance, val);
            }
        }
        /
        ///// @note Assumes radial calculation already done
        //void VisionKinematics::radial2DToRadial3D(NUPoint& point, double distance) const {
        //    arma::vec3 imagePositionSpherical;
        //    imagePositionSpherical << distance << point.screenAngular[0] << point.screenAngular[1];
        //    point.bodyRelativeSpherical = utility::math::coordinates::Cartesian2Spherical(camV2RobotRotation * utility::math::coordinates::Spherical2Cartesian(imagePositionSpherical));
        //}

        double VisionKinematics::getCameraDistanceInPixels() const {
            return m_effectiveCameraDistancePixels;
        }


        arma::vec2 VisionKinematics::getFOV() const {
            return m_FOV;
        }

        /**
          * Calculates the distance to a point at a given height
          * @param pixel The pixel location in the image relative to the top left of the screen.
          * @param object_height The height of the point to be measured (from the ground).
          * @return A 3 dimensional vector containing the distance, bearing and elevation to the point.
          */
        arma::vec3 VisionKinematics::distanceToPoint(arma::vec2 pixel, double objectHeight) const {
            arma::vec3 result;

            arma::vec3 cameraToObjectDirection_cam({    //use subscript to denote coord system
                m_effectiveCameraDistancePixels,
                (m_imageSize[0] * 0.5) - pixel[0],
                (m_imageSize[1] * 0.5) - pixel[1]});   //Are the pixels mapped like this? Seems likely.
            // or like this?
            // -(m_imageSize[0] * 0.5) + pixel[0],
            // -(m_imageSize[1] * 0.5) + pixel[1]});
            // std::cout "VisionKinematics::distanceToPoint - cameraToObjectDirection_cam = "<< cameraToObjectDirection_cam << std::endl;

            arma::vec3 cameraToObjectDirection_robot = m_camToBodyMatrix.submat(0,0,2,2) * cameraToObjectDirection_cam;
            // std::cout "VisionKinematics::distanceToPoint - cameraToObjectDirection_robot = "<< cameraToObjectDirection_robot << std::endl;

            double alpha = std::abs(m_camToBodyMatrix.col(3)[2] - objectHeight) / std::abs(cameraToObjectDirection_robot[2]);      //Similar triangle ratio
            // std::cout "VisionKinematics::distanceToPoint - alpha = "<< alpha << std::endl;

            //alpha == cameraToObject_world / norm(cameraToObjectDirection_robot)
            //therefore
            arma::vec3 cameraToObject_robot = alpha * cameraToObjectDirection_robot; //as they are parallel
            // std::cout "VisionKinematics::distanceToPoint - cameraToObject_world = "<< cameraToObject_world << std::endl;

            //
            return utility::math::coordinates::Cartesian2Spherical(cameraToObject_robot);


            /*
             arma::vec3 result;

            arma::vec3 vcam
            << m_effectiveCameraDistancePixels
            << m_imageSize[0] * 0.5) - pixel[0]
            << (m_imageSize[1] * 0.5) - pixel[1];   //Are the pixels mapped like this?

            arma::vec3 roboVdir = m_camV2RobotRotation * vcam;
            double alpha = (objectHeight - m_camVector[2]) / roboVdir[2];
            arma::vec3 v2FieldPoint = (alpha * roboVdir) + m_camVector;

            arma::vec3 temp;
            temp << v2FieldPoint[0] << v2FieldPoint[1]<< (objectHeight - m_camVector[2]);
            result[0] = arma::norm(temp, 2);
            result[1] = std::atan2(v2FieldPoint[1], v2FieldPoint[0]);
            result[2] = std::asin((objectHeight - m_camVector[2] / result[0]);

           return result;*/
        }

        //void VisionKinematics::screenToGroundCartesian(NUPoint& point) const {
        //    arma::vec3 v = distanceToPoint(point.screenCartesian, 0.0);
        //
        //    arma::vec3 sphericalFootRelative = Kinematics::TransformPosition(ctgRransform, v);
        //
        //    arma::vec3 cartesianFootRelative = mathGeneral::Spherical2Cartesian(sphericalFootRelative);
        //
        //    point.groundCartesian << cartesianFootRelative[0] << cartesianFootRelative[1];
        //}

        //void VisionKinematics::screenToGroundCartesian(vector<NUPoint>& points) const {
        //    for (NUPoint& point : points) {
        //        screenToGroundCartesian(point);
        //    }
        //}

        //NUPoint VisionKinematics::screenToGroundCartesian(const arma::vec2& point) const {
        //    NUPoint ground;
        //    ground.screenCartesian = point;
        //    screenToGroundCartesian(ground);
        //    return ground;
        //}

        //std::vector<NUPoint> VisionKinematics::screenToGroundCartesian(const std::vector<arma::vec2>& points) const {
        //    std::vector<NUPoint> groundPoints;
        //
        //    for (const arma::vec2& point : points) {
        //        groundPoints.push_back(screenToGroundCartesian(point));
        //    }
        //
        //    return groundPoints;
        //}

        void VisionKinematics::setSensors(const Sensors& sensors) {
            m_camToBodyMatrix = sensors.forwardKinematics[ServoID::HEAD_PITCH];
        }

        void VisionKinematics::setCamParams(arma::vec2 imageSize, arma::vec2 FOV) {
            m_imageSize = imageSize;
            m_imageCentre = m_imageSize * 0.5;
            m_FOV = FOV;
            m_tanHalfFOV << tan(m_FOV[0] * 0.5) << tan(m_FOV[1] * 0.5);
            m_screenToRadialFactor << (m_tanHalfFOV[0] / m_imageCentre[0]) << (m_tanHalfFOV[1] / m_imageCentre[1]);
            //OLD CALC: m_screenToRadialFactor << (m_FOV[0] / m_imageCentre[0]) << (m_FOV[1] / m_imageCentre[1]);

            m_effectiveCameraDistancePixels = m_imageCentre[0] / m_tanHalfFOV[0];   //Checked to be correct Jake Fountain 2014
        }


        double VisionKinematics::getD2PError(const NUPoint& location) const{
            double declination_error = SCREEN_LOCATION_UNCERTAINTY_PIXELS*m_FOV[1]/m_imageSize[1];
            double robot_height = std::abs(m_neckPosition[2]);
            double sin_elevation = std::sin(location.bodyRelativeSpherical[2]-arma::math::pi());
            return declination_error*robot_height/(sin_elevation*sin_elevation);
        }

        arma::vec3 VisionKinematics::calculateSphericalError(NUPoint location, DISTANCE_METHOD distanceMethod, float width) const{
            arma::vec3 sphericalError;
            switch (distanceMethod) {
                case D2P: {
                    sphericalError[0] = getD2PError(location);

                    break;
                }

                case WIDTH: {
                    sphericalError[0] = SCREEN_LOCATION_UNCERTAINTY_PIXELS*location.bodyRelativeSpherical[0]/width; //=d*dp/p (assuming error of dp=1 pixel in width)

                    break;
                }

                case AVERAGE: {
                    double width_error = SCREEN_LOCATION_UNCERTAINTY_PIXELS*location.bodyRelativeSpherical[0]/width;
                    double d2p_error = getD2PError(location);

                    sphericalError[0] = std::max(width_error,d2p_error);
                    break;
                }

                case LEAST: {
                    double width_error = SCREEN_LOCATION_UNCERTAINTY_PIXELS*location.bodyRelativeSpherical[0]/width;
                    double d2p_error = getD2PError(location);

                    sphericalError[0] = std::max(width_error,d2p_error);
                    break;
                }

                default: {
                    sphericalError[0] = SCREEN_LOCATION_UNCERTAINTY_PIXELS*location.bodyRelativeSpherical[0]/width; //=d*dp/p (assuming error of 1 pixel in width)

                    break;
                }
            }

            sphericalError[1] = SCREEN_LOCATION_UNCERTAINTY_PIXELS*getFOV()[0]/getImageSize()[0];  //Erordp =1
            sphericalError[2] = SCREEN_LOCATION_UNCERTAINTY_PIXELS*getFOV()[1]/getImageSize()[1];  //Erordp =1
            return sphericalError;
        }

    }
}

