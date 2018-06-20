/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "Transform3D.h"

namespace utility {
namespace math {
    namespace matrix {

        using geometry::UnitQuaternion;

        Transform3D::Transform() {
            eye();  // identity matrix by default
        }

        Transform3D::Transform(const UnitQuaternion& q) : Transform(Rotation3D(q)) {}

        Transform3D::Transform(const Rotation3D& rotation) : Transform() {
            submat(0, 0, 2, 2) = rotation;
        }

        Transform3D::Transform(const Rotation3D& rotation, const arma::vec3& translation) : Transform() {
            submat(0, 0, 2, 2)  = rotation;
            this->translation() = translation;
        }

        Transform3D::Transform(const Transform2D& transform)
            : Transform(Transform3D().translate({transform.x(), transform.y(), 0}).rotateZ(transform.angle())) {}

        Transform3D::Transform(const arma::vec6& in)
            : Transform(Transform3D().translate(in.rows(0, 2)).rotateZ(in[5]).rotateY(in[4]).rotateX(in[3])) {}

        Transform3D::Transform(const arma::vec3& in) : Transform(Transform3D().translate(in)) {}

        Transform3D Transform3D::translate(const arma::vec3& translation) const {
            return *this * createTranslation(translation);
        }

        Transform3D Transform3D::translateX(double translation) const {
            return translate({translation, 0, 0});
        }

        Transform3D Transform3D::translateY(double translation) const {
            return translate({0, translation, 0});
        }

        Transform3D Transform3D::translateZ(double translation) const {
            return translate({0, 0, translation});
        }

        Transform3D Transform3D::rotateX(double radians) const {
            return *this * createRotationX(radians);
        }

        Transform3D Transform3D::rotateY(double radians) const {
            return *this * createRotationY(radians);
        }

        Transform3D Transform3D::rotateZ(double radians) const {
            return *this * createRotationZ(radians);
        }

        Transform3D Transform3D::scale(const arma::vec3& v) const {
            return *this * createScale(v);
        }

        Transform3D Transform3D::rotateLocal(const Rotation3D& rotation, const Transform3D& local) const {
            return Transform3D(Transform3D(rotation) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateXLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationX(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateYLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationY(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::rotateZLocal(double radians, const Transform3D& local) const {
            return Transform3D(createRotationZ(radians) * worldToLocal(local)).localToWorld(local);
        }

        Transform3D Transform3D::worldToLocal(const Transform3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference.i() * (*this);
        }

        Transform3D Transform3D::localToWorld(const Transform3D& reference) const {
            // http://en.wikipedia.org/wiki/Change_of_basis
            return reference * (*this);
        }


        arma::vec3 Transform3D::transformPoint(const arma::vec3& p) const {
            arma::vec4 p4      = arma::join_cols(p, arma::vec({1}));
            arma::vec4 result4 = *this * p4;
            return result4.rows(0, 2);
        }

        arma::vec3 Transform3D::transformVector(const arma::vec3& p) const {
            arma::vec4 p4      = arma::join_cols(p, arma::vec({0}));
            arma::vec4 result4 = *this * p4;
            return result4.rows(0, 2);
        }


        Transform3D Transform3D::i() const {
            // Create a new transform
            Transform3D inverseTransform3D;
            // Transpose the rotation submatrix (top-left 3x3), this is equivalent to taking the inverse of the rotation
            // matrix
            inverseTransform3D.submat(0, 0, 2, 2) = submat(0, 0, 2, 2).t();
            // Multiply translation vector (top-right column vector) by the negated inverse rotation matrix

            inverseTransform3D.submat(0, 3, 2, 3) = -inverseTransform3D.submat(0, 0, 2, 2) * submat(0, 3, 2, 3);
            /*if (arma::norm(inverseTransform3D * (*this) - arma::eye(4,4)) > 1e-10){
                NUClear::log<NUClear::WARN>("Inverse failed! Matrix is singular");
            }*/
            return inverseTransform3D;
        }

        float Transform3D::norm(Transform3D T) {
            float pos_norm = arma::norm(T.translation());
            // return Rotation3D::norm(T.rotation());
            // TODO: how to weight these two?
            return pos_norm + Rotation3D::norm(T.rotation());
        }

        float Transform3D::random(float a, float b) {
            float alpha = rand() / float(RAND_MAX);
            return a * alpha + b * (1 - alpha);
        }

        Transform3D Transform3D::getRandomU(float max_angle, float max_displacement) {
            UnitQuaternion q = UnitQuaternion::getRandomU(max_angle);
            Rotation3D R(q);

            // Get displacement:

            float phi      = random(0, 2 * M_PI);
            float costheta = random(-1, 1);
            float u        = random(0, 1);

            float theta = std::acos(costheta);
            float r     = max_displacement * std::pow(u, 1 / 3.0);

            float x = r * sin(theta) * cos(phi);
            float y = r * sin(theta) * sin(phi);
            float z = r * cos(theta);

            return Transform3D(R, arma::vec3({x, y, z}));
        }

        Transform3D Transform3D::getRandomN(float stddev_angle, float stddev_disp) {
            UnitQuaternion q = UnitQuaternion::getRandomN(stddev_angle);
            Rotation3D R(q);

            // Get displacement:
            arma::vec3 displacement = stddev_disp * arma::randn(3);

            return Transform3D(R, displacement);
        }

        Transform3D Transform3D::createTranslation(const arma::vec3& translation) {
            Transform3D transform;
            transform.col(3).rows(0, 2) = translation;
            return transform;
        }

        Transform3D Transform3D::createRotationX(double radians) {
            Transform3D transform;
            transform.submat(0, 0, 2, 2) = Rotation3D::createRotationX(radians);
            return transform;
        }

        Transform3D Transform3D::createRotationY(double radians) {
            Transform3D transform;
            transform.submat(0, 0, 2, 2) = Rotation3D::createRotationY(radians);
            return transform;
        }

        Transform3D Transform3D::createRotationZ(double radians) {
            Transform3D transform;
            transform.submat(0, 0, 2, 2) = Rotation3D::createRotationZ(radians);
            return transform;
        }

        Transform3D Transform3D::createScale(const arma::vec3& v) {
            Transform3D transform;
            transform.rotation() = arma::diagmat(v);
            return transform;
        }

        Transform3D Transform3D::interpolate(Transform3D T1, Transform3D T2, float alpha) {
            Rotation3D r1     = T1.rotation();
            UnitQuaternion q1 = UnitQuaternion(r1);
            Rotation3D r2     = T2.rotation();
            UnitQuaternion q2 = UnitQuaternion(r2);

            arma::vec3 t1 = T1.translation();
            arma::vec3 t2 = T2.translation();

            UnitQuaternion qResult = q1.slerp(q2, alpha);
            arma::vec3 tResult     = alpha * (t2 - t1) + t1;

            Transform3D TResult   = Transform3D(Rotation3D(qResult));
            TResult.translation() = tResult;

            return TResult;
        }


        Transform2D Transform3D::projectTo2D(const arma::vec3& yawAxis, const arma::vec3& forwardAxis) const {
            Transform2D result;

            // Translation
            arma::vec3 orthoForwardAxis = arma::normalise(arma::cross(yawAxis, arma::cross(forwardAxis, yawAxis)));
            arma::vec3 r                = translation();
            Rotation3D newSpaceToWorld;
            newSpaceToWorld.x()        = orthoForwardAxis;
            newSpaceToWorld.y()        = arma::cross(yawAxis, orthoForwardAxis);
            newSpaceToWorld.z()        = yawAxis;
            Rotation3D worldToNewSpace = newSpaceToWorld.i();
            arma::vec3 rNewSpace       = worldToNewSpace * r;
            result.xy()                = rNewSpace.rows(0, 1);

            // Rotation
            Rotation3D rot       = rotation();
            arma::vec3 x         = rot.x();
            arma::vec3 xNew      = worldToNewSpace * x;
            float theta_x_from_f = std::atan2(xNew[1], xNew[0]);  // sin/cos
            result.angle()       = theta_x_from_f;

            // std::cerr << "in = \n" << *this << std::endl;
            // std::cerr << "out = \n" << result << std::endl;
            return result;
        }
    }  // namespace matrix
}  // namespace math
}  // namespace utility
