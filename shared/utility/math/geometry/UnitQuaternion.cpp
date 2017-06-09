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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "UnitQuaternion.h"
#include "utility/math/angle.h"

namespace utility {
namespace math {
namespace geometry {

    using matrix::Rotation3D;

    UnitQuaternion::UnitQuaternion() {
        real() = 1;
        imaginary().zeros();
    }

    UnitQuaternion::UnitQuaternion(const Rotation3D& rotation) {
        real() = std::sqrt(1.0 + rotation(0,0) + rotation(1,1) + rotation(2,2)) / 2;
        double w4 = 4.0 * real();
        imaginary() = Eigen::Vector3d(
            (rotation(2,1) - rotation(1,2)) / w4,
            (rotation(0,2) - rotation(2,0)) / w4,
            (rotation(1,0) - rotation(0,1)) / w4
        );
    }

    UnitQuaternion::UnitQuaternion(double realPart, const Eigen::Vector3d& imaginaryPart) {
        real() = realPart;
        imaginary() = imaginaryPart;
    }

    UnitQuaternion::UnitQuaternion(const Eigen::Vector3d& v) {
        real() = 0;
        imaginary() = v;
    }

    UnitQuaternion::UnitQuaternion(const Eigen::Vector3d& axis, double angle) {
        real() = std::cos(angle / 2.0);
        imaginary() = std::sin(angle / 2.0) * axis.normalize();
    }

   UnitQuaternion::UnitQuaternion(double W, double X, double Y, double Z)
    {
        real()      = W;
        imaginary() = Eigen::Vector3d(X, Y, Z);
    }

    UnitQuaternion::UnitQuaternion(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2)
    {
        double norm     = vec1.norm() * vec2.norm();
        double half_cos = std::sqrt(0.5 + vec1.dot(vec2) / (2.0 * norm));

        real()      = half_cos;
        imaginary() = vec1.cross(vec2) / (2.0 * half_cos * norm);

        this->normalise();
    }

    void UnitQuaternion::rectify(){
        if(kW() < 0){
            *this = -*this;
        }
    }

    UnitQuaternion UnitQuaternion::i() const {
        UnitQuaternion qi = *this;
        // take the congugate, as it is equal to the inverse when a unit vector
        qi.imaginary() *= -1;
        return qi;
    }

    Eigen::Vector3d UnitQuaternion::rotateVector(const Eigen::Vector3d& v) const {
        // Do the math
        const Eigen::Vector3d t = 2*imaginary().cross(v);
        return v + real() * t + imaginary().cross(t);
    }

    Eigen::Vector3d UnitQuaternion::getAxis() const {
        double angle = getAngle();
        double sinThetaOnTwo = std::sin(angle / 2.0);
        return imaginary() / sinThetaOnTwo;
    }

    double UnitQuaternion::getAngle() const {
        //Max and min prevent nand error, presumably due to computational limitations
        return 2 * utility::math::angle::acos_clamped(std::fmin(1,std::fmax(real(),-1)));
    }

    void UnitQuaternion::setAngle(double angle) {
        real() = std::cos(angle / 2.0);
        imaginary() = std::sin(angle / 2.0) * imaginary().normalize();
    }

    void UnitQuaternion::scaleAngle(double scale) {
        setAngle(getAngle() * scale);
    }

    void UnitQuaternion::normalise() {
        *this = (*this).normalize();
    }

    Eigen::Matrix4d UnitQuaternion::getLeftQuatMultMatrix() const{
        Eigen::Matrix4d Q;
        Q << kW() << -kX() << -kY() << -kZ() << arma::endr
          << kX() <<  kW() << -kZ() <<  kY() << arma::endr
          << kY() <<  kZ() <<  kW() << -kX() << arma::endr
          << kZ() << -kY() <<  kX() <<  kW() << arma::endr;
        return Q;
    }

    Eigen::Matrix4d UnitQuaternion::getRightQuatMultMatrix() const{
        Eigen::Matrix4d W;
        W << kW() << -kX() << -kY() << -kZ() << arma::endr
          << kX() <<  kW() <<  kZ() << -kY() << arma::endr
          << kY() << -kZ() <<  kW() <<  kX() << arma::endr
          << kZ() <<  kY() << -kX() <<  kW() << arma::endr;
        return W;
    }


    float UnitQuaternion::random(float a, float b){
        float alpha = rand() / float(RAND_MAX);
        return a * alpha + b * (1 - alpha);
    }

    UnitQuaternion UnitQuaternion::getRandomU(float max_angle){
        //Get angle:
        float angle = random(0,max_angle);

        //Get axis:
        float phi = random(0,2 * M_PI);
        float costheta = random(-1,1);

        float theta = std::acos( costheta );
        float r = 1;

        float x = r * sin( theta) * cos( phi );
        float y = r * sin( theta) * sin( phi );
        float z = r * cos( theta );
        Eigen::Vector3d axis = {x,y,z};

        return UnitQuaternion(axis, angle);
    }

    UnitQuaternion UnitQuaternion::getRandomN(float stddev){
        //Get angle:
        float angle = stddev * arma::randn(1)[0];

        //Get axis:
        float phi = random(0,2 * M_PI);
        float costheta = random(-1,1);

        float theta = std::acos( costheta );
        float r = 1;

        float x = r * sin( theta) * cos( phi );
        float y = r * sin( theta) * sin( phi );
        float z = r * cos( theta );
        Eigen::Vector3d axis = {x,y,z};

        return UnitQuaternion(axis, angle);
    }

    double UnitQuaternion::norm() {
        return kW() * kW() + kX() * kX() + kY() * kY() + kZ() * kZ();
    }

    UnitQuaternion UnitQuaternion::operator - (const UnitQuaternion& p) const {
        return *this * p.inverse();
    }

    UnitQuaternion UnitQuaternion::operator * (const UnitQuaternion& p) const {
        //From http://en.wikipedia.org/wiki/Quaternion#Quaternions_and_the_geometry_of_R3
        double realPart = real() * p.real() - imaginary().dot(p.imaginary());

        Eigen::Vector3d imaginaryPart = imaginary().cross(p.imaginary())
                                 + p.real() *   imaginary()
                                 +   real() * p.imaginary();

        return UnitQuaternion(realPart, imaginaryPart);
    }

    UnitQuaternion UnitQuaternion::slerp(const UnitQuaternion& p, const double& t) {
        // See http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
        // Where qa = *this and qb = p

        double cosHalfTheta = kW() * p.kW() + kX() * p.kX() + kY() * p.kY() + kZ() * p.kZ();

        // If qa=qb or qa=-qb then theta = 0 and we can return qa
        if (std::abs(cosHalfTheta) >= 1.0) {
            return *this;
        }

        double halfTheta = utility::math::angle::acos_clamped(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);

        // If theta = 180 degrees then result is not fully defined
        // We could rotate around any axis normal to qa or qb
        if (std::abs(sinHalfTheta) < 0.001) {
            return *this * 0.5 + p * 0.5;
        }

        // Interpolate
        double ratioA = std::sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = std::sin(t * halfTheta) / sinHalfTheta;

        return *this * ratioA + p * ratioB;
    }

}
}
}
