/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "WalkEngine.h"

#include "utility/math/matrix/Rotation.h"

namespace modules {
namespace motion {

    using messages::behaviour::LimbID;
    using messages::input::ServoID;
    using messages::input::Sensors;

    void WalkEngine::balance(std::vector<double>& qLegs, const Sensors& sensors) {
        double gyroRoll0 = 0;
        double gyroPitch0 = 0;

        // double phaseComp = std::min({1.0, phaseSingle / 0.1, (1 - phaseSingle) / 0.1});

        //TODO: crashes
        /*ServoID supportLegID = (swingLeg == LimbID::RIGHT_LEG) ? ServoID::L_ANKLE_PITCH : ServoID::R_ANKLE_PITCH;
        arma::mat33 ankleRotation = sensors.forwardKinematics.find(supportLegID)->second.submat(0,0,2,2);
        // get effective gyro angle considering body angle offset
        arma::mat33 kinematicGyroSORAMatrix = sensors.orientation * ankleRotation;   //DOUBLE TRANSPOSE
        std::pair<arma::vec3, double> axisAngle = utility::math::matrix::axisAngleFromRotationMatrix(kinematicGyroSORAMatrix);
        arma::vec3 kinematicsGyro = axisAngle.first * (axisAngle.second / balanceWeight);

        gyroRoll0 = -kinematicsGyro[0]*180.0/M_PI;
        gyroPitch0 = -kinematicsGyro[1]*180.0/M_PI;

        emit(graph("roll", gyroRoll0));
        emit(graph("pitch", gyroPitch0));*/

        /*double yawAngle = 0;
        if (!active) {
            // double support
            yawAngle = (uLeftFoot[2] + uRightFoot[2]) / 2 - uTorsoActual[2];
        }
        else if (swingLeg == LimbID::RIGHT_LEG) {
            yawAngle = uLeftFoot[2] - uTorsoActual[2];
        }
        else {
            yawAngle = uRightFoot[2] - uTorsoActual[2];
        }

        double gyroRoll = gyroRoll0 * std::cos(yawAngle) - gyroPitch0 * std::sin(yawAngle);
        double gyroPitch = gyroPitch0 * std::cos(yawAngle) - gyroRoll0 * std::sin(yawAngle);

        double armShiftX = procFunc(gyroPitch * armImuParamY[1], armImuParamY[2], armImuParamY[3]);
        double armShiftY = procFunc(gyroRoll * armImuParamY[1], armImuParamY[2], armImuParamY[3]);

        double ankleShiftX = procFunc(gyroPitch * ankleImuParamX[1], ankleImuParamX[2], ankleImuParamX[3]);
        double ankleShiftY = procFunc(gyroRoll * ankleImuParamY[1], ankleImuParamY[2], ankleImuParamY[3]);
        double kneeShiftX = procFunc(gyroPitch * kneeImuParamX[1], kneeImuParamX[2], kneeImuParamX[3]);
        double hipShiftY = procFunc(gyroRoll * hipImuParamY[1], hipImuParamY[2], hipImuParamY[3]);

        ankleShift[0] += ankleImuParamX[0] * (ankleShiftX - ankleShift[0]);
        ankleShift[1] += ankleImuParamY[0] * (ankleShiftY - ankleShift[1]);
        kneeShift += kneeImuParamX[0] * (kneeShiftX - kneeShift);
        hipShift[1] += hipImuParamY[0] * (hipShiftY - hipShift[1]);
        armShift[0] += armImuParamX[0] * (armShiftX - armShift[0]);
        armShift[1] += armImuParamY[0] * (armShiftY - armShift[1]);

        // TODO: toe/heel lifting

        if (!active) {
            // Double support, standing still
            // qLegs[1] += hipShift[1]; // Hip roll stabilization
            qLegs[3] += kneeShift; // Knee pitch stabilization
            qLegs[4] += ankleShift[0]; // Ankle pitch stabilization
            // qLegs[5] += ankleShift[1]; // Ankle roll stabilization

            // qLegs[7] += hipShift[1]; // Hip roll stabilization
            qLegs[9] += kneeShift; // Knee pitch stabilization
            qLegs[10] += ankleShift[0]; // Ankle pitch stabilization
            // qLegs[11] += ankleShift[1]; // Ankle roll stabilization
        } else if (swingLeg == LimbID::RIGHT_LEG) {
            qLegs[1] += hipShift[1]; // Hip roll stabilization
            qLegs[3] += kneeShift; // Knee pitch stabilization
            qLegs[4] += ankleShift[0]; // Ankle pitch stabilization
            qLegs[5] += ankleShift[1]; // Ankle roll stabilization

            qLegs[10] += toeTipCompensation * phaseComp; // Lifting toetip
            qLegs[1] += hipRollCompensation * phaseComp; // Hip roll compensation

        } else {
            qLegs[7] += hipShift[1]; // Hip roll stabilization
            qLegs[9] += kneeShift; // Knee pitch stabilization
            qLegs[10] += ankleShift[0]; // Ankle pitch stabilization
            qLegs[11] += ankleShift[1]; // Ankle roll stabilization

            qLegs[4] += toeTipCompensation * phaseComp; // Lifting toetip
            qLegs[7] -= hipRollCompensation * phaseComp; // Hip roll compensation
        }*/
    }

}
}