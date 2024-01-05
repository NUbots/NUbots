/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "SensorFilter.hpp"

#include "message/actuation/BodySide.hpp"

#include "utility/actuation/tinyrobotics.hpp"
#include "utility/input/FrameID.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"

namespace module::input {

    using message::actuation::BodySide;

    using utility::actuation::tinyrobotics::forward_kinematics_to_servo_map;
    using utility::actuation::tinyrobotics::sensors_to_configuration;
    using utility::input::FrameID;
    using utility::input::ServoID;

    void SensorFilter::update_kinematics(std::unique_ptr<Sensors>& sensors, const RawSensors& raw_sensors) {

        // Convert the sensor joint angles to a configuration vector
        Eigen::Matrix<double, n_joints, 1> q = sensors_to_configuration<double, n_joints>(sensors);

        // **************** Kinematics ****************
        // Htx is a map from FrameID to homogeneous transforms from each frame to the torso
        std::vector<Eigen::Isometry3d> fk = tinyrobotics::forward_kinematics(nugus_model, q);
        auto Htx                          = forward_kinematics_to_servo_map(fk);
        for (const auto& entry : Htx) {
            sensors->Htx[entry.first] = entry.second.matrix();
        }

        // **************** Centre of Mass and Inertia Tensor ****************
        sensors->rMTt = tinyrobotics::center_of_mass(nugus_model, q);

        // **************** Foot Down Information ****************
        sensors->feet[BodySide::RIGHT].down = true;
        sensors->feet[BodySide::LEFT].down  = true;
        const Eigen::Isometry3d Htr(sensors->Htx[FrameID::R_ANKLE_ROLL]);
        const Eigen::Isometry3d Htl(sensors->Htx[FrameID::L_ANKLE_ROLL]);
        const Eigen::Isometry3d Hlr = Htl.inverse() * Htr;
        const Eigen::Vector3d rRLl  = Hlr.translation();
        switch (cfg.footDown.method()) {
            case FootDownMethod::Z_HEIGHT:
                if (rRLl.z() < -cfg.footDown.threshold()) {
                    sensors->feet[BodySide::LEFT].down = false;
                }
                else if (rRLl.z() > cfg.footDown.threshold()) {
                    sensors->feet[BodySide::RIGHT].down = false;
                }
                break;
            case FootDownMethod::FSR:
                // Determine if any two diagonally opposite FSRs are in contact with the ground, which is either fsr1
                // and fsr3, or fsr2 and fsr4. A FSR is in contact with the ground if its value is greater than the
                // certainty threshold
                sensors->feet[BodySide::LEFT].down = (raw_sensors.fsr.left.fsr1 > cfg.footDown.threshold()
                                                      && raw_sensors.fsr.left.fsr3 > cfg.footDown.threshold())
                                                     || (raw_sensors.fsr.left.fsr2 > cfg.footDown.threshold()
                                                         && raw_sensors.fsr.left.fsr4 > cfg.footDown.threshold());
                sensors->feet[BodySide::RIGHT].down = (raw_sensors.fsr.right.fsr1 > cfg.footDown.threshold()
                                                       && raw_sensors.fsr.right.fsr3 > cfg.footDown.threshold())
                                                      || (raw_sensors.fsr.right.fsr2 > cfg.footDown.threshold()
                                                          && raw_sensors.fsr.right.fsr4 > cfg.footDown.threshold());
                break;
            default: log<NUClear::WARN>("Unknown foot down method"); break;
        }
    }
}  // namespace module::input
