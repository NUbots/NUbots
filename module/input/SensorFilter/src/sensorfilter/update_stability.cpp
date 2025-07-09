/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

namespace module::input {

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;

    void SensorFilter::update_stability(std::unique_ptr<Sensors>& sensors, WalkState::State walk_state) {
        // ********** FIRST FIND IF FALLEN **********
        // Transform to torso{t} from world{w} space
        Eigen::Matrix4d Hwt = sensors.Htw.inverse().matrix();
        // Basis Z vector of torso {t} in world {w} space
        Eigen::Vector3d uZTw = Hwt.block(0, 2, 3, 1);

        // Get the angle of the robot with the world z axis
        double angle = std::acos(Eigen::Vector3d::UnitZ().dot(uZTw));

        // // Check if angle between torso z axis and world z axis is greater than config value
        if (angle > cfg.fallen_angle) {
            emit(std::make_unique<Stability>(Stability::FALLEN));
            return;
        }

        // ********** THEN CHECK FOR FALLING **********
        auto& a = sensors.accelerometer;
        auto& g = sensors.gyroscope;

        // Smooth the values we use to determine if we are falling
        gyro_mag  = smooth(gyro_mag,
                          std::abs(std::abs(g.x()) + std::abs(g.y()) + std::abs(g.z()) - cfg.gyro_mag.mean),
                          cfg.gyro_mag.smoothing);
        acc_mag   = smooth(acc_mag,  //
                         std::abs(a.norm()),
                         cfg.acc_mag.smoothing);
        acc_angle = smooth(acc_angle,
                           std::acos(std::min(1.0, std::abs(a.normalized().z())) - cfg.acc_angle.mean),
                           cfg.acc_angle.smoothing);


        // Check if we are stable according to each sensor
        State gyro_mag_state  = gyro_mag < cfg.gyro_mag.unstable  ? State::STABLE
                                : gyro_mag < cfg.gyro_mag.falling ? State::UNSTABLE
                                                                  : State::FALLING;
        State acc_mag_state   = acc_mag > cfg.acc_mag.unstable  ? State::STABLE
                                : acc_mag > cfg.acc_mag.falling ? State::UNSTABLE
                                                                : State::FALLING;
        State acc_angle_state = acc_angle < cfg.acc_angle.unstable  ? State::STABLE
                                : acc_angle < cfg.acc_angle.falling ? State::UNSTABLE
                                                                    : State::FALLING;

        // Falling if any 2 of 3 sensors report falling or the angle reports falling
        bool falling = (gyro_mag_state == State::FALLING && acc_mag_state == State::FALLING)
                       || (gyro_mag_state == State::FALLING && acc_angle_state == State::FALLING)
                       || (acc_mag_state == State::FALLING && acc_angle_state == State::FALLING)
                       || (acc_angle_state == State::FALLING);

        if (falling) {
            emit(std::make_unique<Stability>(Stability::FALLING));
            return;
        }

        // ******** DETERMINE IF STANDING **********
        if (walk_state == WalkState::State::STANDING || walk_state == WalkState::State::UNKNOWN) {
            // If the walk engine is standing or unknown, we are likely standing
            emit(std::make_unique<Stability>(Stability::STANDING));
            return;
        }
        emit(std::make_unique<Stability>(Stability::STATIC));
    }

}  // namespace module::input
