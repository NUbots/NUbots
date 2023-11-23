/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef MODULES_PLATFORM_CM740_HARDWARESIMULATOR_HPP
#define MODULES_PLATFORM_CM740_HARDWARESIMULATOR_HPP

#include <Eigen/Core>
#include <mutex>
#include <nuclear>

#include "message/platform/RawSensors.hpp"

namespace module::platform {

    /**
     * This NUClear Reactor is responsible for reading in the data for the CM740 Platform and emitting it to
     * the rest of the system
     *
     * @author Jade Fountain
     */
    class HardwareSimulator : public NUClear::Reactor {
    private:
        message::platform::RawSensors sensors;

        float imu_drift_rate                     = 0.0f;
        static constexpr size_t UPDATE_FREQUENCY = 90;
        void addNoise(std::unique_ptr<message::platform::RawSensors>& sensors) const;
        struct NoiseConfig {
            NoiseConfig() = default;
            Eigen::Vector3f accelerometer{0.001, 0.001, 0.001};
            Eigen::Vector3f gyroscope{0.001, 0.001, 0.001};
        } noise;
        double bodyTilt                      = 0;
        Eigen::Vector3d integrated_gyroscope = Eigen::Vector3d::Zero();
        void setRightFootDown(bool down);
        void setLeftFootDown(bool down);

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareSimulator(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::platform
#endif
