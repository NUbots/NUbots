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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_CM740_HARDWARESIMULATOR_HPP
#define MODULES_PLATFORM_CM740_HARDWARESIMULATOR_HPP

#include <Eigen/Core>
#include <mutex>
#include <nuclear>

#include "message/platform/RawSensors.hpp"

namespace module::platform::cm740 {

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
            Eigen::Vector3f accelerometer{0.001f, 0.001f, 0.001f};
            Eigen::Vector3f gyroscope{0.001f, 0.001f, 0.001f};
        } noise;
        float bodyTilt                       = 0.0f;
        Eigen::Vector3d integrated_gyroscope = Eigen::Vector3d::Zero();
        void setRightFootDown(bool down);
        void setLeftFootDown(bool down);

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareSimulator(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::platform::cm740
#endif
