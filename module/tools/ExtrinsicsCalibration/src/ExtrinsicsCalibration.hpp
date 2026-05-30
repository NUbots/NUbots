/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_TOOLS_EXTRINSICSCALIBRATION_HPP
#define MODULE_TOOLS_EXTRINSICSCALIBRATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <string>
#include <vector>

#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldIntersections.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"

namespace module::tools {

    /**
     * @brief Calibrates a single camera's extrinsic offsets (roll, pitch, yaw) by optimising offset deltas
     * (del_roll, del_pitch, del_yaw) so that the projected YOLO field-landmark detections (X, L, T
     * intersections) align with the known ground-truth landmark positions.
     *
     * Assumptions:
     *  - The robot is static, placed on the centre of the field, looking straight toward the goal.
     *  - The field type (and hence the field dimensions / ground-truth landmark positions) is known via the
     *    FieldDescription.
     */
    class ExtrinsicsCalibration : public NUClear::Reactor {
    private:
        /// @brief Number of optimisation variables (del_roll, del_pitch, del_yaw)
        static constexpr unsigned int n_params = 3;

        /// @brief Stores configuration values
        struct Config {
            /// @brief Path to the URDF model used for camera forward kinematics
            std::string urdf_path{};

            /// @brief Which camera to calibrate ("Left" or "Right")
            std::string camera{};

            /// @brief True if the calibrated camera is the left camera
            bool is_left_camera = true;

            /// @brief Assumed torso heading in field space when placed at the centre [rad]
            double field_yaw = 0.0;

            /// @brief Seconds to wait after startup before collecting detections
            double start_delay = 0.0;

            /// @brief Seconds over which to accumulate detections once collection begins
            double collection_duration = 0.0;

            /// @brief Minimum number of associated samples required before running the optimisation
            size_t min_samples = 0;

            /// @brief Maximum distance [m] in field space for a detection to be associated with a landmark
            double max_association_distance = 0.0;

            /// @brief Search bounds (half-width) on each offset delta [rad] (del_roll, del_pitch, del_yaw)
            Eigen::Vector3d offset_bounds = Eigen::Vector3d::Zero();

            /// @brief NLopt relative tolerance on the optimisation parameters
            double xtol_rel = 0.0;

            /// @brief NLopt relative tolerance on the optimisation function value
            double ftol_rel = 0.0;

            /// @brief NLopt maximum number of evaluations
            size_t maxeval = 0;

            /// @brief Whether to write the optimised offsets back to the robot's camera config file
            bool write_config = false;
        } cfg;

        /// @brief A single associated calibration sample.
        struct Sample {
            /// @brief Detection ray in camera {c} space (offset-independent, recovered from the pixel)
            Eigen::Vector3d uICc = Eigen::Vector3d::Zero();
            /// @brief World {w} from head-pitch {p} transform at the time of detection (offset-free)
            Eigen::Isometry3d Hwp = Eigen::Isometry3d::Identity();
            /// @brief Field {f} from world {w} transform at the time of detection (from placement assumption)
            Eigen::Isometry3d Hfw = Eigen::Isometry3d::Identity();
            /// @brief Associated ground-truth landmark position in field {f} space
            Eigen::Vector3d rLFf = Eigen::Vector3d::Zero();
        };

        /// @brief Calibration lifecycle states
        enum class State { WAITING, COLLECTING, DONE };

        /// @brief Current calibration state
        State state = State::WAITING;

        /// @brief Time at which the module started up
        NUClear::clock::time_point startup_time = NUClear::clock::now();

        /// @brief Time at which detection collection began
        NUClear::clock::time_point collection_start = NUClear::clock::now();

        /// @brief Base (offset-free) head-pitch {p} from camera {c} transform, from URDF forward kinematics
        Eigen::Isometry3d Hpc_base = Eigen::Isometry3d::Identity();

        /// @brief Current extrinsic offsets (roll, pitch, yaw) read from the robot's camera config [rad]
        Eigen::Vector3d current_offsets = Eigen::Vector3d::Zero();

        /// @brief Ground-truth field landmarks (position in field space + type)
        std::vector<utility::localisation::Landmark> landmarks{};

        /// @brief Accumulated associated samples used for the optimisation
        std::vector<Sample> samples{};

        /// @brief Path to the robot's camera config file that will be (re)written
        std::string camera_config_path{};

        /**
         * @brief Build the extrinsic offset rotation matrix from (roll, pitch, yaw), matching the convention
         * used by the Camera module.
         * @param offsets The offsets (roll, pitch, yaw) [rad]
         * @return The offset rotation matrix
         */
        Eigen::Matrix3d offset_rotation(const Eigen::Vector3d& offsets) const;

        /**
         * @brief Re-project a detection ray onto the field ground plane in field {f} space for a given set of
         * extrinsic offsets.
         * @param offsets The candidate extrinsic offsets (roll, pitch, yaw) [rad]
         * @param sample The sample to re-project
         * @return The projected landmark position in field {f} space
         */
        Eigen::Vector3d project_to_field(const Eigen::Vector3d& offsets, const Sample& sample) const;

        /**
         * @brief Associate the detections in a frame with the known landmarks using the Hungarian algorithm,
         * evaluated at the current extrinsic offsets, and append the matched samples to the buffer.
         * @param field_intersections The field intersection detections
         * @param Hwp The offset-free world from head-pitch transform for this frame
         * @param Hfw The field from world transform for this frame
         */
        void associate_and_store(const message::vision::FieldIntersections& field_intersections,
                                 const Eigen::Isometry3d& Hwp,
                                 const Eigen::Isometry3d& Hfw);

        /**
         * @brief Total squared re-projection cost over all accumulated samples for a given set of offsets.
         * @param offsets The candidate extrinsic offsets (roll, pitch, yaw) [rad]
         * @return The summed squared distance between each projected detection and its associated landmark
         */
        double total_cost(const Eigen::Vector3d& offsets) const;

        /**
         * @brief NLopt objective. The optimisation variables are the offset deltas (del_roll, del_pitch,
         * del_yaw); the cost is evaluated at (current_offsets + delta).
         * @param x The offset deltas (del_roll, del_pitch, del_yaw) [rad]
         * @param grad Unused (BOBYQA is derivative-free)
         * @param data Pointer to the owning ExtrinsicsCalibration instance
         * @return The total re-projection cost
         */
        static double objective(const std::vector<double>& x, std::vector<double>& grad, void* data);

        /**
         * @brief Run the BOBYQA optimisation over the accumulated samples to find the best offsets.
         * @return Pair <optimal offsets (roll, pitch, yaw), final cost>
         */
        std::pair<Eigen::Vector3d, double> optimise();

        /**
         * @brief Write the optimised offsets back to the robot's camera config file, formatted as
         * "<degrees> * pi / 180" expression strings.
         * @param offsets The optimised offsets (roll, pitch, yaw) [rad]
         */
        void write_offsets(const Eigen::Vector3d& offsets);

    public:
        /// @brief Called by the powerplant to build and setup the ExtrinsicsCalibration reactor.
        explicit ExtrinsicsCalibration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_EXTRINSICSCALIBRATION_HPP
