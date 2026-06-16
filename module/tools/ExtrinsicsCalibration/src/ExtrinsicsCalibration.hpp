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
#include <nlopt.hpp>
#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldIntersections.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"

namespace module::tools {

    /**
     * @brief Calibrates a single camera's extrinsic offsets (roll, pitch, yaw) by optimising offset deltas
     * (del_roll, del_pitch, del_yaw) so that the projected YOLO field-landmark detections (X, L, T
     * intersections) align with the known ground-truth landmark positions.
     *
     * The robot stands itself up and automatically sweeps its head through a yaw/pitch grid (via the PlanLook
     * LookAround provider, configured by the role-specific PlanLook.yaml) so the detections span the image
     * without any manual input.
     *
     * Assumptions:
     *  - The robot is placed on the centre of the field, looking straight toward the goal.
     *  - The field type (and hence the field dimensions / ground-truth landmark positions) is known via the
     *    FieldDescription.
     */
    class ExtrinsicsCalibration : public ::extension::behaviour::BehaviourReactor {
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

            /// @brief Minimum number of associated samples required before running the optimisation
            size_t min_samples = 0;

            /// @brief Maximum distance [m] in field space for a detection to be associated with a landmark
            double max_association_distance = 0.0;

            /// @brief Minimum head yaw/pitch change [rad] since the last captured frame before a new frame is
            /// gathered. Keeps samples spread across distinct head poses (the head sweeps automatically) instead
            /// of over-sampling whatever pose the head is lingering in.
            double min_head_pose_change = 0.0;

            /// @brief Search bounds (half-width) on each offset delta [rad] (del_roll, del_pitch, del_yaw)
            Eigen::Vector3d offset_bounds = Eigen::Vector3d::Zero();

            /// @brief NLopt relative tolerance on the optimisation parameters
            double xtol_rel = 0.0;

            /// @brief NLopt relative tolerance on the optimisation function value
            double ftol_rel = 0.0;

            /// @brief NLopt maximum number of evaluations
            size_t maxeval = 0;

            /// @brief Maximum number of ICP iterations (re-associate then re-optimise) before giving up
            size_t max_icp_iterations = 0;
        } cfg;

        /// @brief A single raw detection, kept so it can be re-associated as the offsets are refined (ICP).
        struct Observation {
            /// @brief Detection ray in camera {c} space (offset-independent, recovered from the pixel)
            Eigen::Vector3d uICc = Eigen::Vector3d::Zero();
            /// @brief Detection type (X / L / T intersection), used to restrict same-type associations
            message::vision::FieldIntersection::IntersectionType type =
                message::vision::FieldIntersection::IntersectionType::UNKNOWN;
        };

        /// @brief One captured frame: the offset-free transforms plus every detection in that frame.
        /// Detections are grouped per frame so the Hungarian association stays one-to-one within a frame.
        struct Frame {
            /// @brief World {w} from head-pitch {p} transform for this frame (offset-free)
            Eigen::Isometry3d Hwp = Eigen::Isometry3d::Identity();
            /// @brief Field {f} from world {w} transform for this frame (from placement assumption)
            Eigen::Isometry3d Hfw = Eigen::Isometry3d::Identity();
            /// @brief Raw detections captured in this frame
            std::vector<Observation> observations{};
        };

        /// @brief A single associated calibration sample (one detection paired with a ground-truth landmark).
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
        /// @brief Base (offset-free) head-pitch {p} from camera {c} transform, from URDF forward kinematics
        Eigen::Isometry3d Hpc_base = Eigen::Isometry3d::Identity();

        /// @brief Current extrinsic offsets (roll, pitch, yaw) read from the robot's camera config [rad]
        Eigen::Vector3d current_offsets = Eigen::Vector3d::Zero();

        /// @brief Head yaw/pitch [rad] of the most recently captured frame, used to gather samples only once the
        /// head has moved far enough (see Config::min_head_pose_change)
        double last_head_yaw   = 0.0;
        double last_head_pitch = 0.0;

        /// @brief True once the calibration has finished and written the offsets. The robot keeps standing and the
        /// head holds still (rather than exiting and collapsing); the operator stops the binary.
        bool calibrated = false;

        /// @brief Ground-truth field landmarks (position in field space + type)
        std::vector<utility::localisation::Landmark> landmarks{};

        /// @brief Accumulated raw frames (re-associated each ICP iteration)
        std::vector<Frame> frames{};

        /// @brief Total number of raw detections collected so far across all frames
        size_t collected_detections = 0;

        /// @brief Associated samples for the current ICP iteration (rebuilt by associate(), used by the optimiser)
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
         * @brief Recover the offset-independent camera-frame rays for a frame's detections and store the raw
         * frame for later (re-)association. No landmark association is committed here.
         * @param field_intersections The field intersection detections
         * @param Hwp The offset-free world from head-pitch transform for this frame
         * @param Hfw The field from world transform for this frame
         */
        void collect(const message::vision::FieldIntersections& field_intersections,
                     const Eigen::Isometry3d& Hwp,
                     const Eigen::Isometry3d& Hfw);

        /**
         * @brief (Re-)associate every collected detection with the known landmarks using the Hungarian algorithm,
         * evaluated at the given offsets. Rebuilds the `samples` buffer used by the optimiser.
         * @param offsets The extrinsic offsets (roll, pitch, yaw) [rad] at which to project and associate
         * @return Per-observation signature (frame-major) of assigned landmark indices, -1 where unassociated.
         *         Used to detect ICP convergence (associations no longer changing).
         */
        std::vector<int> associate(const Eigen::Vector3d& offsets);

        /**
         * @brief Run the ICP loop: alternate (associate at the current best offsets) and (optimise the offsets
         * with those associations held fixed), re-centring on the refined offsets each iteration, until the
         * associations stop changing or the iteration cap is reached. Writes the result and exits on success.
         */
        void run_calibration();

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
         * @return Tuple <optimal offsets (roll, pitch, yaw), final cost, optimisation result code>
         */
        std::tuple<Eigen::Vector3d, double, nlopt::result> optimise();

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
