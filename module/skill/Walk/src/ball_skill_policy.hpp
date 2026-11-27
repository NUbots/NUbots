/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
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
#ifndef MODULE_SKILL_WALK_BALL_SKILL_POLICY_HPP
#define MODULE_SKILL_WALK_BALL_SKILL_POLICY_HPP

#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ball_skill.hpp"

#include "utility/onnx/ONNXRuntime.hpp"

namespace module::skill::ball_skill {

    /// The K1 joints in JointIndexK1 order, as the ONNX metadata names them once canonical() is applied
    inline const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {
        "head_yaw",          "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_pitch",
        "left_elbow_yaw",    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_pitch",  "right_elbow_yaw",
        "left_hip_pitch",    "left_hip_roll",        "left_hip_yaw",        "left_knee_pitch",    "left_ankle_pitch",
        "left_ankle_roll",   "right_hip_pitch",      "right_hip_roll",      "right_hip_yaw",      "right_knee_pitch",
        "right_ankle_pitch", "right_ankle_roll"};

    /// mjlab prefixes some joints with A or AA to fix MuJoCo's ordering ("AAHead_yaw", "ALeft_Shoulder_Pitch").
    /// Lower case, with the prefix dropped.
    [[nodiscard]] inline std::string canonical(std::string name) {
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
        const auto start = name.find_first_not_of('a');
        return start == std::string::npos ? name : name.substr(start);
    }

    [[nodiscard]] inline std::vector<std::string> split(const std::string& text) {
        std::vector<std::string> parts;
        std::stringstream stream(text);
        for (std::string part; std::getline(stream, part, ',');) {
            parts.push_back(part);
        }
        return parts;
    }

    /// One exported mjlab ball-skill policy: obs [1, 234] -> actions [1, 22]. Observation normalisation is inside
    /// the graph. The joint order, default pose, action scale and PD gains are read from the file's metadata, so a
    /// retrained policy with the same layout needs no config change.
    class Policy {
    public:
        std::string path;
        std::array<double, JOINT_COUNT> default_pose{};
        std::array<double, JOINT_COUNT> action_scale{};
        std::array<double, JOINT_COUNT> kp{};
        std::array<double, JOINT_COUNT> kd{};
        /// False if the file's gains were placeholders (kp 1, kd 0) and the fallback gains are used instead
        bool gains_from_file = true;

        /// @throws std::runtime_error if the file cannot be loaded or does not match the ball-skill layout
        Policy(const std::string& model_path,
               const std::string& device,
               const std::array<double, JOINT_COUNT>& fallback_kp,
               const std::array<double, JOINT_COUNT>& fallback_kd)
            : path(model_path), runtime(std::make_unique<utility::onnx::ONNXRuntime>(model_path, device)) {
            const auto numel = [](const std::vector<int64_t>& shape) {
                return std::accumulate(shape.begin(), shape.end(), std::size_t{1}, [](std::size_t n, int64_t d) {
                    return n * static_cast<std::size_t>(d);
                });
            };
            if (numel(runtime->input_shape()) != INPUT_SIZE || numel(runtime->output_shape()) != JOINT_COUNT) {
                fail("ONNX I/O is " + std::to_string(numel(runtime->input_shape())) + " -> "
                     + std::to_string(numel(runtime->output_shape())) + ", the ball skills are "
                     + std::to_string(INPUT_SIZE) + " -> " + std::to_string(JOINT_COUNT));
            }

            const auto names = split(lookup("joint_names"));
            if (names.size() != JOINT_COUNT) {
                fail("joint_names lists " + std::to_string(names.size()) + " joints, not 22");
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                if (canonical(names[i]) != JOINT_NAMES[i]) {
                    fail("joint " + std::to_string(i) + " is '" + names[i] + "', JointIndexK1's is '" + JOINT_NAMES[i]
                         + "'");
                }
            }
            if (const auto observed = runtime->metadata("observation_names");
                observed && *observed != "base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,ball") {
                fail("observation terms are '" + *observed + "', not the ball-skill layout");
            }
            default_pose = numbers("default_joint_pos");
            action_scale = numbers("action_scale");
            kp           = numbers("joint_stiffness");
            kd           = numbers("joint_damping");
            // Files exported from runs with the torque-speed motor model before the exporter fix carry kp 1 / kd 0
            const bool usable = std::all_of(kp.begin(), kp.end(), [](double v) { return v > 1.0; })
                                && std::all_of(kd.begin(), kd.end(), [](double v) { return v > 0.0; });
            if (!usable) {
                kp              = fallback_kp;
                kd              = fallback_kd;
                gains_from_file = false;
            }
        }

        [[nodiscard]] std::array<float, JOINT_COUNT> infer(const std::vector<float>& obs) {
            const std::vector<float> out = runtime->infer(obs);
            std::array<float, JOINT_COUNT> action{};
            std::copy_n(out.begin(), JOINT_COUNT, action.begin());
            return action;
        }

    private:
        std::unique_ptr<utility::onnx::ONNXRuntime> runtime;

        [[noreturn]] void fail(const std::string& what) const {
            throw std::runtime_error("ball-skill policy " + path + ": " + what);
        }

        [[nodiscard]] std::string lookup(const char* key) const {
            const auto value = runtime->metadata(key);
            if (!value) {
                fail(std::string("no '") + key + "' in its metadata");
            }
            return *value;
        }

        [[nodiscard]] std::array<double, JOINT_COUNT> numbers(const char* key) const {
            const auto parts = split(lookup(key));
            if (parts.size() != JOINT_COUNT) {
                fail(std::string(key) + " has " + std::to_string(parts.size()) + " values, not 22");
            }
            std::array<double, JOINT_COUNT> values{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                values[i] = std::stod(parts[i]);
            }
            return values;
        }
    };

}  // namespace module::skill::ball_skill

#endif  // MODULE_SKILL_WALK_BALL_SKILL_POLICY_HPP
