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
#ifndef MODULE_PLANNING_INVERTEDMPCWALKPATH_PROBLEM_HPP
#define MODULE_PLANNING_INVERTEDMPCWALKPATH_PROBLEM_HPP

#include <Eigen/Core>
#include <cmath>

namespace module::planning::inverted_walk_mpc {

    /// @brief The MPC's optimal control problem: MPCWalkPath's (MPC-1), planning the velocity the robot delivers,
    /// with the capability envelope added as linear constraints on the state (set up in InvertedWalkMPC). Every
    /// 10 Hz tick it is solved, in the robot frame at the time of the solve, over N steps of DT:
    ///
    ///     state   s = (x, y, θ, vx, vy, ω)      pose, and the velocity planned for the previous step
    ///     input   a = (ax, ay, aω)              the change of velocity per second
    ///     step    u = s[3:] + a·dt              velocity during the step
    ///             pose⁺ = RK4(pose, u, dt),  s⁺ = (pose⁺, u)
    ///
    ///     minimise  Σₖ w_pos·ρ(‖pₖ − p*‖) + w_head·hₖ(θₖ) + Σᵢ w_eff,i·uₖ,ᵢ² + Σᵢ w_rate,i·(aₖ,ᵢ·dt)² + slack penalty
    ///               ρ(d) = δ²(√(1 + d²/δ²) − 1)                                    pseudo-Huber distance
    ///               hₖ(θ) = gₖ(1 − cos(θ − θ*)) + (w_face/w_head)(1 − gₖ)(1 − cos(θ − βₖ))   heading
    ///
    ///     subject to  velocity limits on s[3:] at stages 1..N
    ///                 C·s[3:] ≤ d, the capability envelope, at stages 1..N (soft)
    ///                 ‖pₖ − oⱼ‖² ≥ r² for each active obstacle j (soft)
    ///
    /// gₖ (the heading fade-in) and βₖ (the bearing to the target) are computed by the caller from the initial guess,
    /// so they are parameters. Every function is generic in its scalar type, so autodiff differentiates it for the
    /// solver (ocp::ProblemFunctions).
    struct Problem {
        /// @brief Steps in the horizon
        static constexpr int N = 25;
        /// @brief Horizon step and planner period (s)
        static constexpr double DT = 0.1;
        /// @brief Obstacle slots, unused ones are switched off
        static constexpr int MAX_OBSTACLES = 4;
        /// @brief Linear constraints on the planned velocity, unused ones are switched off
        static constexpr int ENVELOPE_ROWS = 24;

        static constexpr int NX = 6;
        static constexpr int NU = 3;
        static constexpr int NH = MAX_OBSTACLES;

        // Per-stage parameter vector layout
        static constexpr int P_TARGET          = 0;  // target x, y, θ in the robot frame (3)
        static constexpr int P_GATE            = 3;  // heading fade-in gₖ
        static constexpr int P_BEARING         = 4;  // bearing βₖ from the predicted position to the target
        static constexpr int P_W_POSITION      = 5;
        static constexpr int P_W_HEADING       = 6;
        static constexpr int P_W_FACE          = 7;
        static constexpr int P_HUBER_DELTA     = 8;
        static constexpr int P_W_EFFORT        = 9;   // per axis (3)
        static constexpr int P_W_RATE          = 12;  // per axis (3)
        static constexpr int P_OBSTACLE_RADIUS = 15;
        static constexpr int P_OBSTACLES       = 16;                      // x, y of each slot (2 · MAX_OBSTACLES)
        static constexpr int P_ACTIVE = P_OBSTACLES + 2 * MAX_OBSTACLES;  // 1 for a real obstacle, 0 for a free slot
        static constexpr int NP       = P_ACTIVE + MAX_OBSTACLES;

        template <typename T>
        using State = Eigen::Matrix<T, NX, 1>;
        template <typename T>
        using Input = Eigen::Matrix<T, NU, 1>;
        template <typename T>
        using Vector3 = Eigen::Matrix<T, 3, 1>;

        /// @brief Omnidirectional kinematics: the body twist u rotated into the solve frame
        template <typename T>
        static Vector3<T> kinematics(const Vector3<T>& pose, const Vector3<T>& u) {
            using std::cos;
            using std::sin;
            const T c = cos(pose.z());
            const T s = sin(pose.z());
            return {c * u.x() - s * u.y(), s * u.x() + c * u.y(), u.z()};
        }

        /// @brief One RK4 step of the kinematics
        template <typename T>
        static Vector3<T> rk4(const Vector3<T>& pose, const Vector3<T>& u, const double dt) {
            const Vector3<T> k1 = kinematics<T>(pose, u);
            const Vector3<T> k2 = kinematics<T>(pose + dt / 2.0 * k1, u);
            const Vector3<T> k3 = kinematics<T>(pose + dt / 2.0 * k2, u);
            const Vector3<T> k4 = kinematics<T>(pose + dt * k3, u);
            return pose + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }

        /// @brief The velocity during the step: the previous one plus the change
        template <typename T>
        static Vector3<T> velocity(const State<T>& s, const Input<T>& a) {
            return s.template tail<3>() + a * DT;
        }

        template <typename T>
        static State<T> dynamics(const State<T>& s, const Input<T>& a, const double* /*p*/) {
            const Vector3<T> u = velocity<T>(s, a);
            State<T> next;
            next << rk4<T>(s.template head<3>(), u, DT), u;
            return next;
        }

        /// @brief The pose terms: pseudo-Huber distance to the target, and the heading
        template <typename T>
        static T pose_cost(const State<T>& s, const double* p) {
            using std::cos;
            using std::sqrt;
            const double delta = p[P_HUBER_DELTA];
            const double gate  = p[P_GATE];
            const T dx         = s(0) - p[P_TARGET];
            const T dy         = s(1) - p[P_TARGET + 1];
            const T d2         = dx * dx + dy * dy;
            const T position   = delta * delta * (sqrt(1.0 + d2 / (delta * delta)) - 1.0);
            // w_head·h(θ), written out so that w_head = 0 doesn't divide by zero
            const T heading = p[P_W_HEADING] * gate * (1.0 - cos(s(2) - p[P_TARGET + 2]))
                              + p[P_W_FACE] * (1.0 - gate) * (1.0 - cos(s(2) - p[P_BEARING]));
            return p[P_W_POSITION] * position + heading;
        }

        /// @brief Stage k costs the pose s_k (reached by the previous step) and the velocity planned now. At k = 0 the
        /// pose is fixed, so its terms are a constant.
        template <typename T>
        static T cost(const State<T>& s, const Input<T>& a, const double* p) {
            const Vector3<T> u = velocity<T>(s, a);
            T effort           = T(0.0);
            T rate             = T(0.0);
            for (int i = 0; i < 3; ++i) {
                effort += p[P_W_EFFORT + i] * u(i) * u(i);
                rate += p[P_W_RATE + i] * (a(i) * DT) * (a(i) * DT);
            }
            return pose_cost<T>(s, p) + effort + rate;
        }

        /// @brief The last pose has no velocity after it, so it is costed with the stage weights
        template <typename T>
        static T terminal_cost(const State<T>& s, const double* p) {
            return pose_cost<T>(s, p);
        }

        /// @brief Obstacle clearance, r² − ‖p − oⱼ‖² ≤ 0, switched off for free slots
        template <typename T>
        static Eigen::Matrix<T, NH, 1> terminal_constraint(const State<T>& s, const double* p) {
            const double r2 = p[P_OBSTACLE_RADIUS] * p[P_OBSTACLE_RADIUS];
            Eigen::Matrix<T, NH, 1> h;
            for (int j = 0; j < MAX_OBSTACLES; ++j) {
                const T dx = s(0) - p[P_OBSTACLES + 2 * j];
                const T dy = s(1) - p[P_OBSTACLES + 2 * j + 1];
                h(j)       = p[P_ACTIVE + j] * (r2 - (dx * dx + dy * dy));
            }
            return h;
        }

        template <typename T>
        static Eigen::Matrix<T, NH, 1> constraint(const State<T>& s, const Input<T>& /*a*/, const double* p) {
            return terminal_constraint<T>(s, p);
        }
    };

}  // namespace module::planning::inverted_walk_mpc

#endif  // MODULE_PLANNING_INVERTEDMPCWALKPATH_PROBLEM_HPP
