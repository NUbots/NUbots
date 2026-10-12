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
#include "Track.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace module::tools::walk_track {

    Track::Track(const Eigen::Vector3d& start,
                 const double heading_offset,
                 const std::vector<Segment>& segments,
                 const double speed_scale)
        : heading_offset(heading_offset) {
        if (speed_scale <= 0.0) {
            throw std::invalid_argument("A track's speed scale must be positive");
        }

        first.pose      = Eigen::Vector3d(start.x(), start.y(), start.z() + heading_offset);
        first.direction = start.z();

        Eigen::Vector2d position = start.head<2>();
        double direction         = start.z();
        for (const auto& segment : segments) {
            if (segment.duration < 0.0 || segment.speed < 0.0) {
                throw std::invalid_argument("A track segment's duration and speed can't be negative");
            }
            Piece piece;
            piece.segment    = Segment{segment.speed * speed_scale,
                                    segment.acceleration * speed_scale * speed_scale,
                                    segment.curvature,
                                    segment.turn,
                                    segment.duration / speed_scale};
            piece.start_time = total_duration;
            piece.position   = position;
            piece.direction  = direction + segment.turn;
            pieces.push_back(piece);

            const TargetState end = state(piece, piece.segment.duration);
            position              = end.pose.head<2>();
            direction             = end.direction;
            total_duration += piece.segment.duration;
        }

        last.pose      = Eigen::Vector3d(position.x(), position.y(), direction + heading_offset);
        last.direction = direction;
    }

    std::pair<double, double> Track::travelled(const Segment& segment, double tau) {
        // A deceleration stops the target partway through, after which it stays put
        if (segment.acceleration < 0.0) {
            tau = std::min(tau, -segment.speed / segment.acceleration);
        }
        return {segment.speed * tau + 0.5 * segment.acceleration * tau * tau,
                std::max(segment.speed + segment.acceleration * tau, 0.0)};
    }

    TargetState Track::state(const Piece& piece, const double tau) const {
        const auto [s, v]       = travelled(piece.segment, tau);
        const double kappa      = piece.segment.curvature;
        const double psi0       = piece.direction;
        const double psi        = psi0 + kappa * s;
        const double c0         = std::cos(psi0);
        const double s0         = std::sin(psi0);
        Eigen::Vector2d forward = Eigen::Vector2d(c0, s0) * s;
        // Arc: integrate (cos ψ, sin ψ) ds with ψ = ψ0 + κs. Below a tiny turn, the straight line is exact enough and
        // avoids dividing by κ.
        if (std::abs(kappa * s) > 1e-9) {
            forward = Eigen::Vector2d(std::sin(psi) - s0, c0 - std::cos(psi)) / kappa;
        }

        TargetState target;
        target.direction = psi;
        target.pose =
            Eigen::Vector3d(piece.position.x() + forward.x(), piece.position.y() + forward.y(), psi + heading_offset);
        target.velocity = Eigen::Vector2d(std::cos(psi), std::sin(psi)) * v;
        return target;
    }

    TargetState Track::at(const double t) const {
        if (pieces.empty() || t <= 0.0) {
            return pieces.empty() ? first : state(pieces.front(), 0.0);
        }
        if (t >= total_duration) {
            return last;
        }
        // The last piece starting at or before t
        const auto piece =
            std::prev(std::upper_bound(pieces.begin(), pieces.end(), t, [](const double time, const Piece& p) {
                return time < p.start_time;
            }));
        return state(*piece, t - piece->start_time);
    }

}  // namespace module::tools::walk_track
