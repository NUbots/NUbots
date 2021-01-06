/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "Footstep.hpp"

#include <cmath>
#include <stdexcept>

#include "utility/math/angle.hpp"

namespace utility {
namespace motion {
    namespace splines {

        Footstep::Footstep(float foot_distance, bool is_left_support_foot) {
            if (foot_distance <= 0.0f) {
                throw std::logic_error("Footstep invalid distance");
            }

            // State initialization
            this->foot_distance        = foot_distance;
            this->is_left_support_foot = is_left_support_foot;
            left_in_world.setZero();
            right_in_world.setZero();
            reset(is_left_support_foot);
        }

        void Footstep::reset(bool is_left_support_foot) {
            this->is_left_support_foot = is_left_support_foot;
            support_to_last.x()        = 0.0f;
            if (is_left_support_foot) {
                support_to_last.y() = -foot_distance;
            }
            else {
                support_to_last.y() = foot_distance;
            }
            support_to_last.z() = 0.0f;
            support_to_next     = support_to_last;
        }

        void Footstep::resetInWorld(bool is_left_support_foot) {
            if (is_left_support_foot) {
                right_in_world.y() = -foot_distance;
            }
            else {
                left_in_world.y() = foot_distance;
            }
        }

        void Footstep::stepFromSupport(const Eigen::Vector3f& diff) {
            // Update relative diff from support foot
            support_to_last = diffInv(support_to_next);
            support_to_next = diff;
            // Update world integrated position
            if (is_left_support_foot) {
                left_in_world = poseAdd(right_in_world, diff);
            }
            else {
                right_in_world = poseAdd(left_in_world, diff);
            }
            // Update current support foot
            is_left_support_foot = !is_left_support_foot;
        }

        void Footstep::stepFromOrders(const Eigen::Vector3f& diff) {
            // Compute step diff in next support foot frame
            Eigen::Vector3f tmpDiff = Eigen::Vector3f::Zero();
            // No change in forward step
            tmpDiff.x() = diff.x();
            // Add lateral foot offset
            if (is_left_support_foot) {
                tmpDiff.y() += foot_distance;
            }
            else {
                tmpDiff.y() -= foot_distance;
            }
            // Allow lateral step only on external foot (internal foot will return to zero pose)
            if ((is_left_support_foot && diff.y() > 0.0f) || (!is_left_support_foot && diff.y() < 0.0f)) {
                tmpDiff.y() += diff.y();
            }
            // No change in turn (in order to rotate around trunk center)
            tmpDiff.z() = diff.z();

            // Make the step
            stepFromSupport(tmpDiff);
        }

        Eigen::Vector3f Footstep::poseAdd(const Eigen::Vector3f& pose, const Eigen::Vector3f& diff) const {
            const float cos_z = std::cos(pose.z());
            const float sin_z = std::sin(pose.z());
            return Eigen::Vector3f(pose.x() + diff.x() * cos_z - diff.y() * sin_z,
                                   pose.y() + diff.x() * sin_z + diff.y() * cos_z,
                                   utility::math::angle::normalizeAngle(pose.z() + diff.z()));
        }

        Eigen::Vector3f Footstep::diffInv(const Eigen::Vector3f& diff) const {
            const float cos_z = std::cos(-diff.z());
            const float sin_z = std::sin(-diff.z());
            return Eigen::Vector3f(-diff.x() * cos_z + diff.y() * sin_z,
                                   -diff.x() * sin_z - diff.y() * cos_z,
                                   -diff.z());
        }

    }  // namespace splines
}  // namespace motion
}  // namespace utility
