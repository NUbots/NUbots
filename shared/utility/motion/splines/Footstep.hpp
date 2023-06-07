/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_FOOTSTEP_HPP
#define UTILITY_MOTION_SPLINES_FOOTSTEP_HPP

#include <Eigen/Core>

namespace utility::motion::splines {

    /**
     * Footstep
     *
     * Manage humanoid footstep generation and state
     */
    class Footstep {
    public:
        Footstep() = delete;
        /**
         * Initialization with lateral foot distance and support foot
         */
        Footstep(const double& foot_distance, const bool& is_left_support_foot = true);

        /**
         * Set the lateral foot distance parameters
         */
        constexpr void set_foot_distance(const double& foot_distance_) {
            foot_distance = foot_distance_;
        }
        [[nodiscard]] constexpr double getFootDistance() const {
            return foot_distance;
        }

        /**
         * Reset to neutral position the current step (not the integrated odometry)
         */
        void reset(const bool& is_left_support_foot);

        // Reset odometry
        void resetInWorld(const bool& is_left_support_foot);

        /**
         * Current support foot
         */
        [[nodiscard]] constexpr bool is_left_support() const {
            return is_left_support_foot;
        }

        /**
         * Starting position of current flying foot in support foot frame
         */
        [[nodiscard]] const Eigen::Vector3d& get_last() const {
            return support_to_last;
        }

        /**
         * Target pose of current flying foot in support foot frame
         */
        [[nodiscard]] const Eigen::Vector3d& get_next() const {
            return support_to_next;
        }

        /**
         * Returns the odometry change of the current step.
         */
        [[nodiscard]] const Eigen::Vector3d& getOdom() const;

        /**
         * Left and right, current or next pose of foot in world initial frame
         */
        [[nodiscard]] const Eigen::Vector3d& getLeft() const {
            return left_in_world;
        }
        [[nodiscard]] const Eigen::Vector3d& getRight() const {
            return right_in_world;
        }

        /**
         * Set the target pose of current support foot during next support phase and update support foot.
         * The target foot pose diff is given with respect to next support foot pose (current flying foot
         * target).
         */
        void stepFromSupport(const Eigen::Vector3d& diff);

        /**
         * Set target pose of current support foot using diff orders.
         * Zero vector means in place walking.
         * Special handle of lateral and turn step to avoid foot collision.
         */
        void step_from_orders(const Eigen::Vector3d& diff);

    private:
        /**
         * Static lateral distance between the feet
         */
        double foot_distance;

        /**
         * Current support foot
         * (left or right)
         */
        bool is_left_support_foot;

        /**
         * Pose diff [dx, dy, dtheta] from support foot to flying foot last and next position
         */
        Eigen::Vector3d support_to_last = Eigen::Vector3d::Zero();
        Eigen::Vector3d support_to_next = Eigen::Vector3d::Zero();

        /**
         * Pose integration of left and right foot in initial frame.
         * Set at "future" state taking into account next expected fot pose.
         */
        Eigen::Vector3d left_in_world  = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_in_world = Eigen::Vector3d::Zero();
    };

}  // namespace utility::motion::splines
#endif
