// Encapsulates the model of a robot for gait purposes.
// File: RobotModel.h
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Ensure header is only included once
#ifndef CAP_ROBOTMODEL_H
#define CAP_ROBOTMODEL_H

// Includes
#include <Eigen/Core>

#include "../WalkConfig.h"
#include "Action.h"

#include "utility/math/geometry/Frame.h"

namespace gait {
namespace contrib {

    // RobotModel class
    class RobotModel {
    public:
        // Constructor
        explicit RobotModel(const gait::WalkConfig& capConfig);

        // Configuration variables
        const gait::WalkConfig& getConfig() const {
            return config;
        }

        // Reset function
        void reset(bool resetModel = true);

        // Robot odometry
        // Resets the CoM odometry to a zero position in the world frame
        void resetOdom();
        // Set the CoM odometry to a particular position and fused yaw in the world frame
        void setOdom(double comX, double comY, double fYaw);

        // Update the robot model with new pose and fused angle data
        void update(const Pose& pose, double fusedX, double fusedY);

        // Sets the support leg sign of the model and updates the required associated frames
        void setSupportLeg(int supportLegSign);

        // Robot model CoM vector (vector from footstep frame to CoM in footstep frame coordinates)
        // Returns the vector pointing from the support footstep frame to the CoM in support footstep frame coordinates
        Eigen::Vector3d suppComVector() const;
        // Returns the vector pointing from the free footstep frame to the CoM in free footstep frame coordinates
        Eigen::Vector3d freeComVector() const;
        // Returns the vector pointing from the left footstep frame to the CoM in left footstep frame coordinates
        Eigen::Vector3d leftComVector() const;
        // Returns the vector pointing from the right footstep frame to the CoM in right footstep frame coordinates
        Eigen::Vector3d rightComVector() const;

        // Robot model step vector (vector from footstep frame to other footstep frame in footstep frame coordinates)
        // Returns the vector pointing from the support footstep frame to the free footstep frame in support footstep
        // frame coordinates
        Eigen::Vector3d suppStepVector() const;
        // Returns the vector pointing from the free footstep frame to the support footstep frame in free footstep frame
        // coordinates
        Eigen::Vector3d freeStepVector() const;
        // Returns the vector pointing from the left footstep frame to the right footstep frame in left footstep frame
        // coordinates
        Eigen::Vector3d leftStepVector() const;
        // Returns the vector pointing from the right footstep frame to the left footstep frame in right footstep frame
        // coordinates
        Eigen::Vector3d rightStepVector() const;

        // Robot model swing vector (vector from CoM to other footstep frame in footstep frame coordinates)
        // Returns the vector pointing from the CoM to the free footstep frame in support foot coordinates
        Eigen::Vector3d suppSwingVector() const;
        // Returns the vector pointing from the CoM to the support footstep frame in free foot coordinates
        Eigen::Vector3d freeSwingVector() const;
        // Returns the vector pointing from the CoM to the right footstep frame in left foot coordinates
        Eigen::Vector3d leftSwingVector() const;
        // Returns the vector pointing from the CoM to the left footstep frame in right foot coordinates
        Eigen::Vector3d rightSwingVector() const;

        // Robot model step yaw (difference in fused yaw from footstep frame to other footstep frame in the global
        // coordinate frame) Returns the difference in fused yaw from the support footstep frame to the free footstep
        // frame in the global coordinate frame
        double suppStepYaw() const;
        // Returns the difference in fused yaw from the free footstep frame to the support footstep frame in the global
        // coordinate frame
        double freeStepYaw() const;
        // Returns the difference in fused yaw from the left footstep frame to the right footstep frame in the global
        // coordinate frame
        double leftStepYaw() const;
        // Returns the difference in fused yaw from the right footstep frame to the left footstep frame in the global
        // coordinate frame
        double rightStepYaw() const;

        // Static conversion functions
        // Returns the fused yaw of a quaternion orientation in the range (-pi,pi]
        static double fusedYaw(const Eigen::Quaterniond& q);
        // Converts a pure fused pitch/roll rotation to a quaternion
        static Eigen::Quaterniond quatFromFusedPR(double fusedX, double fusedY);

    protected:
        // Initialisation of kinematic model
        // Initialises the entire kinematic chain
        void initKinematicModel();
        // Initialises the reference frame hierarchy of the kinematic chain
        void initKinematicHierarchy();
        // Initialises the translations of the kinematic chain
        void initKinematicTranslations();
        // Initialises the rotations of the kinematic chain
        void initKinematicRotations();

        // Update functions
        // Updates the left/right footstep frames based on the left/right foot floor point frames
        void updateLeftRightFootstep();
        // Updates the support and free footstep frames as being the left or right footstep frames, based on the current
        // support leg sign
        void updateSuppFreeFootstep();

        // Set robot pose functions
        // Set the kinematic pose of the modelled robot and apply it
        void setPose(const Pose& pose) {
            this->pose = pose;
            applyPose();
        }
        // Applies the currently set joint angles to the kinematic model
        void applyPose();
        // Set the fused pitch/roll orientation of the model (fused yaw is part of the odometry)
        void alignModel(double fusedX, double fusedY);

        // Helper functions
        double getStepYaw(const utility::math::geometry::Frame& fromFootstep,
                          const utility::math::geometry::Frame& toFootstep) const;

        // Configuration variables
        const gait::WalkConfig& config;
        void robotSpecCallback() {
            initKinematicTranslations();
        }

        // Robot pose (encapsulates the currently set joint angles, applied to the kinematic frames by the applyPose()
        // function)
        Pose pose;

    public:
        // Root frame
        // Global frame placed at the centre between the two hip joints (the nominal "base" reference point of the
        // robot)
        utility::math::geometry::Frame base;

        // Footstep frames
        // Global frame placed at the left foot floor point, with only the yaw component of the rotation of the left
        // foot (left footstep frame)
        utility::math::geometry::Frame leftFootstep;
        // Global frame placed at the right foot floor point, with only the yaw component of the rotation of the right
        // foot (right footstep frame)
        utility::math::geometry::Frame rightFootstep;
        // Frame placed at either the left or right footstep frame depending on the current support leg (support
        // footstep frame)
        utility::math::geometry::Frame suppFootstep;
        // Frame placed at either the left or right footstep frame depending on the current free leg (free footstep
        // frame)
        utility::math::geometry::Frame freeFootstep;

        // Trunk frames
        // Frame placed at the CoM of the robot
        utility::math::geometry::Frame com;
        // Frame placed at the trunk link of the robot as per the tf (i.e. as defined by the URDF model)
        utility::math::geometry::Frame trunkLink;

        // Head frames
        // Frame placed at the neck joint, aligned with the head
        utility::math::geometry::Frame neck;
        // Frame placed at the centre of the head, aligned with the head
        utility::math::geometry::Frame head;

        // Left arm frames
        // Frame placed at the left shoulder joint, aligned with the upper arm
        utility::math::geometry::Frame lShoulder;
        // Frame placed at the left elbow joint, aligned with the lower arm
        utility::math::geometry::Frame lElbow;
        // Frame placed at the left hand, aligned with the lower arm
        utility::math::geometry::Frame lHand;

        // Right arm frames
        // Frame placed at the right shoulder joint, aligned with the upper arm
        utility::math::geometry::Frame rShoulder;
        // Frame placed at the right elbow joint, aligned with the lower arm
        utility::math::geometry::Frame rElbow;
        // Frame placed at the right hand, aligned with the lower arm
        utility::math::geometry::Frame rHand;

        // Left leg frames
        // Frame placed at the left hip joint, aligned with the upper leg
        utility::math::geometry::Frame lHip;
        // Frame placed at the left knee joint, aligned with the lower leg
        utility::math::geometry::Frame lKnee;
        // Frame placed at the left ankle joint, aligned with the foot
        utility::math::geometry::Frame lAnkle;
        // Frame placed at the point on the bottom of the left foot that is assumed to be in contact with the ground
        // when left is the support foot, aligned with the foot
        utility::math::geometry::Frame lFootFloorPoint;

        // Right leg frames
        // Frame placed at the right hip joint, aligned with the upper leg
        utility::math::geometry::Frame rHip;
        // Frame placed at the right knee joint, aligned with the lower leg
        utility::math::geometry::Frame rKnee;
        // Frame placed at the right ankle joint, aligned with the foot
        utility::math::geometry::Frame rAnkle;
        // Frame placed at the point on the bottom of the right foot that is assumed to be in contact with the ground
        // when right is the support foot, aligned with the foot
        utility::math::geometry::Frame rFootFloorPoint;

        // TODO: A current shortcoming of the robot model is that x-offsets from the hip midpoint to the shoulder
        // line/arm axis/head axis aren't considered.

        // Support leg information
        enum { LEFT_LEG = -1, RIGHT_LEG = 1 };
        // Sign of the current support leg (1 = Right leg, -1 = Left leg)
        int supportLegSign;
        // Flag whether a support exchange happened during a call to update()
        bool supportExchange;
        // Flag whether a support exchange is currently disallowed (used to enforce a hysteresis)
        bool supportExchangeLock;
    };

}  // namespace contrib
}  // namespace gait

#endif
