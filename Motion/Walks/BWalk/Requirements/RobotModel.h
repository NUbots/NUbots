/**
* @file RobotModel.h
*
* Declaration of class RobotModel
*
* @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
*/

#pragma once

#include "Pose3D.h"
#include "MassCalibration.h"
#include <vector>

/**
 * @class RobotModel
 *
 * Contains information about extremities.
 */
class RobotModel
{

public:
  Pose3D limbs[MassCalibration::numOfLimbs]; /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
  Vector3<> centerOfMass; /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
  float totalMass; /**< The mass of the robot. */

  /** Constructor */
  RobotModel() : totalMass(0) {}

  /**
  * Constructs the RobotModel from given joint data.
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
    RobotModel(const std::vector<float>& joints, const MassCalibration& massCalibration);

  /**
  * Recalculates the RobotModel from given joint data.
  * @param joints The joint data.
  * @param robotDimensions The dimensions of the robot.
  * @param massCalibration The mass calibration of the robot.
  */
  void setJointData(const std::vector<float>& joints, const MassCalibration& massCalibration);
};
