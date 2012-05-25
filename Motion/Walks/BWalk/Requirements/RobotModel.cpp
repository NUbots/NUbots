/**
* @file RobotModel.cpp
* Implementation of class RobotModel.
* @author Alexander Härtl
*/

#include "RobotModel.h"
#include "ForwardKinematic.h"

RobotModel::RobotModel(const std::vector<float>& joints, const MassCalibration& massCalibration)
{
  setJointData(joints, massCalibration);
}

void RobotModel::setJointData(const std::vector<float>& joints, const MassCalibration& massCalibration)
{
  ForwardKinematic::calculateHeadChain(joints, massCalibration, limbs);

  for(int side = 0; side < 2; side++)
  {
    const bool left = side == 0;
    ForwardKinematic::calculateArmChain(left, joints, massCalibration, limbs);
    ForwardKinematic::calculateLegChain(left, joints, massCalibration, limbs);
  }

  // calculate center of mass
  centerOfMass = Vector3<>();
  totalMass = 0.0;
  for(int i = 0; i < MassCalibration::numOfLimbs; i++)
  {
    const MassCalibration::MassInfo& limb(massCalibration.masses[i]);
    totalMass += limb.mass;
    //std::cout << "Adding mass (" << i << " = [" << ((limbs[i] * limb.offset) * limb.mass).x << ", " << ((limbs[i] * limb.offset) * limb.mass).y << ", " << ((limbs[i] * limb.offset) * limb.mass).z << "]" << std::endl;
    centerOfMass += (limbs[i] * limb.offset) * limb.mass;
  }
  centerOfMass /= totalMass;
  //std::cout << "Centre of mass: [" << centerOfMass.x << ", " << centerOfMass.y << ", " << centerOfMass.z << "]" << std::endl;
}
