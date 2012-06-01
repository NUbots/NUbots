/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include "Tools/Math/Vector3.h"


class MassCalibration
{
public:
  enum Limb
    {
    neck,
    head,
    shoulderLeft,
    bicepsLeft,
    elbowLeft,
    foreArmLeft,
    shoulderRight,
    bicepsRight,
    elbowRight,
    foreArmRight,
    pelvisLeft,
    hipLeft,
    thighLeft,
    tibiaLeft,
    ankleLeft,
    footLeft,
    pelvisRight,
    hipRight,
    thighRight,
    tibiaRight,
    ankleRight,
    footRight,
    torso,
    numOfLimbs
    };

  /**
  * Information on the mass distribution of a limb of the robot.
  */
  class MassInfo
  {
  public:
    float mass; /**< The mass of this limb. */
    Vector3<> offset; /**< The offset of the center of mass of this limb relative to its hinge. */

    /**
    * Default constructor.
    */
    MassInfo() : mass(0), offset() {}
  };

  MassCalibration()
  {
      masses[neck].mass = 59.3;
      masses[neck].offset = Vector3<>(-0.02,0.17,-25.56);

      masses[head].mass = 496.71;
      masses[head].offset = Vector3<>(2.83,-0.93,52.56);

      masses[shoulderLeft].mass = 69.96;
      masses[shoulderLeft].offset = Vector3<>(-1.78,-24.96,0.18);

      masses[bicepsLeft].mass = 123.09;
      masses[bicepsLeft].offset = Vector3<>(18.85,5.77,0.62);

      masses[elbowLeft].mass = 59.59;
      masses[elbowLeft].offset = Vector3<>(-25.73,-0.01,-0.2);

      masses[foreArmLeft].mass = 112.82;
      masses[foreArmLeft].offset = Vector3<>(69.92, 0.96, -1.14);

      masses[shoulderRight].mass = 69.96;
      masses[shoulderRight].offset = Vector3<>(-1.78, 24.96, 0.18);
        
      masses[bicepsRight].mass = 123.09;
      masses[bicepsRight].offset = Vector3<>(18.85, -5.77, 0.62);
        
      masses[elbowRight].mass = 59.59;
      masses[elbowRight].offset = Vector3<>(-25.73, 0.01, -0.2);

      masses[foreArmRight].mass = 112.82;
      masses[foreArmRight].offset = Vector3<>(69.92, -0.96, -1.14);
        
      masses[pelvisLeft].mass = 72.44;
      masses[pelvisLeft].offset = Vector3<>(-7.17, -11.87, 27.05);

      masses[hipLeft].mass = 135.3;
      masses[hipLeft].offset = Vector3<>(-16.49, 0.29, -4.75);

      masses[thighLeft].mass = 397.98;
      masses[thighLeft].offset = Vector3<>(1.31, 2.01, -53.86);
      
      masses[tibiaLeft].mass = 297.06;
      masses[tibiaLeft].offset = Vector3<>(4.71, 2.1, -48.91);

      masses[ankleLeft].mass = 138.92;
      masses[ankleLeft].offset = Vector3<>(1.42, 0.28, 6.38);

      masses[footLeft].mass = 163.04;
      masses[footLeft].offset = Vector3<>(24.89, 3.3, -32.08);

      masses[pelvisRight].mass = 72.44;
      masses[pelvisRight].offset = Vector3<>(-7.17, 11.87, 27.05);
      
      masses[hipRight].mass = 135.3;
      masses[hipRight].offset = Vector3<>(-16.49, -0.29, -4.75);

      masses[thighRight].mass = 397.98;
      masses[thighRight].offset = Vector3<>(1.31, -2.01, -53.86);

      masses[tibiaRight].mass = 297.06;
      masses[tibiaRight].offset = Vector3<>(4.71, -2.1, -48.91);

      masses[ankleRight].mass = 138.92;
      masses[ankleRight].offset = Vector3<>(1.42, -0.28, 6.38);

      masses[footRight].mass = 163.04;
      masses[footRight].offset = Vector3<>(24.89, -3.3, -32.08);

      masses[torso].mass = 1026.28;
      masses[torso].offset = Vector3<>(-4.8, 0.06, 127.27);

  }

  MassInfo masses[numOfLimbs]; /**< Information on the mass distribution of all joints. */
};
