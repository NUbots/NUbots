#include "Kinematics.h"
#include <cmath>
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"

using namespace TransformMatrices;

// Constant distances (in cm) From documentation

// Top camera
const float Kinematics::cameraTopOffsetZ = 6.79;
const float Kinematics::cameraTopOffsetX = 5.39;
const float Kinematics::cameraTopOffsetAngle = mathGeneral::deg2rad(0.0);

// Bottom Camera
const float Kinematics::cameraBottomOffsetZ = 2.381;
const float Kinematics::cameraBottomOffsetX = 4.88;
const float Kinematics::cameraBottomOffsetAngle = mathGeneral::deg2rad(40.0);

// Neck
const float Kinematics::neckOffsetZ = 12.65;

// Hips 
const float Kinematics::hipOffsetY = 5.0;
const float Kinematics::hipOffsetZ = 8.5;

// Legs
const float Kinematics::thighLength = 10.0;
const float Kinematics::tibiaLength = 10.0;
const float Kinematics::footHeight = 4.6;

/*
void Kinematics::TransformPosition(double distance,double bearing,double elevation, double *transformedDistance,double *transformedBearing,double *transformedElevation){
    std::vector<float> position3D = TransformPosition(distance,bearing,elevation);
    double x,y,z;
    x = position3D[0];
    y = position3D[1];
    z = position3D[2];
    if(x == 0) x = 0.000001; // Prevent divide by zero.

    *transformedDistance = sqrt(x*x + y*y + z*z);
    *transformedBearing = atan2(y,x);
    if(*transformedDistance == 0) *transformedDistance = 0.000001; // Prevent extremely unlikely divide by zero.
    *transformedElevation = asin(z / *transformedDistance);
}



std::vector<float> Kinematics::TransformPosition(double distance,double bearing,double elevation)
{

    // Definitions:
    // - Angles in radians.
    // - Distances in centimeters.
    // - Elevation - -ve angle is down.
    // - Bearing - +ve angle is left.
    // - Yaw - +ve angle is left.
    // - Pitch - +ve angle is down.
    // - Roll - +ve angle is clockwise.

    // Convert the spherical (radius, elevation, bearing) coordinates to Cartesian (x,y,z) 
    //    => x axis - Horizontal from center forwards
    //    => y axis - Horizontal from center to left
    //    => z axis - Vertical from bottom to top

    double x = distance * cos(bearing) * cos(elevation);
    double y = distance * sin(bearing) * cos(elevation);
    double z = distance * sin(elevation);

    double tempX, tempY, tempZ;
    
    float cameraOffsetZ, cameraOffsetX, cameraAngleOffset;

    if(cameraNumber == CAMERA_BOTTOM){
      cameraAngleOffset = cameraBottomOffsetAngle;
      cameraOffsetZ = cameraBottomOffsetZ;
      cameraOffsetX = cameraBottomOffsetX;
    } else {
      cameraAngleOffset = cameraTopOffsetAngle;
      cameraOffsetZ = cameraTopOffsetZ;
      cameraOffsetX = cameraTopOffsetX;
    }
    
    // TODO:
    // Note:  This angle offset should really be a rotation about the original value, as rotation is not really about the robots neck
    //        but about the center of the head (?).
    float tempHeadPitch = headPitch + cameraAngleOffset;

    // Translate to base of neck.
    x = x + cameraOffsetX;
    z = z + cameraOffsetZ;

    // Rotate for head pitch. (Rotate about y-axis)
    tempX = x;
    tempY = y;
    tempZ = z;
    x = tempX*cos(tempHeadPitch)+tempZ*sin(tempHeadPitch);
    y = tempY;
    z = -tempX*sin(tempHeadPitch)+tempZ*cos(tempHeadPitch);

    // Rotate for Head yaw. (Rotate about z-axis)
    tempX = x;
    tempY = y;
    tempZ = z;
    x = tempX*cos(headYaw)-tempY*sin(headYaw);
    y = tempX*sin(headYaw)+tempY*cos(headYaw);
    z = tempZ;

    

    // Coordinate frame should now be sitting the base of the neck, aligned with the torso coords.
    // Translate to base of torso.
    x = x;
    y = y;
    z = z + neckOffsetZ + hipOffsetZ;
    
    // rotate for body pitch. (Rotate about y-axis)
    tempX = x;
    tempY = y;
    tempZ = z;
    x = tempX*cos(bodyPitch)+tempZ*sin(bodyPitch);
    y = tempY;
    z = -tempX*sin(bodyPitch)+tempZ*cos(bodyPitch);

    // Rotate for body roll. (Rotate about x-axis)
    tempX = x;
    tempY = y;
    tempZ = z;
    x = x;
    y = tempY*cos(bodyRoll)-tempZ*sin(bodyRoll);
    z = tempY*sin(bodyRoll)+tempZ*cos(bodyRoll);
    
    
 
    // Pack results into vector to return
    std::vector<float> position3D;
    position3D.resize(3);
    position3D[0] = x;
    position3D[1] = y;
    position3D[2] = z;

    return position3D;
}
*/


std::vector<float> Kinematics::CalculateLeftFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll){
  
  Matrix temp;
  std::vector<float> position3D;
  position3D.resize(3);

  temp = Translation(0.0f, hipOffsetY, -hipOffsetZ);                    // To top of leg
  temp = temp * ModifiedDH(-3.0*mathGeneral::PI/4.0, 0, thetaHipYawPitch-mathGeneral::PI/2.0, 0); // Hip Yaw Pitch
  temp = temp * ModifiedDH(-mathGeneral::PI/2.0, 0, thetaHipRoll+mathGeneral::PI/4.0, 0);         // Hip Roll
  temp = temp * ModifiedDH(mathGeneral::PI/2.0, 0, thetaHipPitch, 0);                // Hip Pitch
  temp = temp * ModifiedDH(0, -thighLength, thetaKneePitch, 0);         // Knee Pitch
  temp = temp * ModifiedDH(0, -tibiaLength, thetaAnklePitch, 0);        // Ankle Pitch
  temp = temp * ModifiedDH(-mathGeneral::PI/2.0, 0, thetaAnkleRoll, 0);              // Ankle Roll
  temp = temp * RotZ(mathGeneral::PI)*RotY(-mathGeneral::PI/2.0)*Translation(0,0,-footHeight);    // To foot

  position3D[0] = (float)temp[0][3];
  position3D[1] = (float)temp[1][3];
  position3D[2] = (float)temp[2][3];

  return position3D; 
}

std::vector<float> Kinematics::CalculateRightFootPosition(float thetaHipYawPitch, float thetaHipRoll, float thetaHipPitch, float thetaKneePitch, float thetaAnklePitch, float thetaAnkleRoll){

  Matrix temp;
  std::vector<float> position3D;
  position3D.resize(3);
  temp = Translation(0.0f, -hipOffsetY, -hipOffsetZ);                   // To top of leg
  temp = temp * ModifiedDH(-mathGeneral::PI/4.0, 0, thetaHipYawPitch-mathGeneral::PI/2.0, 0);     // Hip Yaw Pitch
  temp = temp * ModifiedDH(-mathGeneral::PI/2.0, 0, thetaHipRoll-mathGeneral::PI/4.0, 0);         // Hip Roll
  temp = temp * ModifiedDH(mathGeneral::PI/2.0, 0, thetaHipPitch, 0);                // Hip Pitch
  temp = temp * ModifiedDH(0, -thighLength, thetaKneePitch, 0);         // Knee Pitch
  temp = temp * ModifiedDH(0, -tibiaLength, thetaAnklePitch, 0);        // Ankle Pitch
  temp = temp * ModifiedDH(-mathGeneral::PI/2.0, 0, thetaAnkleRoll, 0);              // Ankle Roll
  temp = temp * RotZ(mathGeneral::PI)*RotY(-mathGeneral::PI/2.0)*Translation(0,0,-footHeight);    // To foot
  position3D[0] = (float)temp[0][3];
  position3D[1] = (float)temp[1][3];
  position3D[2] = (float)temp[2][3];
  return position3D; 
}


/*
float Kinematics::CalculateHeightOfOrigin()
{
  float originHeight;
  if (touchLeftFootOnGround == true && touchRightFootOnGround == false){ 
    // Left foot is support leg
    std::vector<float> legPosition = GetLeftFootPosition();
    originHeight = fabs(legPosition[2]);
  } else if (touchLeftFootOnGround == false && touchRightFootOnGround == true) {
    // Right foot is support leg
    std::vector<float> legPosition = GetRightFootPosition();
    originHeight = fabs(legPosition[2]);
  } else if (touchLeftFootOnGround == true && touchRightFootOnGround == true) {
    // Both feet are support legs, or can't decide
    std::vector<float> leftLegPosition = GetLeftFootPosition();
    std::vector<float> rightLegPosition = GetRightFootPosition();
    originHeight = fabs( (leftLegPosition[2] + rightLegPosition[2]) / 2 );
  } else {
    // Robot is not stanging on its legs
    originHeight = 0;    
  }
  return originHeight;
}
*/
