#ifndef CAMERAMATRIX_H
#define CAMERAMATRIX_H

#include "Tools/Math/Matrix.h"

/*!
  * A class to create a camera matrix for a robot
  */

class CameraMatrix
{
public:
    /*!
      @brief Default constructor for CameraMatrix class
      */
    CameraMatrix();
    /*!
      @brief A constructor for CameraMatrix class .
      @param paramaters An array of calibration paramaters.
      @param joints An array of joint angle values.
      @param x The x ordinate of the robot.
      @param y The y ordinate of the robot.
      @param theta The bearing of the robot.
      @param bottomCamera The camera being used.
      */
    CameraMatrix(double* paramaters,double* joints,double x=0, double y=0, double bearing=0, bool bottomCamera = false);

    /*!
      @brief Accepts an angle and an axis and returns a 4x4 rotation matrix.
      @param angle The angle to rotate by.
      @param axis The axis to rotate around.
      */
    Matrix Rot(double angle, char axis);

    /*!
      @brief Accepts three values and returns a 4x4 translation matrix.
      @param x The amount to translate by in the x direction.
      @param y The amount to translate by in the y direction.
      @param z The amount to translate by in the z direction.
      */
    Matrix Trans(double x=0, double y=0, double z=0);

    /*!
      @brief Returns the left camera matrix
      */
    Matrix getLeft() {return leftCam;}

    /*!
      @brief Returns the right camera matrix
      */
    Matrix getRight() {return rightCam;}

private:
    Matrix leftCam;                     //!< Left camera matrix
    Matrix rightCam;                    //!< Right camera matrix
};


#endif // CAMERAMATRIX_H
