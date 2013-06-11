#include "cameramatrix.h"
#include <math.h>
#include <QDebug>

CameraMatrix::CameraMatrix()
{
    leftCam = Matrix(4,4,true);
    rightCam = Matrix(4,4,true);
}

CameraMatrix::CameraMatrix(double* paramaters,double* joints,double x, double y, double bearing, bool bottomCamera)
{
    // Initialise the matrix variables
    leftCam = Matrix(4,4,true);
    rightCam = Matrix(4,4,true);

    // Set the constants
    const double TORSO_HEIGHT = 21.55,HEAD_DEPTH = 5.39,HEAD_HEIGHT = 6.79;
    const double FOOT_HEIGHT = 4.6,TIBIA_LENGTH = 10,THIGH_LENGTH = 10;
    const double LEFT_HIP_OFFSET = -5, RIGHT_HIP_OFFSET = 5;

    // Declare all the transformation matrices
    Matrix lPositionTrans,rPositionTrans,bearingRot,footHeightTrans,tibiaTrans,thighTrans, torsoTrans;
    Matrix headYawRot,headPitchRot,headTrans;
    Matrix lHipYawPitchRot,lHipRollRot,lHipPitchRot,lKneePitchRot,lAnklePitchRot,lAnkleRollRot,lHipOffsetTrans;
    Matrix rHipYawPitchRot,rHipRollRot,rHipPitchRot,rKneePitchRot,rAnklePitchRot,rAnkleRollRot,rHipOffsetTrans;
    Matrix headYawParamaterRot,headPitchParamaterRot,lHipPitchParamaterRot,lHipRollParamaterRot,
        rHipPitchParamaterRot,rHipRollParamaterRot;


    // Set all the transformation matrices
    bearingRot = Rot(bearing, 'z');
    footHeightTrans = Trans(0,0,FOOT_HEIGHT);
    tibiaTrans = Trans(0,0,TIBIA_LENGTH);
    thighTrans = Trans(0,0,THIGH_LENGTH);
    torsoTrans = Trans(0,0,TORSO_HEIGHT);
    headYawRot = Rot(joints[0],'z');

    if (bottomCamera)
    {
        headPitchRot = Rot(joints[1]+40, 'y');
        headTrans = Trans(HEAD_DEPTH-0.51,0,HEAD_HEIGHT-4.409);
    }
    else
    {
        headPitchRot = Rot(joints[1], 'y');
        headTrans = Trans(HEAD_DEPTH,0,HEAD_HEIGHT);
    }

    lHipYawPitchRot = Rot(joints[10]/2,'z') * Rot(joints[10]/2,'y');
    lHipRollRot = Rot(joints[11],'x');
    lHipPitchRot = Rot(joints[12],'y');
    lKneePitchRot = Rot(joints[13],'y');
    lAnklePitchRot = Rot(joints[14],'y');
    lAnkleRollRot = Rot(joints[15],'x');
    lHipOffsetTrans = Trans(0,LEFT_HIP_OFFSET,0);

    rHipYawPitchRot = Rot(-joints[16]/2,'z') * Rot(joints[16]/2,'y');
    rHipRollRot = Rot(joints[17],'x');
    rHipPitchRot = Rot(joints[18],'y');
    rKneePitchRot = Rot(joints[19],'y');
    rAnklePitchRot = Rot(joints[20],'y');
    rAnkleRollRot = Rot(joints[21],'x');
    rHipOffsetTrans = Trans(0,RIGHT_HIP_OFFSET,0);

    headYawParamaterRot = Rot(paramaters[0],'z');
    headPitchParamaterRot = Rot(paramaters[1],'y');
    lHipPitchParamaterRot = Rot(paramaters[2],'y');
    lHipRollParamaterRot = Rot(paramaters[3],'x');
    rHipPitchParamaterRot = Rot(paramaters[4],'y');
    rHipRollParamaterRot = Rot(paramaters[5],'x');

    //Find position of each foot by working backwards from hip
    Matrix leftLegTemp = Matrix(4,1);
    Matrix rightLegTemp = Matrix(4,1);
    leftLegTemp[3][0] = 1;
    rightLegTemp[3][0] = 1;

    leftLegTemp = bearingRot * InverseMatrix(footHeightTrans * lAnkleRollRot * lAnklePitchRot * tibiaTrans * lKneePitchRot *
               thighTrans * lHipPitchRot * lHipRollRot * lHipYawPitchRot * lHipOffsetTrans) * leftLegTemp;

    rightLegTemp = bearingRot * InverseMatrix(footHeightTrans * rAnkleRollRot * rAnklePitchRot * tibiaTrans * rKneePitchRot *
               thighTrans * rHipPitchRot * rHipRollRot * rHipYawPitchRot * rHipOffsetTrans) * rightLegTemp;

    // Create both position transformations based on position of feet
    lPositionTrans = Trans(x+leftLegTemp[0][0],y+leftLegTemp[1][0],0);
    rPositionTrans = Trans(x+rightLegTemp[0][0],y+rightLegTemp[1][0],0);

    //Create total leg and torso transformations

    Matrix totalLeftLegTransformation = lPositionTrans * bearingRot * footHeightTrans * lAnkleRollRot * lAnklePitchRot *
                                        tibiaTrans * lKneePitchRot * thighTrans * lHipPitchRot * lHipPitchParamaterRot *
                                        lHipRollRot * lHipRollParamaterRot * lHipYawPitchRot * lHipOffsetTrans;

    Matrix totalRightLegTransformation = rPositionTrans * bearingRot * footHeightTrans * rAnkleRollRot * rAnklePitchRot *
                                        tibiaTrans * rKneePitchRot * thighTrans * rHipPitchRot * rHipPitchParamaterRot *
                                        rHipRollRot * rHipRollParamaterRot * rHipYawPitchRot * rHipOffsetTrans;

    Matrix totalTorsoTransformation = torsoTrans * headYawRot * headYawParamaterRot * headPitchRot * headPitchParamaterRot
                                      * headTrans;


    // Create total transformation matrices. Note: inverted to provide World-to-Camera coordinates
    leftCam = InverseMatrix(totalLeftLegTransformation * totalTorsoTransformation);
    rightCam = InverseMatrix(totalRightLegTransformation * totalTorsoTransformation);



    // Create matrix transform axis from robot coordinate system
    // to vision coordinate system (NOT screen coordinates)


    /*
    Robot Coordinate System     Vision Coordinate System

             z                    y
             |  x                  |  z
             | /                   | /
       y_____|/                    |/_____ x


    */

    Matrix axisTransform = Matrix(4,4);
    axisTransform[0][1] = -1;
    axisTransform[1][2] = 1;
    axisTransform[2][0] = 1;
    axisTransform[3][3] = 1;

    //Multiply camera by axis transformation to get World-to-Vision transformation
    leftCam = axisTransform * leftCam;
    rightCam = axisTransform * rightCam;
}
Matrix CameraMatrix::Rot(double bearing, char axis)
{
    Matrix RotationMatrix = Matrix(4,4,true);

    // Convert bearing to Rad's.
    // As all calculations are on frames, multiply by -1
    bearing = -bearing * M_PI / 180;

    // Create matrix for rotation by 'bearing' around given axis
    switch (axis)
    {
        case 'x' :
            RotationMatrix[1][1] = cos(bearing);
            RotationMatrix[1][2] = sin(bearing);
            RotationMatrix[2][1] = -sin(bearing);
            RotationMatrix[2][2] = cos(bearing);
                break;
        case 'y' :
            RotationMatrix[0][0] = cos(bearing);
            RotationMatrix[0][2] = -sin(bearing);
            RotationMatrix[2][0] = sin(bearing);
            RotationMatrix[2][2] = cos(bearing);
                break;
        case 'z' :
            RotationMatrix[0][0] = cos(bearing);
            RotationMatrix[0][1] = sin(bearing);
            RotationMatrix[1][0] = -sin(bearing);
            RotationMatrix[1][1] = cos(bearing);
                break;
        }
    return RotationMatrix;
}
Matrix CameraMatrix::Trans(double x, double y, double z)
{
        // Create translation matrix for given x,y,z.
        Matrix std::vector(4,1);
        std::vector[0][0] = x;
        std::vector[1][0] = y;
        std::vector[2][0] = z;
        std::vector[3][0] = 1;
        Matrix TranslationMatrix = Matrix(4,4,true);
        TranslationMatrix.setCol(3,std::vector);
        return TranslationMatrix;

}

