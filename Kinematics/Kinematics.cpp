#include "Kinematics.h"
#include <cmath>
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include "debug.h"


using namespace TransformMatrices;

//! TODO: Make this function load from a file.
bool Kinematics::LoadModel(const std::string& fileName)
{
    // Constant distances (in cm) From documentation

    // Top camera
    m_cameraTopOffsetZ = 6.79;
    m_cameraTopOffsetX = 5.39;
    m_cameraTopOffsetAngle = mathGeneral::deg2rad(0.0);

    // Bottom Camera
    m_cameraBottomOffsetZ = 2.381;
    m_cameraBottomOffsetX = 4.88;
    m_cameraBottomOffsetAngle = mathGeneral::deg2rad(40.0);

    // Neck
    m_neckOffsetZ = 12.65;

    // Hips
    m_hipOffsetY = 5.0;
    m_hipOffsetZ = 8.5;

    // Legs
    m_thighLength = 10.0;
    m_tibiaLength = 10.0;
    m_footHeight = 4.6;

    // Feet (Measured)
    m_footInnerWidth = 4.5;
    m_footOuterWidth = 5.0;
    m_footForwardLength = 9.0;
    m_footBackwardLength = 7.0;

    DHParameters tempParam;
    std::vector<Link> links;
    Matrix startTrans;
    Matrix endTrans;

    // ---------------
    // BOTTOM CAMERA
    // ---------------
    startTrans = Translation( 0, 0, m_neckOffsetZ );
    endTrans = RotX( mathGeneral::PI/2.0 )*RotY( mathGeneral::PI/2.0 );
    endTrans = endTrans * Translation(m_cameraBottomOffsetX,0,m_cameraBottomOffsetZ) * RotY(m_cameraBottomOffsetAngle);

    tempParam.alpha = 0.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"Head Yaw"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = -mathGeneral::PI/2.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"Head Pitch"));

    m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, "Bottom Camera"));

    // ---------------
    // TOP CAMERA
    // ---------------
    startTrans = Translation( 0, 0, m_neckOffsetZ );
    endTrans = RotX( mathGeneral::PI/2.0 )*RotY( mathGeneral::PI/2.0 );
    endTrans = endTrans * Translation(m_cameraTopOffsetX,0,m_cameraTopOffsetZ);

    links.clear();

    tempParam.alpha = 0.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"Head Yaw"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = -mathGeneral::PI/2.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"Head Pitch"));

    m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, "Top Camera"));

    // ---------------
    // LEFT FOOT
    // ---------------
    startTrans = Translation(0.0f, m_hipOffsetY, -m_hipOffsetZ);
    endTrans = RotZ(mathGeneral::PI)*RotY(-mathGeneral::PI/2.0)*Translation(0,0,-m_footHeight);    // To foot
    links.clear();

    tempParam.alpha = -3.0*mathGeneral::PI/4.0;
    tempParam.a = 0;
    tempParam.thetaOffset = -mathGeneral::PI/2.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"HipYawPitch"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = mathGeneral::PI/4.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"LeftHipRoll"));

    tempParam.alpha = mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"LeftHipPitch"));

    tempParam.alpha = 0;
    tempParam.a = -m_thighLength;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"LeftKneePitch"));

    tempParam.alpha = 0;
    tempParam.a = -m_tibiaLength;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"LeftAnklePitch"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"LeftAnkleRoll"));

    m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, "Left Foot"));

    // ---------------
    // RIGHT FOOT
    // ---------------
    startTrans = Translation(0.0f, -m_hipOffsetY, -m_hipOffsetZ);
    endTrans = RotZ(mathGeneral::PI)*RotY(-mathGeneral::PI/2.0)*Translation(0,0,-m_footHeight);    // To foot
    links.clear();

    tempParam.alpha = -mathGeneral::PI/4.0;
    tempParam.a = 0;
    tempParam.thetaOffset = -mathGeneral::PI/2.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"HipYawPitch"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = -mathGeneral::PI/4.0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"RightHipRoll"));

    tempParam.alpha = mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"RightHipPitch"));

    tempParam.alpha = 0;
    tempParam.a = -m_thighLength;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"RightKneePitch"));

    tempParam.alpha = 0;
    tempParam.a = -m_tibiaLength;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"RightAnklePitch"));

    tempParam.alpha = -mathGeneral::PI/2.0;
    tempParam.a = 0;
    tempParam.thetaOffset = 0;
    tempParam.d = 0;
    links.push_back(Link(tempParam,"RightAnkleRoll"));

    m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, "Left Foot"));
    return true;
}


Matrix Kinematics::CalculateTransform(Effector effectorId, const std::vector<float>& jointValues)
{
    std::vector<float> modifiedJointValues;

    switch(effectorId)
    {
        case bottomCamera:
        case topCamera:
            modifiedJointValues = ReOrderKneckJoints(jointValues);
            break;
        case leftFoot:
        case rightFoot:
            modifiedJointValues = ReOrderLegJoints(jointValues);
            break;
        default:
            modifiedJointValues = jointValues;
    }
    return m_endEffectors[effectorId].CalculateTransform(modifiedJointValues);
}

Vector3<float> Kinematics::DistanceToPoint(const Matrix& Camera2GroundTransform, double angleFromCameraCentreX, double angleFromCameraCentreY)
{
    const double nearDistance = 10;     // 10 cm
    const double farDistance = 3000;   // 3,000 cm (30 metres)

    std::vector<float> resultCartesian(3, 0.0f);

    // pre-calculate required trig values.
    double xcos = cos(angleFromCameraCentreX);
    double xsin = sin(angleFromCameraCentreX);
    double ycos = cos(angleFromCameraCentreY);
    double ysin = sin(angleFromCameraCentreY);

    // Build the near measurement vector
    Matrix nearCartCol(4,1);
    nearCartCol[0][0] = nearDistance * xcos * ycos;
    nearCartCol[1][0] = nearDistance * xsin * ycos;
    nearCartCol[2][0] = nearDistance * ysin;
    nearCartCol[3][0] = 1.0;

    // Build the far measurement vector
    Matrix farCartCol(4,1);
    farCartCol[0][0] = farDistance * xcos * ycos;
    farCartCol[1][0] = farDistance * xsin * ycos;
    farCartCol[2][0] = farDistance * ysin;
    farCartCol[3][0] = 1.0;

    // Caluculate The Transformed positions of both the near and far values.
    Matrix nearResult = Camera2GroundTransform * nearCartCol;
    Matrix farResult = Camera2GroundTransform * farCartCol;

    // Interpolate between near and far values to find the point at which z = 0 (the ground)
    double zScaleFactor = nearResult[2][0] / (nearResult[2][0] - farResult[2][0]);
    resultCartesian[0] = nearResult[0][0] + (farResult[0][0] - nearResult[0][0]) * zScaleFactor;
    resultCartesian[1] = nearResult[1][0] + (farResult[1][0] - nearResult[1][0]) * zScaleFactor;
    resultCartesian[2] = 0.0;

    // Convert back to polar coodinates.
    std::vector<float> resultSpherical(mathGeneral::Cartesian2Spherical(resultCartesian));

    Vector3<float> result;
    result[0] = resultSpherical[0];
    result[1] = resultSpherical[1];
    result[2] = resultSpherical[2];

    //! TODO: Get the right thing to output, what do we want from this function??
    return result;

}

Matrix Kinematics::CalculateCamera2GroundTransform(const Matrix& origin2SupportLegTransform, const Matrix& origin2CameraTransform)
{
    double legOffsetX = origin2SupportLegTransform[0][3];
    double legOffsetY = origin2SupportLegTransform[1][3];
    return Translation(legOffsetX,legOffsetY,0)* InverseMatrix(origin2SupportLegTransform) * origin2CameraTransform;
}

std::vector<float> Kinematics::TransformPosition(const Matrix& Camera2GroundTransform, const std::vector<float>& cameraBasedPosition)
{
    Matrix cameraBasedPosMatrix(3,1);
    cameraBasedPosMatrix[0][0] = cameraBasedPosition[0];
    cameraBasedPosMatrix[1][0] = cameraBasedPosition[1];
    cameraBasedPosMatrix[2][0] = cameraBasedPosition[2];

    Matrix resultMatrix = TransformPosition(Camera2GroundTransform, cameraBasedPosMatrix);

    // Construct the result
    std::vector<float> result(3,0.0f);
    result[0] = resultMatrix[0][0];
    result[1] = resultMatrix[1][0];
    result[2] = resultMatrix[2][0];

    return result;
}

Vector3<float> Kinematics::TransformPosition(const Matrix& Camera2GroundTransform, const Vector3<float>& cameraBasedPosition)
{
    Matrix cameraBasedPosMatrix(3,1);
    cameraBasedPosMatrix[0][0] = cameraBasedPosition.x;
    cameraBasedPosMatrix[1][0] = cameraBasedPosition.y;
    cameraBasedPosMatrix[2][0] = cameraBasedPosition.z;

    Matrix resultMatrix = TransformPosition(Camera2GroundTransform, cameraBasedPosMatrix);

    // Construct the result
    Vector3<float> result;
    result[0] = resultMatrix[0][0];
    result[1] = resultMatrix[1][0];
    result[2] = resultMatrix[2][0];

    return result;
}

Matrix Kinematics::TransformPosition(const Matrix& Camera2GroundTransform, const Matrix& cameraBasedPosition)
{
    const Matrix cameraCartesian(mathGeneral::Spherical2Cartesian(cameraBasedPosition));

    // Build the far measurement vector
    Matrix one(1,1);
    one[0][0] = 1.0;
    Matrix cameraCoordCol(4,1);
    cameraCoordCol = vertcat(cameraCartesian,one);

    // Do the transform
    Matrix resultMatrix = Camera2GroundTransform * cameraCoordCol;

    return mathGeneral::Cartesian2Spherical(resultMatrix);
}

std::vector<float> Kinematics::LookToPoint(const std::vector<float>& pointFieldCoordinates)
{
    std::vector<float> result;
    return result;
}

 Rectangle Kinematics::CalculateFootPosition(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix, Effector theFoot)
{
    if((theFoot != leftFoot) && (theFoot != rightFoot)) return Rectangle();
    Matrix totalTransform = InverseMatrix(supportFootTransformMatrix) * theFootTransformMatrix;

    // Select 2 corners to get total rect area
    Matrix frontRightPoint(4, 1);
    Matrix backLeftPoint(4, 1);
    if(theFoot == leftFoot)
    {
        frontRightPoint[1][0] = m_footInnerWidth;
        backLeftPoint[1][0] = -m_footOuterWidth;
    }
    else
    {
        frontRightPoint[1][0] = m_footOuterWidth;
        backLeftPoint[1][0] = -m_footInnerWidth;
    }

    frontRightPoint[0][0] = m_footForwardLength;
    backLeftPoint[0][0] = -m_footBackwardLength;
    frontRightPoint[3][0] = 1.0;
    backLeftPoint[3][0] = 1.0;

    Matrix frontRightResult = totalTransform * frontRightPoint;
    Matrix backLeftResult = totalTransform * backLeftPoint;
    return Rectangle(backLeftResult[0][0], frontRightResult[0][0], backLeftResult[1][0], frontRightResult[1][0]);
}

double Kinematics::CalculateRelativeFootHeight(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix, Effector theFoot)
{
    if((theFoot != leftFoot) && (theFoot != rightFoot)) return 0.0;
    Matrix totalTransform = InverseMatrix(supportFootTransformMatrix) * theFootTransformMatrix;

    // Use centre of the foot
    Matrix footCentre(4, 1, false);
    footCentre[3][0] = 1.0;

    Matrix result = totalTransform * footCentre;
    return result[2][0];
}

float Kinematics::CalculateRadialLegLength(const vector<float>& legJoints)
{
    float kneeAngle = legJoints[3];
    // Using law of cosines.
    return sqrt( pow(m_thighLength,2) + pow(m_tibiaLength,2) - 2*m_thighLength*m_tibiaLength*cos(mathGeneral::PI/2.0f - kneeAngle));
}

float Kinematics::CalculateHipPitchAngleForRelYPosition(const vector<float>& legJoints, float relYPos)
{
    float radialLegLength = CalculateRadialLegLength(legJoints);
    return asin(relYPos / radialLegLength);
}
/*
float Kinematics::CalculateRelY(const vector<float>& legJoints, float relYPos)
{
    float radialLegLength = CalculateRadialLegLength(legJoints);
    return asin(relYPos / radialLegLength);
}
*/

Vector2<float> Kinematics::TransformPositionToFoot(const Matrix& FootTransformMatrix, Vector2<float> position)
{
    Matrix footInverse = InverseMatrix(FootTransformMatrix);
    Matrix positionCol(4,1);
    positionCol[0][0] = position.x;
    positionCol[1][0] = position.y;
    positionCol[2][0] = -footInverse[2][3];
    positionCol[3][0] = 1.0f;
    Matrix result =  footInverse* positionCol;
    Vector2<float> returnResult;
    returnResult.x = result[0][0];
    returnResult.y = result[1][0];
    return returnResult;
}
