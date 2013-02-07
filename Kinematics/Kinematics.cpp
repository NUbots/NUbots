#include "Kinematics.h"
#include <cmath>
#include "Tools/Math/TransformMatrices.h"
#include "Tools/Math/General.h"
#include "NUPlatform/NUCamera.h"
#include "nubotdataconfig.h"
#include "debug.h"

#include <cstdlib>


using namespace TransformMatrices;

bool Kinematics::LoadModel()
{
    const std::string default_file = (CONFIG_DIR + string("Motion/kinematics") + ".cfg");
    std::ifstream file(default_file.c_str());
    bool worked = false;
    if(file.is_open())
    {
       worked = LoadModelFromFile(file);
    }
    else
    {
        // File was not opened correctly. Send out a warning/error message.
        debug << "Kinematics::Kinematics(). WARNING: Unable to load file: " << default_file << endl;
        errorlog << "Kinematics::Kinematics(). WARNING: Unable to load file: " << default_file << endl;
    }
    file.close();
    return worked;
}

std::string trim(std::string const& source, char const* delims = " \t\r\n")
{
  std::string result(source);
  std::string::size_type index = result.find_last_not_of(delims);
  if(index != std::string::npos)
    result.erase(++index);

  index = result.find_first_not_of(delims);
  if(index != std::string::npos)
    result.erase(0, index);
  else
    result.erase();
  return result;
}

bool Kinematics::LoadModelFromFile(std::ifstream& file)
{
    // File Format should be of the form:
    // [Effector]
    // Transpose x, y, z
    // Rot axis, value
    // Joint Number, Joint Name, alpha, a, thetaOffset, d
    // Joint Number, Joint Name, alpha, a, thetaOffset, d
    // [Effector]
    // Joint Number, Joint Name, alpha, a, thetaOffset, d
    // etc..

    m_endEffectors.clear();
    std::string line, effector;
    std::vector<Link> links;
    Matrix startTrans(4,4,true), endTrans(4,4,true);
    Matrix temp;
    bool startTrans_done = false;

    while(std::getline(file, line))
    {
//        std::cout << "Line: " << line << std::endl;
        if(!line.length()) continue;    // Blank Line
        if(line[0] == '#') continue;    // Comment

        if(line[0] == 'T' or line[0] == 't')    // Translation
        {
            temp = TranslationFromText(line);
            if(!startTrans_done)
            {
                startTrans = startTrans * temp;
                continue;
            }
            else
            {
                endTrans = endTrans * temp;
                continue;
            }
        }
        else if(line[0] == 'R' or line[0] == 'r')
        {
            temp = RotationFromText(line);
            if(!startTrans_done)
            {
                startTrans = startTrans * temp;
                continue;
            }
            else
            {
                endTrans = endTrans * temp;
                continue;
            }
        }
        else
        {
           startTrans_done = true;
        }

        // New effector
        if (line[0] == '[')
        {
            if(effector.size())
            {
                // Save current effector
                m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, effector));
            }


            // Add new effector
            effector = trim(line.substr(1,line.find(']')-1));
            startTrans_done = false;
            startTrans = Matrix(4,4,true);
            endTrans = startTrans;
            links.clear();
            continue;
        }

        Link temp_link = LinkFromText(line);
        links.push_back(temp_link);
    }
    // Do the last effector, if there were any.
    if(effector.size())
    {
        m_endEffectors.push_back(EndEffector(startTrans, links, endTrans, effector));
        return true;
    }
    return false;
}

Matrix Kinematics::TranslationFromText(const std::string& text)
{
    const std::string c_prefix = "Translation";
    std::string value, temp;
    std::stringstream temp_stream;
    float x=0, y=0, z=0;

    // crop off prefix
    int index = text.find(c_prefix);
    temp = text.substr(index + c_prefix.size());

    temp_stream.str(temp);

    // Get x value.
    std::getline(temp_stream,value,',');
    x = atof(trim(value).c_str());

    // Get y value.
    std::getline(temp_stream,value,',');
    y = atof(trim(value).c_str());

    // Get Z value.
    std::getline(temp_stream,value,',');
    z = atof(trim(value).c_str());

    return Translation(x,y,z);
}

Matrix Kinematics::RotationFromText(const std::string& text)
{
    const std::string c_prefix = "Rotation";
    std::string value, temp;
    std::stringstream temp_stream;
    char axis;
    float rot_value;
    Matrix result;

    // crop off prefix
    int index = text.find(c_prefix);
    temp = text.substr(index + c_prefix.size());
    temp_stream.str(temp);

    // Get axis value.
    std::getline(temp_stream,value,',');
    axis = trim(value)[0];

    // Get rotation value.
    std::getline(temp_stream,value,',');
    rot_value = atof(trim(value).c_str());

    switch(axis)
    {
    case 'x':
        result = RotX(rot_value);
        break;
    case 'y':
        result = RotY(rot_value);
        break;
    case 'z':
        result = RotZ(rot_value);
        break;
    default:
        result = Translation(0,0,0);
        break;
    }
    return result;
}

Link Kinematics::LinkFromText(const std::string& text)
{
    std::string jointName, value;
    std::stringstream temp_stream;
    DHParameters tempParam;
    int joint_num;

    temp_stream.str(text);

    // Get joint number.
    std::getline(temp_stream,value,',');
    joint_num = atoi(trim(value).c_str());

    // Get the joint name.
    std::getline(temp_stream,value,',');
    jointName = trim(value);

    // Get alpha value.
    std::getline(temp_stream,value,',');
    tempParam.alpha = atof(trim(value).c_str());

    // Get a value.
    std::getline(temp_stream,value,',');
    tempParam.a = atof(trim(value).c_str());

    // Get theta offset.
    std::getline(temp_stream,value,',');
    tempParam.thetaOffset = atof(trim(value).c_str());

    // Get d value.
    std::getline(temp_stream,value,',');
    tempParam.d = atof(trim(value).c_str());

    return Link(tempParam, jointName);
}

void Kinematics::UpdateEffector(unsigned int index, const std::vector<float>& jointValues)
{
    m_endEffectors[index].UpdateModel(jointValues);
}

Matrix Kinematics::EndEffectorPosition(unsigned int index) const
{
    return m_endEffectors[index].EndPosition();
}

Matrix Kinematics::CalculateTransform(unsigned int index, const std::vector<float>& jointValues)
{
    return m_endEffectors[index].CalculateTransform(jointValues);
}

//Vector3<float> Kinematics::calculateCentreOfMass()
//{
//    Vector3<float> com_position; // Position of CoM in 3D space (x,y,z)
//    for (RobotModel::iterator effector = m_endEffectors.begin(); effector != m_endEffectors.end(); ++effector)
//    {
//        // do something.
//    }
//    return com_position;
//}

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

//std::vector<float> Kinematics::LookToPoint(const std::vector<float>& pointFieldCoordinates)
//{
//    std::vector<float> result;
//    return result;
//}

double Kinematics::CalculateRelativeFootHeight(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix)
{
    Matrix totalTransform = InverseMatrix(supportFootTransformMatrix) * theFootTransformMatrix;

    // Use centre of the foot
    Matrix footCentre(4, 1, false);
    footCentre[3][0] = 1.0;

    Matrix result = totalTransform * footCentre;
    return result[2][0];
}

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
