#ifndef H_KINEMATICS_H_DEFINED
#define H_KINEMATICS_H_DEFINED
#include <vector>
#include <string>
#include "EndEffector.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Rectangle.h"
#include "debug.h"
#include "Tools/Math/General.h"

class Kinematics
{
public:
    typedef std::vector<EndEffector> RobotModel;

    /*!
        @brief Loads a localisation model from the default configuration file
        @return true if the model was loaded sucessfully, false if it was not.
      */
    bool LoadModel();

    /*!
    * @brief Loads the kinematic model from the file given.
    *
    * File Format should be of the form:
    * [Effector]
    * Transpose x, y, z
    * Rot axis, value
    * Joint Number, Joint Name, alpha, a, thetaOffset, d
    * Joint Number, Joint Name, alpha, a, thetaOffset, d
    * [Effector]
    * Joint Number, Joint Name, alpha, a, thetaOffset, d
    * Trans x, y, z
    * Rot axis, value
    * [Effector]
    * etc.. with names and values in place of the variable names.
    *
    * @param file Stream containing the localisation model.
    * @return true if the model was loaded sucessfully, false if it was not.
    */
    bool LoadModelFromFile(std::ifstream& file);

    /*!
    * @brief Creates a Translation matrix from the a text description.
    *
    * The text should describe the translation operation as follows:
    * Transpose x, y, z
    * where x, y, z are the translations along the x, y, and z-axes respectively.
    *
    * @param text The text containing the description of the translation matrix to create.
    * @return Transform matrix containing the translation.
    */
    static Matrix TranslationFromText(const std::string& text);

    /*!
    * @brief Creates a Rotation matrix from the a text description.
    *
    * The text should describe the rotation operation as follows:
    * Rot axis, value
    * where axis is the character 'x', 'y', or 'z' for their respective axis, and value is the magnitude of the rotation in radians.
    *
    * @param text The text containing the description of the rotation matrix to create.
    * @return Transform matrix containing the rotation.
    */
    static Matrix RotationFromText(const std::string& text);

    /*!
    * @brief Creates a Link using the modified D-H notation, from the a text description.
    *
    * The text should describe the link as follows:
    * Joint Number, Joint Name, alpha, a, thetaOffset, d
    * where Joint Number is the joints position in the chain, Joint Name is the name of the joint,
    * alpha is the alpha rotation in the modified D-H notation in radians, a is the a offset distance,
    * thetaOffset is the offset rotation in radians, d is the d offset distance.
    *
    * @param text The text containing the description of the link to create.
    * @return Link created to the specifications in the text.
    */
    static Link LinkFromText(const std::string& text);

    void UpdateEffector(unsigned int index, const std::vector<float>& jointValues);
    Matrix EndEffectorPosition(unsigned int index) const;
    Vector3<float> calculateCentreOfMass();


    /*!
    * @brief Calculates the transform to the end of an effector.
    *
    * The joint angles given are applied to each of the links in the kinematic chain leading to the effector.
    *
    * @param index The inde of the effector to calculate the transform of.
    * @param jointValues The values of each of the joints in the chain for the effector.
    * @return Transform matrix converting from the origin space to the effector space.
    */
    Matrix CalculateTransform(unsigned int index, const std::vector<float>& jointValues);

    /*!
    * @brief Calculates the transform from the camera space to the world space at the ground plane.
    *
    * Combines bot the leg and camera transform to get a total transform between the camera and world space.
    *
    * @param origin2SupportLegTransform The transform from the support foot to the camera space.
    * @param origin2Camera The transform from the origin to the camera space
    * @return Transform matrix from the camera to the ground space.
    */
    static Matrix CalculateCamera2GroundTransform(const Matrix& origin2SupportLegTransform, const Matrix& origin2Camera);

    /*!
    * @brief Calculates the position of an object in sperical coordinates centred at the robot origin, given the position in an image.
    *
    * The distance to point calculation takes a position in the camera image and projects this point until it intercepts the ground plane.
    * The sperical position in terms of distance, heading and elevation are calculated for this intercept point.
    *
    * @param Camera2GroundTransform The transform matrix from the camera space to the ground space.
    * @param angleFromCameraCentreX The angle of the point from the centre of the image along the images horizontal x-axis.
    * @param angleFromCameraCentreY The angle of the point from the centre of the image along the images vertical y-axis.
    * @return A vector containing the three elements, distance, heading and elevation to the point.
    */
    static Vector3<float> DistanceToPoint(const Matrix& Camera2GroundTransform, double angleFromCameraCentreX, double angleFromCameraCentreY);

    /*!
    * @brief Converts a position from the camera space to the origin space. The position is described in spherical coordinates.
    *
    * The distance to point calculation takes a position in the camera image and projects this point until it intercepts the ground plane.
    * The sperical position in terms of distance, heading and elevation are calculated for this intercept point.
    *
    * @param Camera2GroundTransform The transform matrix from the camera space to the ground space.
    * @param cameraBasedPosition A vector containing the position to be converted in sperical coordinates.
    * @return A vector containing the three elements, distance, heading and elevation to the point. Now in origin coordinates.
    */
    static std::vector<float> TransformPosition(const Matrix& Camera2GroundTransform, const std::vector<float>& cameraBasedPosition);

    /*!
    * @brief Converts a position from the camera space to the origin space. The position is described in spherical coordinates.
    *
    * The distance to point calculation takes a position in the camera image and projects this point until it intercepts the ground plane.
    * The sperical position in terms of distance, heading and elevation are calculated for this intercept point.
    *
    * @param Camera2GroundTransform The transform matrix from the camera space to the ground space.
    * @param cameraBasedPosition A Vector3 containing the position to be converted in sperical coordinates.
    * @return A Vector3 containing the three elements, distance, heading and elevation to the point. Now in origin coordinates.
    */
    static Vector3<float> TransformPosition(const Matrix& Camera2GroundTransform, const Vector3<float>& cameraBasedPosition);

    /*!
    * @brief Converts a position from the camera space to the origin space. The position is described in spherical coordinates.
    *
    * The distance to point calculation takes a position in the camera image and projects this point until it intercepts the ground plane.
    * The sperical position in terms of distance, heading and elevation are calculated for this intercept point.
    *
    * @param Camera2GroundTransform The transform matrix from the camera space to the ground space.
    * @param cameraBasedPosition A collumn Matrix containing the position to be converted in sperical coordinates.
    * @return A collumn Matrix containing the three elements, distance, heading and elevation to the point. Now in origin coordinates.
    */
    static Matrix TransformPosition(const Matrix& Camera2GroundTransform, const Matrix& cameraBasedPosition);

    /*!
    * @brief Calculate the required head position to look ar a point on the field. NOT YET IMPLEMENTED!
    *
    * @param pointFieldCoordinates The 2d position in field coordinates.
    * @return The joint angles required to look at the given position.
    */
//    static std::vector<float> LookToPoint(const std::vector<float>& pointFieldCoordinates);

    /*!
    * @brief Calculate position in relative 3D space from a 4x4 transform matrix.
    *
    * @param transformMatrix The transform matrix from which to extract a position.
    * @return The 3D position in terms of x, y, and z.
    */
    static std::vector<float> PositionFromTransform(const Matrix& transformMatrix)
    {
        std::vector<float> result(3,0.0f);
        result[0] = transformMatrix[0][3];
        result[1] = transformMatrix[1][3];
        result[2] = transformMatrix[2][3];
        return result;
    }

    /*!
    * @brief Calculate orientation in relative 3D space from a 4x4 transform matrix.
    *
    * @param transformMatrix The transform matrix from which to extract an orientation.
    * @return The 3D orientation in terms of x rotation, y rotation, and z rotaion.
    */
    static std::vector<float> OrientationFromTransform(const Matrix& transformMatrix)
    {
		// Derived from matrix formed by RotZ(psi)*RotY(theta)*RotX(Phi)
        std::vector<float> result(3,0.0f);
        result[1] = asin(transformMatrix[2][0]);
        result[0] = -atan2(transformMatrix[2][1], transformMatrix[2][2]);
        result[2] = atan2(transformMatrix[1][0], transformMatrix[0][0]);
		if(transformMatrix[2][2] * cos(result[1]) < 0)
		{
			result[0] = mathGeneral::normaliseAngle(result[0] + mathGeneral::PI);
		}

		if(transformMatrix[0][0] * cos(result[1]) < 0)
		{
			result[2] = mathGeneral::normaliseAngle(result[2] + mathGeneral::PI);
		}

        return result;
    }

    /*!
    * @brief Calculate the difference in height between the two feet.
    *
    * Calculates the distance from the support foot to the other foot. Above the support foot gives a positive value,
    * below the support foot gives a negative value.
    *
    * @param supportFootTransformMatrix The transform matrix of the support foot.
    * @param theFootTransformMatrix The transform matrix of the other foot.
    * @return The distance between the two feet.
    */
    static double CalculateRelativeFootHeight(const Matrix& supportFootTransformMatrix,const Matrix& theFootTransformMatrix);

    /*!
    * @brief Converts a relative 2D field position into a position relative to a particular foot.
    *
    *
    * @param FootTransformMatrix The transform matrix of the foot in question.
    * @param position The 2D realtive position centred on the robot.
    * @return The 2D relative position transformed to the foot space.
    */
    static Vector2<float> TransformPositionToFoot(const Matrix& FootTransformMatrix, Vector2<float> position);

    /*!
    * @brief Retrieves a pointer to the robot model.
    *
    * @return A pointer to the current robot model.
    */
    const RobotModel* getModel() const {return &m_endEffectors;}

protected:
    RobotModel m_endEffectors;  //!< Robot model made up of a list of end effectors.
};

#endif

