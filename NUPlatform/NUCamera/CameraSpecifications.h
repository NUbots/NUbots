/*!
  @file CameraSpecifications.h
  @brief Decleration of CameraSpecifications class
  @author Steven Nicklin
  */

#ifndef CAMERASPECIFICATIONS_H
#define CAMERASPECIFICATIONS_H

/*!
  @brief Class used to store specifications of the camera.
  */
class CameraSpecifications
{
public:
    /*!
      @brief default Constructor.
      */
    CameraSpecifications();
    /*!
      @brief Constructor to load from specified file.
      */
    CameraSpecifications(const char* fileName);
    /*!
      @brief Copy constructor.
      */
    CameraSpecifications(const CameraSpecifications& sourceSpec);
    bool LoadFromConfigFile(const char* fileName);

    int resolutionWidth;
    int resolutionHeight;
    double horizontalFov;
    double verticalFov;
    double focalLength;
};

#endif // CAMERASPECIFICATIONS_H
