/*!
    @file Horizon.h
    @author Steven Nicklin
    @brief Decleration of Horizon class.
  */

#ifndef Horizon_H_DEFINED
#define Horizon_H_DEFINED
#include "Tools/Math/Line.h"

/*!
    @brief Class used to compute and store the horizon line.

    The horizon line is defined as the plane at which the infinite horizon would appear
    in the current image. This is represented by a plane parallel to the ground set
    at the height of the camera. The horizon line shows the line produced by the intersection
    of this plane through the image.
  */
class Horizon : public Line{
  public:
    //! True if the Horizon object contains information about a horizon line. False if it does not.
    bool exists;
    Horizon(); //!< Standard Constructor.
    ~Horizon(); //!< Destructor.
    void Reset();   //!< Resets the horizon information. This then indicates that the line is no longer valid.
    /*!
      @brief Calculates the new values for the horizon line.
      @param bodyPitch The angle of the robots body about the Y-axis.
      @param bodyRoll The angle of the robots body about the X-axis.
      @param headYaw The angle of the robots head about the Z-axis.
      @param headPitch The angle of the robots head about the Y-axis.
      @param cameraNumber The camera number for which we are claculating the horizon for.
      @todo Change the cameraNumber parameter to a camera information object, containg the fov/resolution
            data and the transform from the base of the neck for the current camera.
      */
    void Calculate(double bodyPitch, double bodyRoll, double headYaw, double headPitch, int cameraNumber);
    
    /*!
      @brief Determine if the given pixel is below the current horizon line.
      @param x The x coordinate of the pixel.
      @param y The y coordinate of the pixel.
      @return True if The pixel lies below the horizon line. False if it lies on or above the horizon line.
      */
    bool IsBelowHorizon(int x, int y) const;
};

#endif
