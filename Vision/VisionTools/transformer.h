#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/groundpoint.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include "Tools/Math/Matrix.h"
#include <vector>
#include "Infrastructure/SensorCalibration.h"

using std::vector;

class Transformer
{
    friend class VisionBlackboard;
    friend class DataWrapper;
    friend class SensorCalibrationWidget;
public:
    Transformer();
    Transformer(const SensorCalibration& calibration)
    {
        setCalibration(calibration);
    }

    void setCalibration(const SensorCalibration& calibration)
    {
        sensor_calibration = calibration;
        preCalculateTransforms();
    }
    SensorCalibration calibration(){return sensor_calibration;}

    void preCalculateTransforms();

    //2D distortion transform
    Vector2<double> correctDistortion(const Vector2<double>& pt);

    //2D - 2D pixel to camera relative polar transforms
    void screenToRadial2D(GroundPoint& pt) const;
    void screenToRadial2D(vector<GroundPoint>& pts) const;
    GroundPoint screenToRadial2D(const Point& pt) const;
    vector<GroundPoint> screenToRadial2D(const vector<Point>& pts) const;


    void radial2DToRadial3D(GroundPoint &pt, double distance) const;
    void screenToRadial3D(GroundPoint &pt, double distance) const;
    GroundPoint screenToRadial3D(const Point &pt, double distance) const;

    //Distance to point - returns the distance of the camera from a point on the
    //                    ground which would be visible at the given screen location.
    bool isDistanceToPointValid() const;
    double distanceToPoint(double bearing, double elevation) const;
    double distanceToPoint(const GroundPoint& gp) const;
    //double distanceToPoint(double bearing, double elevation) const;

    /*!
    * @brief Distance to point calculation. Calculates the distance, bearing and elevation of an point based
    * on the position in the pixel in the image and the camera position.
    * @param x pixel x-position.
    * @param y pixel y-position.
    * @param object_height The height of the object or point at the pixel we are measuring. (default = 0.0) ie ground level.
    * @return The 3d vector containing the distance, bearing and elevation to the point.
    */
    Vector3<double> distanceToPoint(unsigned int x_pixel, unsigned int y_pixel, double object_height=0.0);

    bool isScreenToGroundValid() const;

    //2D pixel - 2D cartesian (feet relative) - assumes point is on the ground
    void screenToGroundCartesian(GroundPoint& pt) const;
    void screenToGroundCartesian(vector<GroundPoint>& pts) const;
    GroundPoint screenToGroundCartesian(const Point& pt) const;
    vector<GroundPoint> screenToGroundCartesian(const vector<Point>& pts) const;

    double getCameraDistanceInPixels() const { return effective_camera_dist_pixels; }

    Vector2<double> getFOV() const {return FOV;}

private:
    //! Calculate the field of view and effective camera distance in pixels.
    void setKinematicParams(bool cam_pitch_valid, double cam_pitch,
                            bool cam_yaw_valid, double cam_yaw,
                            bool cam_height_valid, double cam_height,
                            bool b_pitch_valid, double b_pitch,
                            bool ctg_valid, vector<float> ctg_vector);
    void setCamParams(Vector2<double> imagesize,
                      Vector2<double> fov);

    void setSensors(double new_head_pitch, double new_head_yaw, double new_body_roll, double new_body_pitch, Vector3<double> new_neck_position);

private:
    Vector2<double> FOV;
    double effective_camera_dist_pixels;

    Matrix ctgtransform;
    bool m_ctg_valid;              //! @variable Whether the ctgvector is valid.
//    vector<float> ctvector;     //! @variable The camera transform vector.
//    bool ctvalid;               //! @variable Whether the ctvector is valid.

    Vector2<double> image_size;
    Vector2<double> image_centre;
    Vector2<double> tan_half_FOV;
    Vector2<double> screen_to_radial_factor;
    double camera_pitch;         //! @variable The camera pitch angle.
    bool camera_pitch_valid;    //! @variable Whether the camera pitch is valid.
    double camera_yaw;         //! @variable The camera yaw angle.
    bool camera_yaw_valid;    //! @variable Whether the camera yaw is valid.
    double camera_height;        //! @variable The height of the camera from the ground.
    bool camera_height_valid;   //! @variable Whether the camera height is valid.
    double body_pitch;           //! @variable The body pitch angle.
    bool body_pitch_valid;      //! @variable Whether the body pitch is valid.

    // New for transforms.
    SensorCalibration sensor_calibration;
    Matrix camVector;
    Matrix camV2RobotRotation;
    double head_pitch;
    double head_yaw;
    double body_roll;
    Vector3<double> neck_position;
};

#endif // TRANSFORMER_H
