#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "Vision/basicvisiontypes.h"
#include "Vision/VisionTypes/nupoint.h"
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

    SensorCalibration getCalibration() {return sensor_calibration;}

    //2D distortion transform
    Vector2<double> correctDistortion(const Vector2<double>& pt);

    void calculateRepresentations(NUPoint& pt, bool known_distance = false, double val = 0.0) const;
    void calculateRepresentations(vector<NUPoint>& pts, bool known_distance = false, double val = 0.0) const;
//    NUPoint calculateRepresentations(const Point& pt, bool ground = true, double val = 0.0) const;
//    vector<NUPoint> calculateRepresentations(const vector<Point>& pts, bool ground = true, double val = 0.0) const;

    double getCameraDistanceInPixels() const { return effective_camera_dist_pixels; }

    Vector2<double> getFOV() const {return FOV;}

private:
    //! Calculate the field of view and effective camera distance in pixels.
    void setCamParams(Vector2<double> imagesize, Vector2<double> fov);

    void setSensors(double new_head_pitch, double new_head_yaw, double new_body_roll, double new_body_pitch, Vector3<double> new_neck_position);

    void preCalculateTransforms();

    void screenToRadial3D(NUPoint &pt, double distance) const;
    NUPoint screenToRadial3D(const Point &pt, double distance) const;

    //2D pixel - 2D cartesian (feet relative) - assumes point is on the ground
    void screenToGroundCartesian(NUPoint& pt) const;
    void screenToGroundCartesian(vector<NUPoint>& pts) const;
    NUPoint screenToGroundCartesian(const Point& pt) const;
    vector<NUPoint> screenToGroundCartesian(const vector<Point>& pts) const;

    /**
      * Calculates the distance to a point at a given height
      * @param pixel The pixel location in the image relative to the top left of the screen.
      * @param object_height The height of the point to be measured (from the ground).
      * @return A 3 dimensional vector containing the distance, bearing and elevation to the point.
      */
    Vector3<double> distanceToPoint(Vector2<double> pixel, double object_height=0.0);

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

    // New for transforms.
    SensorCalibration sensor_calibration;
    Matrix camVector;
    Matrix camV2RobotRotation;
    double head_pitch;
    double head_yaw;
    double body_roll;
    double body_pitch;
    Vector3<double> neck_position;
};

#endif // TRANSFORMER_H
