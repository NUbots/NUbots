#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "Vision/basicvisiontypes.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Math/Vector3.h"
#include <vector>

using std::vector;

class Transformer
{
    friend class VisionBlackboard;
public:
    Transformer();

    //2D distortion transform
    Point correctDistortion(const Point& pt);

    //2D - 2D pixel to camera relative polar transforms
    Point screenToRadial2D(Point pt) const;
    vector<Point> screenToRadial2D(const vector<Point>& pts) const;

    //Distance to point - returns the distance of the camera from a point on the
    //                    ground which would be visible at the given screen location.
    bool isDistanceToPointValid() const;
    double distanceToPoint(Point pixel_loc) const;
    double distanceToPoint(double bearing, double elevation) const;
    //double distanceToPoint(double bearing, double elevation) const;

    //2D pixel - 3D polar (feet relative) - assumes point is on the ground
    bool isScreenToGroundValid() const;
//    Vector3<double> screenToGroundRadial(Point pt) const;
//    vector<Vector3<double> > screenToGroundRadial(const vector<Point>& pts) const;

    //2D pixel - 2D cartesian (feet relative) - assumes point is on the ground
    Point screenToGroundCartesian(Point pt) const;
    vector<Point> screenToGroundCartesian(const vector<Point>& pts) const;

    double getCameraDistanceInPixels() const { return effective_camera_dist_pixels; }

    Vector2<double> getFOV() const {return FOV;}

private:
    //! Calculate the field of view and effective camera distance in pixels.
    void setKinematicParams(bool cam_pitch_valid, double cam_pitch,
                            bool cam_height_valid, double cam_height,
                            bool b_pitch_valid, double b_pitch,
                            bool ctg_valid, vector<float> ctg_vector);
    void setCamParams(Vector2<double> imagesize,
                      Vector2<double> fov);

private:
    Vector2<double> FOV;
    double effective_camera_dist_pixels;

    vector<float> m_ctg_vector;    //! @variable The camera to ground vector (for d2p).
    bool m_ctg_valid;              //! @variable Whether the ctgvector is valid.
//    vector<float> ctvector;     //! @variable The camera transform vector.
//    bool ctvalid;               //! @variable Whether the ctvector is valid.

    Point image_size;
    Point image_centre;
    Vector2<double> tan_half_FOV;
    Vector2<double> screen_to_radial_factor;
    double camera_pitch;         //! @variable The camera pitch angle.
    bool camera_pitch_valid;    //! @variable Whether the camera pitch is valid.
    double camera_height;        //! @variable The height of the camera from the ground.
    bool camera_height_valid;   //! @variable Whether the camera height is valid.
    double body_pitch;           //! @variable The body pitch angle.
    bool body_pitch_valid;      //! @variable Whether the body pitch is valid.
};

#endif // TRANSFORMER_H
