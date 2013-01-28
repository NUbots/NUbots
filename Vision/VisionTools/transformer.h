#ifndef TRANSFORMER_H
#define TRANSFORMER_H

class Transformer
{
public:
    Transformer();

    //2D distortion transform
    Vector2<float> correctDistortion(const Vector2<float>& pt);

    // 3D Basis transforms - Polar

    //2D - 3D screen to polar transforms
    Point screenToRadial2D(Point pt) const;
    vector<Point> screenToRadial2D(const vector<Point>& pts) const;

    //Distance to point - returns the distance of the camera from a point on the
    //                    ground which would be visible at the given screen location.
    bool isDistanceToPointValid() const;
    float distanceToPoint(float bearing, float elevation) const;

    //2D pixel - 3D polar (feet relative) - assumes point is on the ground
    bool isScreenToGroundValid() const;
    Vector3<float> screenToGround(Point pt) const;
    vector<Vector3<float> > screenToGround(const vector<Point>& pts) const;

    //2D pixel - 3D cartesian (feet relative) - assumes point is on the ground
    Point transformToGround(Point pt) const;
    vector<Point> transformToGround(const vector<Point>& pts) const;

private:
    Vector2<float> FOV;
    float effective_camera_dist_pixels;

    vector<float> ctgvector;    //! @variable The camera to ground vector (for d2p).
    bool ctgvalid;              //! @variable Whether the ctgvector is valid.
    vector<float> ctvector;     //! @variable The camera transform vector.
    bool ctvalid;               //! @variable Whether the ctvector is valid.

    float camera_pitch;         //! @variable The camera pitch angle.
    bool camera_pitch_valid;    //! @variable Whether the camera pitch is valid.
    float camera_height;        //! @variable The height of the camera from the ground.
    bool camera_height_valid;   //! @variable Whether the camera height is valid.
    float body_pitch;           //! @variable The body pitch angle.
    bool body_pitch_valid;      //! @variable Whether the body pitch is valid.
};

#endif // TRANSFORMER_H
