#ifndef TRANSFORMER_H
#define TRANSFORMER_H

class Transformer
{
public:
    Transformer();

    // 3D Basis transforms - Polar
    bool isCamToGroundValid() const { return ctgvalid; }
    Vector3<float> camToGround(const Vector3<float>& cam_relative) const;
    bool isCamTransformValid() const { return ctvalid; }
    Vector3<float> camTransform(const Vector3<float>& cam_relative) const;

    //2D distortion transform
    Vector2<float> correctDistortion(const Vector2<float>& pt);

    //2D - 3D screen to polar transforms
    float calculateBearing(float x) const;
    float calculateElevation(float y) const;
    Point screenToRadial2D(Point pt) const;
    vector<Point> screenToRadial2D(const vector<Point>& pts) const;

    //2D - 3D screen to polar - assumes point is on the ground
    bool isScreenToGroundValid() const;
    Vector3<float> screenToGround(Point pt) const;
    vector<Vector3<float> > screenToGround(const vector<Point>& pts) const;
    bool isDistanceToPointValid() const;
    float distanceToPoint(float bearing, float elevation) const;

    //2D - 3D screen to cartesian + basis transforms
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
