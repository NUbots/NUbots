#include "transformer.h"

Transformer::Transformer()
{
}

// 3D Basis transforms - Polar
//Vector3<float> Transformer::camToGround(const Vector3<float>& cam_relative) const
//{

//}

//Vector3<float> Transformer::camTransform(const Vector3<float>& cam_relative) const
//{
//    m_transformed_spherical_pos = Kinematics::TransformPosition(Matrix4x4fromVector(m_ctvector), cam_relative);
//}

//2D distortion transform
Vector2<float> Transformer::correctDistortion(const Vector2<float>& pt);

//2D - 3D screen to polar transforms
float Transformer::calculateBearing(float x) const;
float Transformer::calculateElevation(float y) const;
Point Transformer::screenToRadial2D(Point pt) const;
vector<Point> Transformer::screenToRadial2D(const vector<Point>& pts) const;

bool Transformer::isScreenToGroundValid() const
{
    return isDistanceToPointValid();
}
Vector3<float> screenToGround(Point pt) const;
vector<Vector3<float> > screenToGround(const vector<Point>& pts) const;

bool Transformer::isDistanceToPointValid() const
{
    return camera_height_valid && camera_pitch_valid && (not VisionConstants::D2P_INCLUDE_BODY_PITCH || body_pitch_valid);
}

float distanceToPoint(float bearing, float elevation) const;

//2D - 3D screen to cartesian + basis transforms
Point transformToGround(Point pt) const;
vector<Point> transformToGround(const vector<Point>& pts) const;
