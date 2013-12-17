#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <nuclear>
#include <armadillo>
#include <vector>

#include "utility/math/matrix.h"
#include "utility/math/coordinates.h"

#include "NUPoint.h"

class Transformer {
public:
    Transformer();
    Transformer(const SensorCalibration& calibration);

    void setCalibration(const SensorCalibration& calibration);

    SensorCalibration getCalibration();

    // 2D distortion transform.
    arma::vec2 correctDistortion(const arma::vec2& point);

    void calculateRepresentationsFromPixelLocation(NUPoint& pt, bool knownDistance = false, double val = 0.0) const;
    void calculateRepresentationsFromPixelLocation(std::vector<NUPoint>& pts, bool knownDistance = false, double val = 0.0) const;
    void calculateRepresentationsFromGroundCartesianLocation(NUPoint& pt) const;
    void calculateRepresentationsFromGroundCartesianLocation(std::vector<NUPoint>& pts) const;

//    NUPoint calculateRepresentations(const Point& pt, bool ground = true, double val = 0.0) const;
//    std::vector<NUPoint> calculateRepresentations(const std::vector<Point>& pts, bool ground = true, double val = 0.0) const;

    double getCameraDistanceInPixels() const;

    arma::vec2& getFOV() const;

private:
    //! Calculate the field of view and effective camera distance in pixels.
    void setCamParams(arma::vec2 imagesize, arma::vec2 fov);

    void setSensors(double new_head_pitch, double new_head_yaw, double new_body_roll, double new_body_pitch, arma::vec3 new_neck_position);

    void preCalculateTransforms();

    void screenToRadial3D(NUPoint &pt, double distance) const;
    NUPoint screenToRadial3D(const Point &pt, double distance) const;

    //2D pixel - 2D cartesian (feet relative) - assumes point is on the ground
//    void screenToGroundCartesian(NUPoint& pt) const;
//    void screenToGroundCartesian(vector<NUPoint>& pts) const;
//    NUPoint screenToGroundCartesian(const Point& pt) const;
//    vector<NUPoint> screenToGroundCartesian(const vector<Point>& pts) const;

    /**
      * Calculates the distance to a point at a given height
      * @param pixel The pixel location in the image relative to the top left of the screen.
      * @param object_height The height of the point to be measured (from the ground).
      * @return A 3 dimensional vector containing the distance, bearing and elevation to the point.
      */
    arma::vec3 distanceToPoint(arma::vec2 pixel, double objectHeight = 0.0) const;

private:
    arma::vec2 m_FOV;
    double m_effectiveCameraDistancePixels;

    arma::mat m_ctgTransform;
    bool m_ctgValid;              //! @variable Whether the ctgvector is valid.
//    std::vector<float> m_ctVector;     //! @variable The camera transform vector.
//    bool m_ctValid;               //! @variable Whether the ctvector is valid.

    arma::vec2 m_imageSize;
    arma::vec2 m_imageCentre;
    arma::vec2 m_tanHalfFOV;
    arma::vec2 m_screenToRadialFactor;

    // New for transforms.
    SensorCalibration m_sensorCalibration;
    Matrix m_camVector;
    Matrix m_camV2RobotRotation;
    double m_headPitch;
    double m_headYaw;
    double m_bodyRoll;
    double m_bodyPitch;
    arma::vec3 m_neckPosition;
};

#endif // TRANSFORMER_H
