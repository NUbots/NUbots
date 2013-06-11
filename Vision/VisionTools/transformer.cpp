#include "transformer.h"
#include <boost/foreach.hpp>
#include "Vision/visionconstants.h"
#include "Tools/Math/General.h"
#include "Kinematics/Kinematics.h"
#include "debug.h"
#include "debugverbosityvision.h"

Transformer::Transformer()
{
    FOV = Vector2<double>(0,0);
    effective_camera_dist_pixels = 0;

    m_ctg_valid = camera_pitch_valid = camera_height_valid = body_pitch_valid = false;

    image_size = Vector2<double>(0,0);
    image_centre = Vector2<double>(0,0);
    tan_half_FOV = Vector2<double>(0,0);
    screen_to_radial_factor = Vector2<double>(0,0);
}

/**
  * Applies radial distortion correction to the given pixel location.
  * @param pt The pixel location to correct.
  */
Vector2<double> Transformer::correctDistortion(const Vector2<double>& pt)
{
    //get position relative to centre
    Vector2<double> half_size = image_size*0.5;
    Vector2<double> centre_relative = pt - half_size;
    //calculate correction factor -> 1+kr^2
    double corr_factor = 1 + VisionConstants::RADIAL_CORRECTION_COEFFICIENT*centre_relative.squareAbs();
    //multiply by factor
    Vector2<double> result = centre_relative*corr_factor;
    //scale the edges back out to meet again
    result.x /= (1+VisionConstants::RADIAL_CORRECTION_COEFFICIENT*half_size.x*half_size.x);
    result.y /= (1+VisionConstants::RADIAL_CORRECTION_COEFFICIENT*half_size.y*half_size.y);
    //get the original position back from the centre relative position
    return result + half_size;
}

/**
  * Calculates the radial position (relative to the camera vector) from the pixel position.
  * @param pt The pixel location.
  */
void Transformer::screenToRadial2D(GroundPoint& pt) const
{    

    pt.angular.x = atan( (image_centre.x-pt.screen.x)  * screen_to_radial_factor.x);
    pt.angular.y = atan( (image_centre.y-pt.screen.y) * screen_to_radial_factor.y) + VisionConstants::D2P_ANGLE_CORRECTION;

    /*if(camera_yaw_valid)
        pt.angular.x += camera_yaw;
    if(camera_pitch_valid)
        pt.angular.y -= camera_pitch;

    if(VisionConstants::D2P_INCLUDE_BODY_PITCH && body_pitch_valid)
        pt.angular.y -= body_pitch;
        */
}

/**
  * Calculates the radial position (relative to the camera vector) from the pixel position for a set of pixels.
  * @param pts The list of pixels.
  */
void Transformer::screenToRadial2D(vector<GroundPoint>& pts) const
{
    BOOST_FOREACH(GroundPoint& p, pts) {
        screenToRadial2D(p);
    }
}

/**
  * Calculates the radial position (relative to the camera vector) from the pixel position.
  * @param pt The pixel location.
  */
GroundPoint Transformer::screenToRadial2D(const Point& pt) const
{
    GroundPoint g;
    g.screen = pt;
    screenToRadial2D(g);
    return g;
}

/**
  * Calculates the radial position (relative to the camera vector) from the pixel position for a set of pixels.
  * @param pts The list of pixels.
  */
vector<GroundPoint> Transformer::screenToRadial2D(const vector<Point>& pts) const
{
    vector<GroundPoint> groundPts;
    BOOST_FOREACH(const Point& p, pts) {
        groundPts.push_back(screenToRadial2D(p));
    }
    return groundPts;
}

/// @note Assumes radial calculation already done
void Transformer::radial2DToRadial3D(GroundPoint &pt, double distance) const
{
    pt.relativeRadial = Kinematics::TransformPosition(ctgtransform, Vector3<double>(distance, pt.angular.x, pt.angular.y));
}

void Transformer::screenToRadial3D(GroundPoint &pt, double distance) const
{
    screenToRadial2D(pt);
    radial2DToRadial3D(pt, distance);
}

GroundPoint Transformer::screenToRadial3D(const Point &pt, double distance) const
{
    GroundPoint g = screenToRadial2D(pt);
    radial2DToRadial3D(g, distance);
    return g;
}

bool Transformer::isDistanceToPointValid() const
{
    return camera_height_valid && camera_pitch_valid && camera_yaw_valid && (not VisionConstants::D2P_INCLUDE_BODY_PITCH || body_pitch_valid);
}

/**
  * Calculates the distance to an object at a given point assuming the object is only the same
  * plane as the robots feet. This is useful as the point of contact with the ground for all field
  * objects can easily be identified visually.
  * @param bearing The horizontal radial coordinate relative to the camera vector.
  * @param elevation The vertical radial coordinate relative to the camera vector.
  * @return The distance to the given point relative to the robots feet.
  */
double Transformer::distanceToPoint(double bearing, double elevation) const
{
#if VISION_TRANSFORM_VERBOSITY > 2
    debug << "Transformer::distanceToPoint: \n";
    debug << "\t(bearing, elevation): (" << bearing << ", " << elevation << ")" << endl;
    debug << "\tbody pitch: include:" << VisionConstants::D2P_INCLUDE_BODY_PITCH << " valid: " << body_pitch_valid << " value: " << body_pitch << endl;
    debug << "\tVisionConstants::D2P_ANGLE_CORRECTION: " << VisionConstants::D2P_ANGLE_CORRECTION << endl;
    debug << "\tcamera height valid: " << camera_height_valid << " value: " << camera_height << endl;
    debug << "\tcamera pitch valid: " << camera_pitch_valid << " value: " << camera_pitch << endl;
#endif
    double theta,
           distance,
           cos_theta,
           cos_bearing;

    //resultant angle inclusive of camera pitch, pixel elevation and angle correction factor
    theta = mathGeneral::PI*0.5 + elevation;

    cos_theta = cos(theta);
    cos_bearing = cos(bearing);
    if(cos_theta == 0 || cos_bearing == 0)
        distance = 0;
    else
        distance = camera_height / cos_theta / cos_bearing;

#if VISION_TRANSFORM_VERBOSITY > 1
    debug << "\ttheta: " << theta << " distance: " << distance << endl;
#endif

    return distance;
}

//! @brief Whether the screen to ground vector calculation will give valid information.
bool Transformer::isScreenToGroundValid() const {
    return isDistanceToPointValid();
}

void Transformer::screenToGroundCartesian(GroundPoint& pt) const
{
    screenToRadial2D(pt);

    double r = distanceToPoint(pt.angular.x, pt.angular.y);

    //return Point(r*sin(cam_angles.x), r*cos(cam_angles.x));

    //Vector3<float> spherical_foot_relative = Kinematics::DistanceToPoint(ctgtransform, cam_angles.x, cam_angles.y);
    Vector3<double> spherical_foot_relative = Kinematics::TransformPosition(ctgtransform, Vector3<double>(r, pt.angular.x, pt.angular.y));

    Vector3<double> cartesian_foot_relative = mathGeneral::Spherical2Cartesian(spherical_foot_relative);

#if VISION_FIELDPOINT_VERBOSITY > 2
    debug << "Transformer::screenToGroundCartesian - the following should be near zero: " << cartesian_foot_relative.z << endl;
#endif
    pt.ground = Vector2<double>(cartesian_foot_relative.x, cartesian_foot_relative.y);
}

void Transformer::screenToGroundCartesian(vector<GroundPoint>& pts) const
{
    BOOST_FOREACH(GroundPoint& p, pts) {
        screenToGroundCartesian(p);
    }
}

GroundPoint Transformer::screenToGroundCartesian(const Point &pt) const
{
    GroundPoint g;
    g.screen = pt;
    screenToGroundCartesian(g);
    return g;
}

vector<GroundPoint> Transformer::screenToGroundCartesian(const vector<Point>& pts) const
{
    vector<GroundPoint> gpts;
    BOOST_FOREACH(const Point& p, pts) {
        gpts.push_back(screenToGroundCartesian(p));
    }
    return gpts;
}

void Transformer::setKinematicParams(bool cam_pitch_valid, double cam_pitch, bool cam_yaw_valid, double cam_yaw,
                                     bool cam_height_valid, double cam_height,
                                     bool b_pitch_valid, double b_pitch,
                                     bool ctg_valid, vector<float> ctg_vector)
{
    camera_pitch_valid = cam_pitch_valid;
    camera_pitch = cam_pitch;
    camera_yaw_valid = cam_yaw_valid;
    camera_yaw = cam_yaw;
    camera_height_valid = cam_height_valid;
    camera_height = cam_height;
    body_pitch_valid = b_pitch_valid;
    body_pitch = b_pitch;
    m_ctg_valid = ctg_valid;
    ctgtransform = Matrix4x4fromVector(ctg_vector);
}

void Transformer::setCamParams(Vector2<double> imagesize, Vector2<double> fov)
{
    image_size = imagesize;
    image_centre = imagesize*0.5;
    FOV = fov;
    tan_half_FOV = Vector2<double>(tan(FOV.x*0.5), tan(FOV.y*0.5));
    screen_to_radial_factor = tan_half_FOV.elemDiv(image_centre);
    //screen_to_radial_factor = (FOV).elemDiv(image_centre);

    effective_camera_dist_pixels = image_centre.x/tan_half_FOV.x;
}
