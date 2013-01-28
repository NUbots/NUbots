#include "transformer.h"
#include <boost/foreach.hpp>
#include "visionconstants.h"
#include "Tools/Math/General.h"

Transformer::Transformer()
{
}

/**
  * Applies radial distortion correction to the given pixel location.
  * @param pt The pixel location to correct.
  */
Point Transformer::correctDistortion(const Point& pt)
{
    //get position relative to centre
    Point half_size = image_size*0.5;
    Point centre_relative = pt - half_size;
    //calculate correction factor -> 1+kr^2
    double corr_factor = 1 + VisionConstants::RADIAL_CORRECTION_COEFFICIENT*centre_relative.squareAbs();
    //multiply by factor
    Point result = centre_relative*corr_factor;
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
Point Transformer::screenToRadial2D(Point pt) const {
    Point result;
    result.x = atan( (image_centre.x-pt.x)  * screen_to_radial_factor.x);
    result.y = atan( (image_centre.y-pt.y) * screen_to_radial_factor.y);
    return result;
}

/**
  * Calculates the radial position (relative to the camera vector) from the pixel position for a set of pixels.
  * @param pts The list of pixels.
  */
vector<Point> Transformer::screenToRadial2D(const vector<Point>& pts) const {
    //reimplemented for speed
    vector<Point> result;
    BOOST_FOREACH(const Point& p, pts) {
        result.push_back(screenToRadial2D(p));
    }
    return result;
}

bool Transformer::isDistanceToPointValid() const
{
    return camera_height_valid && camera_pitch_valid && (not VisionConstants::D2P_INCLUDE_BODY_PITCH || body_pitch_valid);
}

/**
  * Calculates the distance to an object at a given point assuming the object is only the same
  * plane as the robots feet. This is useful as the point of contact with the ground for all field
  * objects can easily be identified visually.
  * @param screen_loc The screen position in pixels relative to the top left.
  * @return The distance to the given point relative to the robots feet.
  */
double Transformer::distanceToPoint(Point pixel_loc) const
{
    Point radial = screenToRadial2D(pixel_loc);
    return distanceToPoint(radial.x, radial.y);
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
#if VISION_BLACKBOARD_VERBOSITY > 1
    debug << "VisionBlackboard::distanceToPoint: \n";
    debug << "\t(bearing, elevation): (" << bearing << ", " <<elevation << ")" << endl;
    debug << "\tcalled with point: " << screen_loc << " angle correction: " << VisionConstants::D2P_ANGLE_CORRECTION << endl;
    debug << "\tbody pitch: include:" << VisionConstants::D2P_INCLUDE_BODY_PITCH << " valid: " << body_pitch_valid << " value: " << body_pitch << endl;
    debug << "\tcamera height valid: " << camera_height_valid << " value: " << camera_height << endl;
    debug << "\tcamera pitch valid: " << camera_pitch_valid << " value: " << camera_pitch << endl;
#endif
    double theta,
           distance,
           cos_theta;

    //resultant angle inclusive of camera pitch, pixel elevation and angle correction factor
    theta = mathGeneral::PI*0.5 - camera_pitch + elevation + VisionConstants::D2P_ANGLE_CORRECTION;

    if(VisionConstants::D2P_INCLUDE_BODY_PITCH && body_pitch_valid)
        theta -= body_pitch;

    cos_theta = cos(theta);
    if(cos_theta == 0)
        distance = 0;
    else
        distance = camera_height / cos(theta) / cos(bearing);

#if VISION_BLACKBOARD_VERBOSITY > 1
    debug << "\ttheta: " << theta << " distance: " << distance << " valid: " << valid << endl;
#endif

    return distance;
}

//! @brief Whether the screen to ground vector calculation will give valid information.
bool Transformer::isScreenToGroundValid() const {
    return isDistanceToPointValid();
}

///** Calculates the radial position from the pixel position.
//  * @param pt The pixel location.
//  */
//Vector3<double> Transformer::screenToGroundRadial(Point pt) const
//{
//    Vector3<double> result;
//    Point radial = screenToRadial2D(pt);
//    result.y = radial.x;
//    result.z = radial.y;
//    result.x = distanceToPoint(radial.x, radial.y);
//    return result;
//}

///** Calculates the radial position from the pixel position for a set of pixels.
//  * @param pts The list of pixels.
//  */
//vector<Vector3<double> > Transformer::screenToGroundRadial(const vector<Point>& pts) const
//{
//    //reimplemented for speed
//    //Vector2<double> halfsize(original_image->getWidth()*0.5, original_image->getHeight()*0.5);
//    //Vector2<double> tanhalfangle(tan(m_FOV.x*0.5), tan(m_FOV.y*0.5));
//    vector<Vector3<double> > result;
//    BOOST_FOREACH(const Point& p, pts) {
//        //result.push_back(Point( atan( (halfsize.x - p.x)*tanhalfangle.x/halfsize.x),
//        //                        atan( (halfsize.y - p.y)*tanhalfangle.y/halfsize.y)));
//        //for lack of a better idea
//        result.push_back(screenToGroundRadial(p));
//    }
//    return result;
//}

Point Transformer::screenToGroundCartesian(Point pt) const
{
    Vector2<double> angles = screenToRadial2D(pt);
    double r = distanceToPoint(angles.x, angles.y);
    return Point(r*cos(angles.x), r*cos(angles.y));
}

vector<Point> Transformer::screenToGroundCartesian(const vector<Point>& pts) const
{
    vector<Point> results;
    BOOST_FOREACH(Point p, pts) {
        results.push_back(screenToGroundCartesian(p));
    }
    return results;
}

void Transformer::setKinematicParams(bool cam_pitch_valid, double cam_pitch,
                                     bool cam_height_valid, double cam_height,
                                     bool b_pitch_valid, double b_pitch)
{
    camera_pitch_valid = cam_pitch_valid;
    camera_pitch = cam_pitch;
    camera_height_valid = cam_height_valid;
    camera_height = cam_height;
    body_pitch_valid = b_pitch_valid;
    body_pitch = b_pitch;
}

void Transformer::setCamParams(Vector2<double> imagesize, Vector2<double> fov)
{
    image_size = imagesize;
    image_centre = imagesize*0.5;
    FOV = fov;
    tan_half_FOV = Vector2<double>(tan(FOV.x*0.5), tan(FOV.y*0.5));
    screen_to_radial_factor = tan_half_FOV.elemDiv(image_centre);
    effective_camera_dist_pixels = image_centre.x/tan_half_FOV.x;
}
