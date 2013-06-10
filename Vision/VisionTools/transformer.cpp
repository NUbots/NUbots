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

    head_pitch = head_yaw = body_roll = 0;

    neck_position = Vector3<double>(0,0,0);

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

void Transformer::preCalculateTransforms()
{
    // Reminder for axes
    // X -> Roll
    // Y -> Pitch
    // Z -> Yaw

    // uses the following member variables:
    // head_pitch, head_yaw, body_roll, body_pitch, sensor_calibration

    // Results written to:
    // camVector, camV2RobotRotation

    // Varibles for orientations.
    Vector3<double> camera_orientation = sensor_calibration.m_camera_angle_offset;
    Vector3<double> head_orientation(0.0, 0.0, 0.0);
    Vector2<double> body_orientation = sensor_calibration.m_body_angle_offset;


    // Add the joint values.
    head_orientation.y += head_pitch;
    head_orientation.z += head_yaw;

    body_orientation.x += body_roll;
    body_orientation.y += body_pitch;

    // Make offset vectors.
    Matrix camOffsetVec = Matrix(sensor_calibration.m_camera_position_offset);

    Matrix neckV = Matrix(neck_position);

    // Calculate rotation matrices
    Matrix body_roll_rot = xRotMatrix(body_orientation.x);
    Matrix body_pitch_rot = yRotMatrix(body_orientation.y);

    Matrix head_pitch_rot = yRotMatrix(head_orientation.y);
    Matrix head_yaw_rot = zRotMatrix(head_orientation.z);

    Matrix camera_roll_rot = xRotMatrix(camera_orientation.x);
    Matrix camera_pitch_rot = yRotMatrix(camera_orientation.y);
    Matrix camera_yaw_rot = zRotMatrix(camera_orientation.z);
    Matrix headV2RobotRotation = body_roll_rot * body_pitch_rot * head_yaw_rot * head_pitch_rot;

    camVector = (headV2RobotRotation * camOffsetVec) + neckV;
    camV2RobotRotation = headV2RobotRotation * camera_pitch_rot * camera_roll_rot * camera_yaw_rot;
}

void Transformer::calculateRepresentations(NUPoint& pt, bool known_distance, double val) const
{
    // Calculate the radial position (relative to the camera vector) from the pixel position.
    pt.screenAngular.x = atan( (image_centre.x-pt.screenCartesian.x)  * screen_to_radial_factor.x);
    pt.screenAngular.y = atan( (image_centre.y-pt.screenCartesian.y) * screen_to_radial_factor.y);

    if(known_distance) {
        // In this case val represents known distance (for e.g. found by perspective comparison)
        Matrix image_position_spherical(Vector3<double>(val, pt.screenAngular.x, pt.screenAngular.y));
        Matrix rel_spherical = mathGeneral::Cartesian2Spherical(camV2RobotRotation * mathGeneral::Spherical2Cartesian(image_position_spherical));
        pt.neckRelativeRadial = Vector3<double>(rel_spherical[0][0], rel_spherical[1][0], rel_spherical[2][0]);
    }
    else {
        // In this case val represents known height
        // Calculate the radial position relative to
        pt.neckRelativeRadial = distanceToPoint(pt.screenCartesian, val);
    }

    pt.groundCartesian.x = cos(pt.neckRelativeRadial.z) * cos(pt.neckRelativeRadial.y) * pt.neckRelativeRadial.x;
    pt.groundCartesian.y = cos(pt.neckRelativeRadial.z) * sin(pt.neckRelativeRadial.y) * pt.neckRelativeRadial.x;
}

void Transformer::calculateRepresentations(vector<NUPoint>& pts, bool known_distance, double val) const
{
    BOOST_FOREACH(NUPoint& p, pts) {
        calculateRepresentations(p, known_distance, val);
    }
}

//NUPoint Transformer::calculateRepresentations(const Point& pt, bool ground = true, double val = 0.0) const
//{
//    NUPoint np;
//    np.screenCartesian = pt;
//    calculateRepresentations(np);
//    return np;
//}

//vector<NUPoint> Transformer::calculateRepresentations(const vector<Point>& pts, bool ground = true, double val = 0.0) const
//{
//    vector<NUPoint> nps;
//    BOOST_FOREACH(const Point& p, pts) {
//        nps.push_back(calculateRepresentations(p));
//    }
//    return nps;
//}


///// @note Assumes radial calculation already done
//void Transformer::radial2DToRadial3D(NUPoint &pt, double distance) const
//{
//    Matrix image_position_spherical(Vector3<double>(distance, pt.screenAngular.x, pt.screenAngular.y));
//    Matrix rel_spherical = mathGeneral::Cartesian2Spherical(camV2RobotRotation * mathGeneral::Spherical2Cartesian(image_position_spherical));
//    pt.neckRelativeRadial = Vector3<double>(rel_spherical[0][0], rel_spherical[1][0], rel_spherical[2][0]);
//}

/**
  * Calculates the distance to a point at a given height
  * @param pixel The pixel location in the image relative to the top left of the screen.
  * @param object_height The height of the point to be measured (from the ground).
  * @return A 3 dimensional vector containing the distance, bearing and elevation to the point.
  */
Vector3<double> Transformer::distanceToPoint(Vector2<double> pixel, double object_height) const
{
    Vector3<double> result;

    Matrix vcam(3,1,false);
    vcam[0][0] = effective_camera_dist_pixels;
    vcam[1][0] = image_size.x / 2.0 - pixel.x;
    vcam[2][0] = image_size.y / 2.0 - pixel.y;

    Matrix roboVdir = camV2RobotRotation * vcam;
    double alpha = (object_height - camVector[2][0]) / roboVdir[2][0];
    Matrix v2FieldPoint = alpha * roboVdir + camVector;

    result.x = sqrt(pow(v2FieldPoint[0][0], 2) + pow(v2FieldPoint[1][0], 2) + pow(object_height - (double)camVector[2][0], 2));
    result.y = atan2((double)v2FieldPoint[1][0], (double)v2FieldPoint[0][0]);
    result.z = asin((object_height - (double)camVector[2][0]) / (double)result.x);
    return result;
}

//void Transformer::screenToGroundCartesian(NUPoint& pt) const
//{
//    Vector3<double> v = distanceToPoint(pt.screenCartesian, 0.0);

//    Vector3<double> spherical_foot_relative = Kinematics::TransformPosition(ctgtransform, v);

//    Vector3<double> cartesian_foot_relative = mathGeneral::Spherical2Cartesian(spherical_foot_relative);

//#if VISION_FIELDPOINT_VERBOSITY > 2
//    debug << "Transformer::screenToGroundCartesian - the following should be near zero: " << cartesian_foot_relative.z << endl;
//#endif
//    pt.groundCartesian = Vector2<double>(cartesian_foot_relative.x, cartesian_foot_relative.y);
//}

//void Transformer::screenToGroundCartesian(vector<NUPoint>& pts) const
//{
//    BOOST_FOREACH(NUPoint& p, pts) {
//        screenToGroundCartesian(p);
//    }
//}

//NUPoint Transformer::screenToGroundCartesian(const Point &pt) const
//{
//    NUPoint g;
//    g.screenCartesian = pt;
//    screenToGroundCartesian(g);
//    return g;
//}

//vector<NUPoint> Transformer::screenToGroundCartesian(const vector<Point>& pts) const
//{
//    vector<NUPoint> gpts;
//    BOOST_FOREACH(const Point& p, pts) {
//        gpts.push_back(screenToGroundCartesian(p));
//    }
//    return gpts;
//}

void Transformer::setSensors(double new_head_pitch, double new_head_yaw, double new_body_roll,
                             double new_body_pitch, Vector3<double> new_neck_position)
{
    head_pitch = new_head_pitch;
    head_yaw = new_head_yaw;
    body_pitch = new_body_pitch;
    body_roll = new_body_roll;
    neck_position = new_neck_position;
    preCalculateTransforms();
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

    cout << "Transformer: imagesize: " << image_size <<
            " centre: " << image_centre <<
            " FOV: " << FOV <<
            " tan half fov: " << tan_half_FOV <<
            " effective camera dist pix: " << effective_camera_dist_pixels << endl;

}
