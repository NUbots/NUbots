/**
 * @file CameraLens.hpp
 * @brief NUbots-compatible camera projection (ray <-> pixel).
 *
 * A pixel-space wrapper over the NUbots vision pipeline's projection
 * (shared/utility/vision/projection.hpp), so that unit rays in the camera frame
 * {c} recorded in the log can be drawn back onto the source video, and pixels
 * can be unprojected to rays (out-of-field landmark work).
 *
 * Camera frame convention (NUbots): x is the optical axis (viewing direction),
 * y points to the left of the image, z points up. Pixel coordinates have (0,0)
 * at the top-left, x to the right, y down.
 *
 * The lens parameters (focal length, centre offset and distortion coefficients)
 * are normalised by the image width, exactly as in the NUbots camera configs
 * (module/input/Camera/data/config/<robot>/Cameras/Left.yaml,
 * module/platform/Webots/data/config/WebotsCameras/left_camera.yaml) and in
 * message::input::Image::Lens. Prefer fromLens() to take them from the Image
 * message, which is correct for whichever camera is running.
 *
 * The projection model is NOT shared across the cameras this runs on. The real
 * robots wear a Lensagon BF10M19828S118C fisheye (EQUIDISTANT, strongly
 * distorted, 1280x1024), while the simulated camera is a plain 90 deg
 * rectilinear pinhole with no distortion at 640x480 (NUWebots
 * protos/robot/nugus/nugus.proto sets the Camera node's `spherical FALSE` and
 * `fieldOfView 1.5707`, whence focal_length = (640/2)/tan(1.5707/2)/640 = 0.5).
 * Running a webots frame through a fisheye calibration does not merely shift the
 * re-projection, it bends it -- hence projection is carried per lens and taken
 * from the message rather than assumed. The compiled-in defaults are sarah's
 * calibration, for offline replay of the recordings made on that robot.
 */
#ifndef CAMERALENS_HPP
#define CAMERALENS_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "message/input/Image.hpp"

#include "utility/vision/projection.hpp"

namespace utility::slam::camera {

    /**
     * @brief Camera lens model with radial distortion (NUbots-compatible).
     *
     * @note This is *not* covered by Camera.hpp/cpp: `utility::slam::camera::Camera` is the
     * MCHA4400 OpenCV pinhole model (cameraMatrix fx/fy/cx/cy with rational + thin-prism
     * distortion, calibrated from camera.xml), which is a different projection entirely.
     *
     * The projection this class implements *is* already in the tree, as
     * `utility::vision::project`/`unproject` (shared/utility/vision/projection.hpp) — the
     * canonical NUbots implementation. The radial polynomials and the projection models
     * below therefore delegate to it rather than restating them. What this struct adds
     * over calling `utility::vision` directly is a pixel-in/pixel-out API (that utility
     * works in width-normalised units).
     */
    struct CameraLens {
        /// @brief How the incidence angle theta maps to radius on the sensor.
        using Projection = message::input::Image::Lens::Projection;

        double width  = 1280.0;  ///< Image width [px]
        double height = 1024.0;  ///< Image height [px]

        /// Projection model: EQUIDISTANT (r = f*theta) on the fisheyes, RECTILINEAR
        /// (r = f*tan(theta)) on the simulated pinhole, EQUISOLID (r = 2f*sin(theta/2)).
        Projection projection = Projection::EQUIDISTANT;

        // All of the following are normalised by the image width, per the NUbots convention.
        // These defaults are sarah's Left.yaml calibration (the robot that made the data2
        // recording) and exist so offline replay of that recording works with no
        // configuration. Live code should prefer fromLens(), which takes the calibration
        // from the Image message and so is correct for whichever camera is running.
        double focalLength = 0.34690945742400775;                             ///< Normalised focal length
        Eigen::Vector2d centre{0.02072339174622414, -0.0011612242293956145};  ///< Normalised optical-centre offset
        Eigen::Vector2d k{0.38553542593448015, 0.1498415334589703};  ///< Radial distortion coefficients [k1, k2]

        /**
         * @brief Build a lens from a live Image message's calibration.
         *
         * `Image::Lens` already stores focal length and centre normalised by image width,
         * so the fields transfer directly. The projection comes across too: it is what
         * distinguishes a hardware frame from a webots one, and getting it from the
         * message is what makes a single build replay either.
         *
         * @param lens The lens block of a message::input::Image
         * @param dimensions The image dimensions in pixels {width, height}
         */
        static CameraLens fromLens(const message::input::Image::Lens& lens,
                                   const Eigen::Matrix<unsigned int, 2, 1>& dimensions) {
            CameraLens l;
            l.width  = double(dimensions.x());
            l.height = double(dimensions.y());
            // A default-constructed message reads UNKNOWN, which would otherwise fall
            // through to the equidistant branch by accident rather than by decision.
            // Name the fallback: it is the fisheye the hardware wears.
            l.projection  = lens.projection == Projection::UNKNOWN ? Projection(Projection::EQUIDISTANT)
                                                                   : Projection(lens.projection);
            l.focalLength = double(lens.focal_length);
            l.centre      = lens.centre.cast<double>();
            l.k           = lens.k.cast<double>();
            return l;
        }

        /// @brief Map an ideal (undistorted) radius to the distorted radius (used when projecting).
        double distort(double r) const {
            return utility::vision::distort<double>(r, *this);
        }

        /// @brief Map a distorted radius back to the ideal radius (used when unprojecting).
        double undistort(double r) const {
            return utility::vision::undistort<double>(r, *this);
        }

        /// @brief Undistorted radius for an incidence angle, per the projection model.
        double radiusForAngle(double theta) const {
            // Switch on .value: the generated enum wrapper converts implicitly to both its
            // protobuf enum and int, which makes a switch on the wrapper itself ambiguous.
            switch (projection.value) {
                case Projection::RECTILINEAR: {
                    // tan diverges at 90 deg. Clamping just short keeps the radius (and
                    // hence the pixel, and hence the distortion polynomial) finite; the
                    // result lands far outside the image, so inImage() still rejects it.
                    constexpr double max_theta = 89.9 * M_PI / 180.0;
                    return utility::vision::rectilinear::r<double>(std::clamp(theta, 0.0, max_theta), focalLength);
                }
                case Projection::EQUISOLID: return utility::vision::equisolid::r<double>(theta, focalLength);
                default: return utility::vision::equidistant::r<double>(theta, focalLength);
            }
        }

        /// @brief Incidence angle for an undistorted radius, per the projection model.
        double angleForRadius(double r) const {
            switch (projection.value) {
                case Projection::RECTILINEAR: return utility::vision::rectilinear::theta<double>(r, focalLength);
                case Projection::EQUISOLID:
                    // asin is undefined past r = 2f, which is the horizon of this model.
                    return utility::vision::equisolid::theta<double>(std::min(r, 2.0 * focalLength), focalLength);
                default: return utility::vision::equidistant::theta<double>(r, focalLength);
            }
        }

        /**
         * @brief Project a unit ray in {c} to a pixel coordinate (x right, y down).
         * @param ray Unit vector in the camera frame (x optical axis, y left, z up)
         * @return Pixel coordinate; check inFrontOfCamera()/inImage() for validity
         */
        Eigen::Vector2d project(const Eigen::Vector3d& ray) const {
            const double x        = std::clamp(ray.x(), -1.0, 1.0);
            const double theta    = std::acos(x);
            const double sinTheta = std::sqrt(std::max(1.0 - x * x, 1e-12));

            const double rUndist = radiusForAngle(theta);
            const double rDist   = distort(rUndist);  // Normalised distorted radius

            // Screen offset (normalised by width) in left/up axes, then to pixels.
            const double scale      = (sinTheta > 1e-9 ? rDist / sinTheta : 0.0) * width;
            const double screenLeft = scale * ray.y();
            const double screenUp   = scale * ray.z();

            return Eigen::Vector2d(width * 0.5 - screenLeft - centre.x() * width,
                                   height * 0.5 - screenUp - centre.y() * width);
        }

        /**
         * @brief Unproject a pixel coordinate (x right, y down) to a unit ray in {c}.
         */
        Eigen::Vector3d unproject(const Eigen::Vector2d& px) const {
            const double screenLeft = width * 0.5 - px.x() - centre.x() * width;
            const double screenUp   = height * 0.5 - px.y() - centre.y() * width;
            const double rDist      = std::sqrt(screenLeft * screenLeft + screenUp * screenUp) / width;
            if (rDist <= 0.0) {
                return Eigen::Vector3d::UnitX();
            }
            const double rUndist  = undistort(rDist);
            const double theta    = angleForRadius(rUndist);
            const double sinTheta = std::sin(theta);
            const double norm     = std::sqrt(screenLeft * screenLeft + screenUp * screenUp);
            return Eigen::Vector3d(std::cos(theta), sinTheta * screenLeft / norm, sinTheta * screenUp / norm);
        }

        /// @brief True if the ray points into the camera's forward hemisphere.
        static bool inFrontOfCamera(const Eigen::Vector3d& ray) {
            return ray.x() > 1e-3;
        }

        /// @brief True if a pixel lies within the image bounds.
        bool inImage(const Eigen::Vector2d& px) const {
            return px.x() >= 0.0 && px.x() < width && px.y() >= 0.0 && px.y() < height;
        }

        /**
         * @brief Half of the horizontal field of view [rad].
         *
         * Measured, not declared: the angle off the optical axis of the ray that
         * unprojects from the middle of the left image edge, so it follows whatever
         * projection model and distortion this calibration carries. The two in use
         * are nowhere near each other -- 86.7 deg for the NUbots fisheye against
         * 45.0 deg for the webots pinhole -- so anything drawing a viewing wedge has
         * to ask rather than assume.
         */
        double horizontalHalfFov() const {
            const Eigen::Vector3d edge = unproject(Eigen::Vector2d(0.0, height * 0.5 - centre.y() * width));
            return std::atan2(std::hypot(edge.y(), edge.z()), edge.x());
        }
    };

}  // namespace utility::slam::camera

#endif
